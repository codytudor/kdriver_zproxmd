/*
 * Proximity & Motion Detector Core
 *
 * Copyright 2015 Tudor Design Systems, LLC.
 *
 * Author: Cody Tudor <cody.tudor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * This driver is a re-implementation of the IMX serial driver
 * found in ./drivers/tty/serial/imx.c; functions and general
 * flow were used as a skeleton for the serial operations. 
 * 
 * This driver requires a device tree entry for a UART with the compatible
 * string "fsl,zproxmd-sensor" and is a 'polling' type driver. The intent
 * is to provide /proc entries for configuring and polling the ZILOG proximity
 * module for motion and proximity detection. 
 *
 */

#include "zproxmd.h"          /* Contains function prototypes and the like...*/

#define DRIVER_NAME     "zproxmd-sensor"
#define ZPROXMD_MAJOR   207
#define ZPROXMD_MINOR   42
#define DEV_NAME        "prox"
#define NUM_SENSORS 1
char *portname = "/dev/prox0";

/* /proc fs directory entry pointers */
struct proc_dir_entry *root_proc_entry;
struct proc_dir_entry *range;
struct proc_dir_entry *sensitivity;
struct proc_dir_entry *time_remaining;
struct proc_dir_entry *motion_detected;
struct proc_dir_entry *version;
struct proc_dir_entry *freq_resp;
struct proc_dir_entry *cmdmode;

/* For storing the active uart_port */
static struct zproxmd_port *zproxmd_ports[NUM_SENSORS]; // We only support a single sensor

/*  Function Protoypes for uart_ops structure */
static void zproxmd_uart_stop_tx( struct uart_port *port );
static unsigned int zproxmd_uart_get_mctrl( struct uart_port *port );
static void zproxmd_uart_set_mctrl( struct uart_port *port, unsigned int mctrl );
static void zproxmd_uart_start_tx( struct uart_port *port );
static void zproxmd_uart_stop_rx( struct uart_port *port );
static unsigned int zproxmd_uart_tx_empty( struct uart_port *port );
static void zproxmd_uart_break_ctl( struct uart_port *port, int break_state );
static void zproxmd_uart_enable_ms( struct uart_port *port );
static int zproxmd_uart_startup( struct uart_port *port );
static void zproxmd_uart_shutdown( struct uart_port *port );
static void zproxmd_uart_set_termios( struct uart_port *port, struct ktermios *termios, struct ktermios *old );
static const char *zproxmd_uart_type( struct uart_port *port );
static void zproxmd_uart_release_port( struct uart_port *port );
static int zproxmd_uart_request_port( struct uart_port *port );
static void zproxmd_uart_config_port( struct uart_port *port, int flags );
static int zproxmd_uart_verify_port( struct uart_port *port, struct serial_struct *ser );
static int configure_sensor( void );

/* Interrupt space function prototypes */
static irqreturn_t zproxmd_interrupt_rx( int irq, void *dev_id );
static irqreturn_t zproxmd_interrupt_tx( int irq, void *dev_id );
static irqreturn_t zproxmd_interrupt_handler( int irq, void *dev_id );

static struct platform_driver serial_zproxmd_driver;

struct file* uart_filp;

struct ring_buffer {
    char buf[BUFFER_MAX_SIZE];
    int head;
    int tail;   
}rx_circ_buffer;

struct zproxmd_port {
    struct uart_port    port;
    struct timer_list   timer;
    unsigned int        old_status;
    int                 txirq, rxirq;
    unsigned int        have_rtscts:1;
    unsigned int        dte_mode:1;
    unsigned int        irda_inv_rx:1;
    unsigned int        irda_inv_tx:1;
    unsigned short      trcv_delay; /* transceiver delay */
    struct clk          *clk_ipg;
    struct clk          *clk_per;
    const void          *devdata;
};

static struct platform_device_id serial_zproxmd_devtype[] = {
    { .name = DRIVER_NAME,}, 
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, serial_zproxmd_devtype);

static struct of_device_id zproxmd_dt_ids[] = {
    { .compatible = "fsl,zproxmd-sensor",},
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, zproxmd_dt_ids);

static struct uart_driver zproxmd_reg = {
    .owner          = THIS_MODULE,
    .driver_name    = DRIVER_NAME,
    .dev_name       = DEV_NAME,
    .major          = ZPROXMD_MAJOR,
    .minor          = ZPROXMD_MINOR,
    .nr             = NUM_SENSORS,
    .cons           = NULL,
};

static inline void remove_proc_entries ( void )
{
    remove_proc_subtree( "range", root_proc_entry );
    remove_proc_subtree( "sensitivity", root_proc_entry );
    remove_proc_subtree( "time_remaining", root_proc_entry );
    remove_proc_subtree( "motion_detected", root_proc_entry );
    remove_proc_subtree( "version", root_proc_entry );
    remove_proc_subtree( "freq_resp", root_proc_entry );
    remove_proc_subtree( "cmdmode", root_proc_entry );
    remove_proc_entry( "zproxmd", NULL ); 
    root_proc_entry = NULL; 
}

void clear_rx_circ_buf( void )
{
    int i;
    for( i = 0; i < BUFFER_MAX_SIZE; i++ ) {
        rx_circ_buffer.buf[i] = 0;
    }
    rx_circ_buffer.head = rx_circ_buffer.tail = 0;  
}

void uart_read_value( char *rx_buffer, const int len )
{
    unsigned int count;
     
    for( count = 0; count < len; count ++ ) {
        rx_buffer[count] = rx_circ_buffer.buf[rx_circ_buffer.tail];
        rx_circ_buffer.tail = rx_circ_buffer.tail + 1;
        if( rx_circ_buffer.tail > ( BUFFER_MAX_SIZE - 1 ) )
            rx_circ_buffer.tail = 0;
    }
    
}

void uart_write_value( const unsigned char *tx_buffer, int buffer_length )
{ 
    struct zproxmd_port *sport = zproxmd_ports[0];
    struct circ_buf *xmit = &sport->port.state->xmit;
    int c;
    
    if( sport == NULL )
        goto err_out;
    
    if( xmit == NULL )
        goto err_out_2;
    
    local_irq_disable();
       
    while( 1 ) {
        c = CIRC_SPACE_TO_END( xmit->head, xmit->tail, UART_XMIT_SIZE );
        if( buffer_length < c )
            c = buffer_length;
        if( c <= 0 )
            break;
        memcpy( xmit->buf + xmit->head, tx_buffer, c );
        xmit->head = ( xmit->head + c ) & ( UART_XMIT_SIZE - 1 );
        tx_buffer += c;
        buffer_length -= c;
    }

    zproxmd_uart_start_tx( (struct uart_port * ) sport );

    local_irq_enable();
   
    return;
    
err_out:
    pr_err( "sport was NULL\n" );
    return;
    
err_out_2:
    pr_err( "xmit was NULL\n" );
}

/*
 * Handle any change of modem status signal since we were last called.
 */
static void zproxmd_mctrl_check( struct zproxmd_port *sport )
{
    unsigned int status, changed;

    status = sport->port.ops->get_mctrl( &sport->port );
    changed = status ^ sport->old_status;

    if( changed == 0 )
        return;

    sport->old_status = status;

    if( changed & TIOCM_RI )
        sport->port.icount.rng++;
    if( changed & TIOCM_DSR )
        sport->port.icount.dsr++;

    wake_up_interruptible( &sport->port.state->port.delta_msr_wait );
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void zproxmd_timeout( unsigned long data )
{
    struct zproxmd_port *sport = (struct zproxmd_port *)data;
    unsigned long flags;

    if( sport->port.state ) {
        spin_lock_irqsave( &sport->port.lock, flags );
        zproxmd_mctrl_check( sport );
        spin_unlock_irqrestore( &sport->port.lock, flags );

        mod_timer( &sport->timer, jiffies + MCTRL_TIMEOUT );
    }
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int zproxmd_uart_tx_empty( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned int retval;
    
    //pr_info( "zproxmd_uart_tx_empty\n" );

    retval = ( readl( sport->port.membase + USR2 ) & USR2_TXDC ) ?  TIOCSER_TEMT : 0;

    return retval;
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int zproxmd_uart_get_mctrl( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned int retval = TIOCM_DSR | TIOCM_CAR;
    
    //pr_info( "zproxmd_uart_get_mctrl\n" );

    if( readl( sport->port.membase + USR1 ) & USR1_RTSS )
        retval |= TIOCM_CTS;

    if( readl( sport->port.membase + UCR2 ) & UCR2_CTS )
        retval |= TIOCM_RTS;

    if( readl( sport->port.membase + 0xb4 ) & UTS_LOOP )
        retval |= TIOCM_LOOP;

    return retval;
}

static void zproxmd_uart_set_mctrl( struct uart_port *port, unsigned int mctrl )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long temp;
    
    //pr_info( "zproxmd_uart_set_mctrl\n" );

    temp = readl( sport->port.membase + UCR2 ) & ~UCR2_CTS;

    if( mctrl & TIOCM_RTS )
        temp |= UCR2_CTS;

    writel( temp, sport->port.membase + UCR2 );

    temp = readl( sport->port.membase + 0xb4 ) & ~UTS_LOOP;
    if( mctrl & TIOCM_LOOP )
        temp |= UTS_LOOP;
    writel( temp, sport->port.membase + 0xb4 );
}

/*
 * Interrupts always disabled.
 */
static void zproxmd_uart_break_ctl( struct uart_port *port, int break_state )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long flags, temp;
    
    //pr_info( "zproxmd_uart_break_ctl\n" );

    spin_lock_irqsave( &sport->port.lock, flags );
    
    temp = readl( sport->port.membase + UCR1 ) & ~UCR1_SNDBRK;

    if( break_state != 0 )
        temp |= UCR1_SNDBRK;

    writel( temp, sport->port.membase + UCR1 );

    spin_unlock_irqrestore( &sport->port.lock, flags );
}

static int zproxmd_uart_startup( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    int retval;
    unsigned long flags, temp;
    
    //pr_info( "zproxmd_uart_startup\n" );
    
    zproxmd_ports[0] = ( struct zproxmd_port * ) port;

    retval = clk_prepare_enable( sport->clk_per );
    if( retval )
        goto error_out1;
        
    retval = clk_prepare_enable(sport->clk_ipg);
    if( retval ) {
        clk_disable_unprepare( sport->clk_per );
        goto error_out1;
    }
    
    /* set receiver / transmitter trigger level */
    temp = readl( sport->port.membase + UFCR ) & ( UFCR_RFDIV | UFCR_DCEDTE );
    temp |= 2 << UFCR_TXTL_SHF | 1;
    writel( temp, sport->port.membase + UFCR );

    /* disable the DREN bit (Data Ready interrupt enable) before
     * requesting IRQs
     */
    temp = readl( sport->port.membase + UCR4 );

    /* set the trigger level for CTS */
    temp &= ~( UCR4_CTSTL_MASK << UCR4_CTSTL_SHF );
    temp |= CTSTL << UCR4_CTSTL_SHF;

    writel( temp & ~UCR4_DREN, sport->port.membase + UCR4 );

    /*
     * Allocate the IRQ(s) i.MX1 has three interrupts whereas later
     * chips only have one interrupt.
     */
    if( sport->txirq > 0 ) {
        retval = request_irq(sport->rxirq, zproxmd_interrupt_rx, 0, DRIVER_NAME, sport);
        if( retval )
            goto error_out1;

        retval = request_irq(sport->txirq, zproxmd_interrupt_tx, 0, DRIVER_NAME, sport);
        if( retval )
            goto error_out2;
            
    } else {
        retval = request_irq(sport->port.irq, zproxmd_interrupt_handler, 0, DRIVER_NAME, sport);
        if( retval ) {
            free_irq( sport->port.irq, sport );
            goto error_out1;
        }
    }

    spin_lock_irqsave( &sport->port.lock, flags );
    /*
     * Finally, clear and enable interrupts
     */
    writel( USR1_RTSD, sport->port.membase + USR1 );

    temp = readl( sport->port.membase + UCR1 );
    temp |= UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN;

    writel( temp, sport->port.membase + UCR1 );

    temp = readl( sport->port.membase + UCR2 );
    temp |= (UCR2_RXEN | UCR2_TXEN);
    if( !sport->have_rtscts )
        temp |= UCR2_IRTS;
    writel( temp, sport->port.membase + UCR2 );

    temp = readl( sport->port.membase + UCR3 );
    temp |= IMX21_UCR3_RXDMUXSEL | UCR3_ADNIMP;
    writel( temp, sport->port.membase + UCR3 );

    /*
     * Enable modem status interrupts
     */
    zproxmd_uart_enable_ms( &sport->port );
    spin_unlock_irqrestore( &sport->port.lock, flags );

    return 0;

error_out2:
    if( sport->rxirq )
        free_irq( sport->rxirq, sport );
        
error_out1:
    return retval;
}

static void zproxmd_uart_shutdown( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long temp;
    unsigned long flags;
    
    //pr_info( "zproxmd_uart_shutdown\n" );

    spin_lock_irqsave( &sport->port.lock, flags );
    temp = readl( sport->port.membase + UCR2 );
    temp &= ~( UCR2_TXEN );
    writel( temp, sport->port.membase + UCR2 );
    spin_unlock_irqrestore( &sport->port.lock, flags );

    /*
     * Stop our timer.
     */
    del_timer_sync( &sport->timer );

    /*
     * Free the interrupts
     */
    if( sport->txirq > 0 ) {
        free_irq( sport->txirq, sport );
        free_irq( sport->rxirq, sport );
    } else
        free_irq( sport->port.irq, sport );

    /*
     * Disable all interrupts, port and break condition.
     */

    spin_lock_irqsave( &sport->port.lock, flags );
    temp = readl( sport->port.membase + UCR1 );
    temp &= ~( UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN );

    writel( temp, sport->port.membase + UCR1 );
    spin_unlock_irqrestore( &sport->port.lock, flags );

    clk_disable_unprepare( sport->clk_per );
    clk_disable_unprepare( sport->clk_ipg );
}

static void zproxmd_uart_set_termios( struct uart_port *port, struct ktermios *termios, struct ktermios *old )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long flags;
    unsigned int ucr2, old_ucr1, old_txrxen, baud, quot;
    unsigned int div, ufcr;
    unsigned long num, denom;
    uint64_t tdiv64;
    
 /*  The settings we wish to implement on the uart
 *     
    tty.c_cflag &= ( B9600 | CS8 | CLOCAL | CREAD ); 
    tty.c_cflag &= ~( PARENB | PARODD );                 // shut off parity 
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB; 
    
    tty.c_iflag = 0;
    tty.c_iflag |= ( IGNBRK | IGNPAR );                  // ignore break & parity on the input

    tty.c_oflag = 0;                                     // no remapping, no delays 
    tty.c_oflag |= OPOST;
    tty.c_lflag = 0;                                     // no canonical processing
                                                         // no signaling chars, no echo,
    tty.c_cc[VTIME] = 2;                                 // 200ms read timeout
    tty.c_cc[VMIN] = 0;                                  // read doesn't block 
*/
    
    //pr_info( "zproxmd_uart_set_termios\n" );

    termios->c_cflag |= ( B9600 | CS8 | CLOCAL | CREAD );     // 9600 Baud, 8 Data bits, enable read
    ucr2 = UCR2_WS | UCR2_SRST | UCR2_IRTS;

    termios->c_cflag &= ~CRTSCTS;
    termios->c_cflag &= ~CSTOPB;                              // one stop bit only
    termios->c_cflag &= ~( PARENB | PARODD );                 // shut off parity 

    del_timer_sync( &sport->timer );

    /*
     * Ask the core to calculate the divisor for us.
     */
    baud = uart_get_baud_rate( port, termios, old, 50, port->uartclk / 16 );
    quot = uart_get_divisor( port, baud );

    spin_lock_irqsave( &sport->port.lock, flags );

    sport->port.ignore_status_mask = 0;
    
    termios->c_iflag |= ( IGNBRK | IGNPAR );                  // ignore break & parity on the input
    sport->port.ignore_status_mask |= URXD_PRERR;
    sport->port.ignore_status_mask |= URXD_BRK;
    sport->port.ignore_status_mask |= URXD_OVRRUN;
    
    termios->c_lflag &= ~ECHO;
    termios->c_lflag &= ~ICANON;

    /*
     * Update the per-port timeout.
     */
    uart_update_timeout( port, termios->c_cflag, baud );

    /*
     * disable interrupts and drain transmitter
     */
    old_ucr1 = readl( sport->port.membase + UCR1 );
    writel( old_ucr1 & ~( UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN ), sport->port.membase + UCR1 );

    while( !( readl( sport->port.membase + USR2 ) & USR2_TXDC ) )
        barrier();

    /* then, disable everything */
    old_txrxen = readl( sport->port.membase + UCR2 );
    writel( old_txrxen & ~( UCR2_TXEN | UCR2_RXEN ), sport->port.membase + UCR2 );
    old_txrxen &= ( UCR2_TXEN | UCR2_RXEN );

    /* custom-baudrate handling */
    div = sport->port.uartclk / ( baud * 16 );
    if( baud == 38400 && quot != div )
        baud = sport->port.uartclk / ( quot * 16 );

    div = sport->port.uartclk / ( baud * 16 );
    if( div > 7 )
        div = 7;
    if( !div )
        div = 1;

    rational_best_approximation( 16 * div * baud, sport->port.uartclk, 1 << 16, 1 << 16, &num, &denom );

    tdiv64 = sport->port.uartclk;
    tdiv64 *= num;
    do_div( tdiv64, denom * 16 * div );
    tty_termios_encode_baud_rate( termios, (speed_t) tdiv64, (speed_t) tdiv64 );

    num -= 1;
    denom -= 1;

    ufcr = readl( sport->port.membase + UFCR );
    ufcr = ( ufcr & ( ~UFCR_RFDIV ) ) | UFCR_RFDIV_REG( div );
    if( sport->dte_mode )
        ufcr |= UFCR_DCEDTE;
    writel( ufcr, sport->port.membase + UFCR );

    writel( num, sport->port.membase + UBIR );
    writel( denom, sport->port.membase + UBMR );

    writel( sport->port.uartclk / div / 1000, sport->port.membase + IMX21_ONEMS );

    writel( old_ucr1, sport->port.membase + UCR1 );

    /* set the parity, stop bits and data size */
    writel( ucr2 | old_txrxen, sport->port.membase + UCR2 );

    if( UART_ENABLE_MS( &sport->port, termios->c_cflag ) )
        zproxmd_uart_enable_ms( &sport->port );

    spin_unlock_irqrestore( &sport->port.lock, flags );
}

static const char *zproxmd_uart_type( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    
    //pr_info( "zproxmd_uart_type\n" );

    return sport->port.type == PORT_ZPROXMD ? "ZMOTION Sensor" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void zproxmd_uart_release_port( struct uart_port *port )
{
    struct platform_device *pdev = to_platform_device( port->dev );
    struct resource *mmres;
    
    //pr_info( "zproxmd_uart_release_port\n" );

    mmres = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    release_mem_region( mmres->start, resource_size( mmres ) );
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int zproxmd_uart_request_port( struct uart_port *port )
{
    struct platform_device *pdev = to_platform_device( port->dev );
    struct resource *mmres;
    void *retval;
    
    //pr_info( "zproxmd_uart_request_port\n" );

    mmres = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    if( !mmres )
        return -ENODEV;

    retval = request_mem_region( mmres->start, resource_size( mmres ), "zproxmd-sensor" );

    return  retval ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void zproxmd_uart_config_port( struct uart_port *port, int flags )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    
    //pr_info( "zproxmd_uart_config_port\n" );

    if( flags & UART_CONFIG_TYPE && zproxmd_uart_request_port( &sport->port ) == 0 ) {
        sport->port.type = PORT_ZPROXMD;
        zproxmd_ports[0]->port = sport->port;
    }
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_IMX and PORT_UNKNOWN
 */
static int zproxmd_uart_verify_port( struct uart_port *port, struct serial_struct *ser )
{
    struct zproxmd_port *sport = (struct zproxmd_port *) port;
    int retval = 0;
    
    //pr_info( "zproxmd_uart_verify_port\n" );

    if( ser->type != PORT_UNKNOWN && ser->type != PORT_ZPROXMD )
        retval = -EINVAL;
    if( sport->port.irq != ser->irq )
        retval = -EINVAL;
    if( ser->io_type != UPIO_MEM )
        retval = -EINVAL;
    if( sport->port.uartclk / 16 != ser->baud_base )
        retval = -EINVAL;
    if( sport->port.mapbase != (unsigned long) ser->iomem_base )
        retval = -EINVAL;
    if( sport->port.iobase != ser->port )
        retval = -EINVAL;
    if( ser->hub6 != 0 )
        retval = -EINVAL;
    return retval;
}

/*
 * Set the modem control timer to fire immediately.
 */
static void zproxmd_uart_enable_ms( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;

    mod_timer( &sport->timer, jiffies );
}

static void zproxmd_transmit_buffer( struct zproxmd_port *sport )
{
    struct circ_buf *xmit = &sport->port.state->xmit;
    
    //pr_info( "zproxmd_transmit_buffer\n" );

    while( !uart_circ_empty( xmit ) && !( readl( sport->port.membase + 0xb4 ) & UTS_TXFULL ) ) {
        /* send xmit->buf[xmit->tail]
         * out the port here but do not send CR or LF for compatibility*/
        if( ( xmit->buf[xmit->tail] != 0x0d ) && ( xmit->buf[xmit->tail] != 0x0a ) )
            writel( xmit->buf[xmit->tail], sport->port.membase + URTX0 );
            //printk( KERN_INFO "Sending the character: %c\n", xmit->buf[xmit->tail] );
        xmit->tail = ( xmit->tail + 1 ) & ( UART_XMIT_SIZE - 1 );
        sport->port.icount.tx++;
    }

    if( uart_circ_chars_pending( xmit ) < WAKEUP_CHARS )
        uart_write_wakeup( &sport->port );

    if( uart_circ_empty( xmit ) )
        zproxmd_uart_stop_tx( &sport->port );
}

/*
 * interrupts disabled on entry
 */
static void zproxmd_uart_start_tx( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long temp;

    /* Clear any pending ORE flag before enabling interrupt */
    temp = readl( sport->port.membase + USR2 );
    writel( temp | USR2_ORE, sport->port.membase + USR2 );

    temp = readl( sport->port.membase + UCR4 );
    temp |= UCR4_OREN;
    writel( temp, sport->port.membase + UCR4 );

    temp = readl( sport->port.membase + UCR1 );
    writel( temp | UCR1_TXMPTYEN, sport->port.membase + UCR1 );

    if( readl( sport->port.membase + 0xb4 ) & UTS_TXEMPTY ) {
        zproxmd_transmit_buffer( sport );
    }
}

static irqreturn_t zproxmd_interrupt_tx( int irq, void *dev_id )
{
    struct zproxmd_port *sport = dev_id;
    struct circ_buf *xmit = &sport->port.state->xmit;
    unsigned long flags;

    spin_lock_irqsave( &sport->port.lock, flags );
 
    if( uart_circ_empty( xmit ) || uart_tx_stopped( &sport->port ) ) {
        zproxmd_uart_stop_tx( &sport->port );
        goto out;
    }

    zproxmd_transmit_buffer( sport );

    if( uart_circ_chars_pending( xmit ) < WAKEUP_CHARS )
        uart_write_wakeup( &sport->port );

out:
    spin_unlock_irqrestore( &sport->port.lock, flags );
    return IRQ_HANDLED;
}

static irqreturn_t zproxmd_interrupt_rx( int irq, void *dev_id )
{
    struct zproxmd_port *sport = dev_id;
    unsigned int rx, flg, ignored = 0;
    struct tty_port *port = &sport->port.state->port;
    unsigned long flags, temp;

    spin_lock_irqsave( &sport->port.lock, flags );

    while( readl( sport->port.membase + USR2 ) & USR2_RDR ) {
        flg = TTY_NORMAL;
        sport->port.icount.rx++;

        rx = readl( sport->port.membase + URXD0 );

        temp = readl( sport->port.membase + USR2 );
        
        //printk( KERN_INFO "Received the character: %c\n", ( unsigned char ) rx );
        
        rx_circ_buffer.buf[rx_circ_buffer.head] = ( char ) rx;
        rx_circ_buffer.head = rx_circ_buffer.head + 1;
        if( rx_circ_buffer.head == rx_circ_buffer.tail )
            rx_circ_buffer.tail = rx_circ_buffer.tail + 1;
        if( rx_circ_buffer.tail > ( BUFFER_MAX_SIZE - 1 ) )
            rx_circ_buffer.tail = 0;
        if( rx_circ_buffer.head > ( BUFFER_MAX_SIZE - 1 ) )
            rx_circ_buffer.head = 0;
        
        if( uart_handle_sysrq_char( &sport->port, ( unsigned char ) rx ) )
            continue;

        if( unlikely( rx & URXD_ERR ) ) {
            if( rx & URXD_BRK )
                sport->port.icount.brk++;
            else if( rx & URXD_PRERR )
                sport->port.icount.parity++;
            else if( rx & URXD_FRMERR )
                sport->port.icount.frame++;
            if( rx & URXD_OVRRUN )
                sport->port.icount.overrun++;

            if( rx & sport->port.ignore_status_mask ) {
                if( ++ignored > 100 )
                    goto out;
                continue;
            }

            rx &= ( sport->port.read_status_mask | 0xFF );

            if( rx & URXD_BRK )
                flg = TTY_BREAK;
            else if( rx & URXD_PRERR )
                flg = TTY_PARITY;
            else if( rx & URXD_FRMERR )
                flg = TTY_FRAME;
            if( rx & URXD_OVRRUN )
                flg = TTY_OVERRUN;

        }

        tty_insert_flip_char( port, rx, flg );
    }

out:
    spin_unlock_irqrestore( &sport->port.lock, flags );
    tty_flip_buffer_push( port );
    return IRQ_HANDLED;
}

static irqreturn_t zproxmd_interrupt_handler( int irq, void *dev_id )
{
    struct zproxmd_port *sport = dev_id;
    unsigned int sts;
    unsigned int sts2;

    sts = readl( sport->port.membase + USR1 );

    if( sts & USR1_RRDY )
        zproxmd_interrupt_rx( irq, dev_id );

    if( sts & USR1_TRDY && readl( sport->port.membase + UCR1 ) & UCR1_TXMPTYEN )
        zproxmd_interrupt_tx( irq, dev_id );

    if( sts & USR1_AWAKE )
        writel( USR1_AWAKE, sport->port.membase + USR1 );

    sts2 = readl( sport->port.membase + USR2 );
    if( sts2 & USR2_ORE ) {
        dev_err( sport->port.dev, "Rx FIFO overrun\n" );
        sport->port.icount.overrun++;
        writel( sts2 | USR2_ORE, sport->port.membase + USR2 );
    }

    return IRQ_HANDLED;
}

/*
 * interrupts disabled on entry
 */
static void zproxmd_uart_stop_tx( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long temp;

    temp = readl( sport->port.membase + UCR1 );
    writel( temp & ~UCR1_TXMPTYEN, sport->port.membase + UCR1 );
}

/*
 * interrupts disabled on entry
 */
static void zproxmd_uart_stop_rx( struct uart_port *port )
{
    struct zproxmd_port *sport = ( struct zproxmd_port * ) port;
    unsigned long temp;

    temp = readl( sport->port.membase + UCR2 );
    writel( temp & ~UCR2_RXEN, sport->port.membase + UCR2 );
}

static int configure_sensor( void )
{
    char local_buffer[BUFFER_MAX_SIZE];
    int retval = 0;
    
    /* First we need to set the serial interface command mode  */
    uart_write_value( (char*) CMD_SER_INTERFACE_WRITE, strlen( CMD_SER_INTERFACE_WRITE ) );
    
    mdelay(20);
    uart_read_value( local_buffer, 1 );

    if( local_buffer[0] == 0x41 ) {
        pr_info( "zproxmd: sensor already in ASCII mode... setting again anyways!\n" );
    }
    
    uart_write_value( (char*) SERIAL_COMMAND_MODE, strlen( SERIAL_COMMAND_MODE ) );
    
    mdelay(20);
    local_buffer[0] = (unsigned char) 0;
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] != SENSOR_ACK) {
        pr_err( "zproxmd: sensor did not respond with ACK\n" );
        retval = -1;
        return retval;
    }
    
    /* Second we set the default range setting  */
    uart_write_value( CMD_RANGE_CONTROL_WRITE, strlen( CMD_RANGE_CONTROL_WRITE ) );
    
    mdelay(20);
    uart_read_value( local_buffer, 3 );
    
    uart_write_value( DEF_RANGE_SETTING, strlen( DEF_RANGE_SETTING ) ) ;   
    
    mdelay(20);
    local_buffer[0] = (unsigned char) 0;
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] != SENSOR_ACK) {
        pr_err( "zproxmd: sensor did not respond with ACK\n" );
        retval = -1;
    }
    
    /* Third we set the default sensitivity setting  */
    uart_write_value( CMD_SENS_WRITE, strlen( CMD_SENS_WRITE ) );
    
    mdelay(20);
    uart_read_value( local_buffer, 3 );
    
    uart_write_value( DEF_SENSITIVITY_SETTING, strlen( DEF_SENSITIVITY_SETTING ) );
    
    mdelay(20);
    local_buffer[0] = (unsigned char) 0;
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] != SENSOR_ACK) {
        pr_err( "zproxmd: sensor did not respond with ACK\n" );
        retval = -1;
    }
    
    /* Finally we set the default frequency response setting  */
    uart_write_value( CMD_FREQ_RESP_WRITE, strlen( CMD_FREQ_RESP_WRITE ) );
    
    mdelay(20);
    uart_read_value( local_buffer, 1 );
        
    uart_write_value( DEF_FREQ_RESP_SETTING, strlen( DEF_FREQ_RESP_SETTING ) ) ;
    
    mdelay(20);
    local_buffer[0] = (unsigned char) 0;
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] != SENSOR_ACK) {
        pr_err( "zproxmd: sensor did not respond with ACK\n" );
        retval = -1;
    }
    
    return 0;
}

/*
 *  Define the basic serial functions we support.
*/

static struct uart_ops zproxmd_serial_ops = {
    
    .tx_empty       = zproxmd_uart_tx_empty,
    .set_mctrl      = zproxmd_uart_set_mctrl,
    .get_mctrl      = zproxmd_uart_get_mctrl,
    .stop_tx        = zproxmd_uart_stop_tx,
    .start_tx       = zproxmd_uart_start_tx,
    .stop_rx        = zproxmd_uart_stop_rx,
    .enable_ms      = zproxmd_uart_enable_ms,
    .break_ctl      = zproxmd_uart_break_ctl,
    .startup        = zproxmd_uart_startup,
    .shutdown       = zproxmd_uart_shutdown,
    .set_termios    = zproxmd_uart_set_termios,
    .type           = zproxmd_uart_type,
    .release_port   = zproxmd_uart_release_port,
    .request_port   = zproxmd_uart_request_port,
    .config_port    = zproxmd_uart_config_port,
    .verify_port    = zproxmd_uart_verify_port,
};


static int serial_zproxmd_probe( struct platform_device *pdev )
{
    struct zproxmd_port *sport;
    struct zproxmd_platform_data *pdata;
    void __iomem *base;
    int retval = 0;
    struct resource *res;
    struct device_node *np = pdev->dev.of_node;
    const struct of_device_id *of_id = of_match_device( zproxmd_dt_ids, &pdev->dev );

    sport = devm_kzalloc( &pdev->dev, sizeof( *sport ), GFP_KERNEL );
    if( !sport )
        return -ENOMEM;
    
    retval = of_alias_get_id(np, "serial");
    
    if( retval < 0 ) {
        dev_err( &pdev->dev, "failed to get alias id, errno %d\n", retval );
        return retval;
    }
    sport->port.line = retval;

    sport->devdata = of_id->data;

    res = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    if( !res )
        return -ENODEV;

    base = devm_ioremap( &pdev->dev, res->start, PAGE_SIZE );
    if( !base ) 
        return -ENOMEM;

    sport->port.dev         = &pdev->dev;
    sport->port.mapbase     = res->start;
    sport->port.membase     = base;
    sport->port.type        = PORT_ZPROXMD,
    sport->port.iotype      = UPIO_MEM;
    sport->port.irq         = platform_get_irq(pdev, 0);
    sport->rxirq            = platform_get_irq(pdev, 0);
    sport->txirq            = platform_get_irq(pdev, 1);
    sport->port.fifosize    = 32;
    sport->port.ops         = &zproxmd_serial_ops;
    sport->port.flags       = UPF_BOOT_AUTOCONF;
    init_timer(&sport->timer);
    sport->timer.function   = zproxmd_timeout;
    sport->timer.data       = ( unsigned long ) sport;

    sport->clk_ipg = devm_clk_get( &pdev->dev, "ipg" );
    
    if( IS_ERR( sport->clk_ipg ) ) {
        retval = PTR_ERR( sport->clk_ipg );
        dev_err(&pdev->dev, "failed to get ipg clk: %d\n", retval);
        return retval;
    }

    sport->clk_per = devm_clk_get( &pdev->dev, "per" );
    
    if( IS_ERR( sport->clk_per ) ) {
        retval = PTR_ERR(sport->clk_per);
        dev_err(&pdev->dev, "failed to get per clk: %d\n", retval);
        return retval;
    }

    sport->port.uartclk = clk_get_rate( sport->clk_per );

    zproxmd_ports[0] = sport;

    pdata = dev_get_platdata( &pdev->dev );
    
    if( pdata && pdata->init ) {
        retval = pdata->init( pdev );
        if( retval )
            return retval;
    }

    retval = uart_add_one_port( &zproxmd_reg, &sport->port );
    if( retval )
        goto deinit;
        
    platform_set_drvdata( pdev, sport );

    return 0;
    
deinit:
    if( pdata && pdata->exit )
        pdata->exit( pdev );
    return retval;
}

static int serial_zproxmd_remove( struct platform_device *pdev )
{
    struct zproxmd_platform_data *pdata; 
    struct zproxmd_port *sport = platform_get_drvdata( pdev );
    
    pdata = dev_get_platdata( &pdev->dev );

    uart_remove_one_port( &zproxmd_reg, &sport->port );

    if( pdata && pdata->exit )
        pdata->exit( pdev );

    return 0;
}

static struct platform_driver serial_zproxmd_driver = {
    .probe      = serial_zproxmd_probe,
    .remove     = serial_zproxmd_remove,
    .id_table   = serial_zproxmd_devtype,
    .driver     = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = zproxmd_dt_ids,
    },
};

static int __init zproxmd_init( void )
{
    int retval = 0;
   
    pr_info( "Loading ZMOTION kernel object driver...\n" );
    
    retval = uart_register_driver( &zproxmd_reg );
    if( retval < 0 ) 
        goto out;
        
    retval = platform_driver_register( &serial_zproxmd_driver );
    if( retval < 0 ) {
        uart_unregister_driver( &zproxmd_reg );
        goto out;
    }
    
    // Create the top level /proc entry as a directory
    root_proc_entry = NULL;
    root_proc_entry = proc_mkdir( "zproxmd", NULL );
        
    if( root_proc_entry == NULL ) {
        retval = -ENOMEM;
        pr_err( "zproxmd: Couldn't create zproxmd proc entry\n" );
    } else {
        // Create our /proc entries for reading and writing to the device under the /proc/zproxmd directory
        range = proc_create( "range", 0644, root_proc_entry, &zproxmd_range_fops );
        sensitivity = proc_create( "sensitivity", 0644, root_proc_entry, &zproxmd_sensitivity_fops );
        time_remaining = proc_create( "time_remaining", 0644, root_proc_entry, &zproxmd_time_remaining_fops );
        motion_detected = proc_create( "motion_detected", 0444, root_proc_entry, &zproxmd_motion_detected_fops );
        version = proc_create( "version", 0444, root_proc_entry, &zproxmd_version_fops );
        freq_resp = proc_create( "freq_resp", 0644, root_proc_entry, &zproxmd_fresp_fops );
        
        mdelay( 200 );
        uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 );       
        if( configure_sensor() < 0 ) {
            pr_err( "zproxmd: error configuring sensor\n" );
            remove_proc_entries();
            retval = -ENOMEM;
        }
        
        rx_circ_buffer.head = rx_circ_buffer.tail = 0;
        filp_close( uart_filp, 0 );
    }

out:   
    if( retval < 0 )
        pr_err( "Error loading ZPROXMD LKM\n" );
    return 0;
}

static void __exit zproxmd_cleanup( void )
{
    char local_buffer[BUFFER_MAX_SIZE];
    int i = 0;
    
    pr_info( "Unloading ZMOTION kernel object driver...\n" );
    uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 );
    
    pr_info( "Resetting ZMOTION PIR Sensor...\n" );
    uart_write_value( (char*) CMD_RESET_REQUEST, strlen( CMD_RESET_REQUEST ) );
    
    mdelay(20);
    local_buffer[0] = (unsigned char) 0;
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] == SENSOR_ACK) {
        for( i = 0; i < 4; i++ ) {
            uart_write_value( (char*) &RESET_CMD_SEQ[i], strlen( &RESET_CMD_SEQ[i] ) );
            mdelay(10);
        }
        local_buffer[0] = (unsigned char) 0;
        uart_read_value( local_buffer, 1 );
    }
    
    filp_close( uart_filp, 0 );
    mdelay(100);

    platform_driver_unregister( &serial_zproxmd_driver );
    mdelay(100);
    
    uart_unregister_driver( &zproxmd_reg );
    
    if( root_proc_entry != NULL )
        remove_proc_entries();
}

/* Declare entry and exit functions */
module_init( zproxmd_init );
module_exit( zproxmd_cleanup );

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cody Tudor <cody.tudor@gmail.com>");
MODULE_DESCRIPTION("Z8FS040 Proximity & Motion Detection Module Driver");

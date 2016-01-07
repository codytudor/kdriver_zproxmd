/*
 * Proximity & Motion Detector Read functions
 *
 * Copyright 2015 Tudor Design Systems, LLC.
 *
 * Author: Cody Tudor <cody.tudor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "zproxmd.h"

/* Local global vars */
static char global_resp_buf[BUFFER_MAX_SIZE];
static char procfs_input_buffer[BUFFER_MAX_SIZE];

/* Private var types */
typedef enum {
    READ_VERSION = 0,
    READ_TIME_REMAINING,
    READ_MOTION_DETECTION,
    READ_RANGE,
    READ_SENSITIVITY,
    READ_FREQ_RESPONSE,
}read_func_t;

typedef enum {
    WRITE_RANGE = 0,
    WRITE_SENSITIVITY,
    WRITE_FREQ_RESPONSE,
    WRITE_TIME_DELAY,
}write_func_t;

/* Private function protoypes */
static int read_function_handler( const read_func_t *cmd );
static int write_function_handler( const write_func_t *cmd ); 
static void clear_buffer( char *buf );
static void trim_to_local_buf( char *new_buf, char *old_buf );
static int validate_entry( const write_func_t *cmd );

static ssize_t range_setting_read( struct file *file, char *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_RANGE;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    clear_buffer( local_buffer );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t sensitivity_read( struct file *file, char *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_SENSITIVITY;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    clear_buffer( local_buffer );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t freq_response_read( struct file *file, char *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_FREQ_RESPONSE;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    clear_buffer( local_buffer );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t version_read( struct file *file, char *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_VERSION;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t motion_detection_read( struct file *file, char __user *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_MOTION_DETECTION;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    clear_buffer( local_buffer );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t time_remaining_read( struct file *file, char *buffer, size_t length, loff_t * offset ) 
{
    read_func_t read_cmd = READ_TIME_REMAINING;
    char local_buffer[BUFFER_MAX_SIZE];
    static int finished = 0;
    
    if( finished ) {
        finished = 0;
        return 0;
    }
    
    /* We only want to loop once per read */
    finished = 1;       
    clear_buffer( global_resp_buf );
    clear_buffer( local_buffer );
    
    if( read_function_handler( &read_cmd ) < 0 )
        strcpy( local_buffer, "invalid read_func\n" );
    else 
        trim_to_local_buf( local_buffer, global_resp_buf );
    
    filp_close( uart_filp, 0 );
    
    if( copy_to_user( buffer, local_buffer, strlen( local_buffer ) ) )
        return -EFAULT;
        
    return strlen( local_buffer );
}

static ssize_t range_setting_write( struct file *file, const char *buffer, size_t len, loff_t * off )
{
    write_func_t write_cmd = WRITE_RANGE;
    unsigned long buffer_size = 0;
    
    if( len > BUFFER_MAX_SIZE ) {
        buffer_size = BUFFER_MAX_SIZE;
    }
    else {
        buffer_size = len;
    }
    
    clear_buffer( procfs_input_buffer );
    
    if( copy_from_user( procfs_input_buffer, buffer, buffer_size ) ) {
        pr_err( "zproxmd: error copying command from user space\n" );
        return buffer_size;
    }
        
    if( strlen( procfs_input_buffer ) > 0 )
        procfs_input_buffer[strlen( procfs_input_buffer ) - 1] = (unsigned char) 0;
    else 
        procfs_input_buffer[0] = (unsigned char) 0;
    
    if( validate_entry( &write_cmd ) < 0 ) {
        pr_err( "zproxmd: error validating command\n" );
        return buffer_size;
    }

    if( write_function_handler( &write_cmd ) < 0 )      
        pr_err( "range_setting_write: write exited with non-zero code\n" );
    
    return buffer_size;
}

static ssize_t sensitivity_write( struct file *file, const char *buffer, size_t len, loff_t * off )
{
    write_func_t write_cmd = WRITE_SENSITIVITY;
    unsigned long buffer_size = 0;
    
    if( len > BUFFER_MAX_SIZE ) {
        buffer_size = BUFFER_MAX_SIZE;
    }
    else {
        buffer_size = len;
    }
    
    clear_buffer( procfs_input_buffer );
    
    if( copy_from_user( procfs_input_buffer, buffer, buffer_size ) ) {
        pr_err( "zproxmd: error copying command from user space\n" );
        return buffer_size;
    }
        
    if( strlen( procfs_input_buffer ) > 0 )
        procfs_input_buffer[strlen( procfs_input_buffer ) - 1] = (unsigned char) 0;
    else 
        procfs_input_buffer[0] = (unsigned char) 0;
    
    if( validate_entry( &write_cmd ) < 0 ) {
        pr_err( "zproxmd: error validating command\n" );
        return buffer_size;
    }

    if( write_function_handler( &write_cmd ) < 0 )      
        pr_err( "sensitivity_write: write exited with non-zero code\n" );
    
    return buffer_size;
}

static ssize_t freq_response_write( struct file *file, const char *buffer, size_t len, loff_t * off )
{
    write_func_t write_cmd = WRITE_FREQ_RESPONSE;
    unsigned long buffer_size = 0;
    
    if( len > BUFFER_MAX_SIZE ) {
        buffer_size = BUFFER_MAX_SIZE;
    }
    else {
        buffer_size = len;
    }
    
    clear_buffer( procfs_input_buffer );
    
    if( copy_from_user( procfs_input_buffer, buffer, buffer_size ) ) {
        pr_err( "zproxmd: error copying command from user space\n" );
        return buffer_size;
    }
    
    if( strlen( procfs_input_buffer ) > 0 )
        procfs_input_buffer[strlen( procfs_input_buffer ) - 1] = (unsigned char) 0;
    else 
        procfs_input_buffer[0] = (unsigned char) 0;
    
    if( validate_entry( &write_cmd ) < 0 ) {
        pr_err( "zproxmd: error validating command\n" );
        return buffer_size;
    }
    
    if( write_function_handler( &write_cmd ) < 0 )      
        pr_err( "freq_response_write: write exited with non-zero code\n" );
    
    return buffer_size;
}

static ssize_t time_delay_write( struct file *file, const char *buffer, size_t len, loff_t * off )
{
    write_func_t write_cmd = WRITE_TIME_DELAY;
    unsigned long buffer_size = 0;
    
    if( len > BUFFER_MAX_SIZE ) {
        buffer_size = BUFFER_MAX_SIZE;
    }
    else {
        buffer_size = len;
    }
    
    clear_buffer( procfs_input_buffer );
    
    if( copy_from_user( procfs_input_buffer, buffer, buffer_size ) ) {
        pr_err( "zproxmd: error copying command from user space\n" );
        return buffer_size;
    }
        
    if( strlen( procfs_input_buffer ) > 0 )
        procfs_input_buffer[strlen( procfs_input_buffer ) - 1] = (unsigned char) 0;
    else 
        procfs_input_buffer[0] = (unsigned char) 0;
    
    if( validate_entry( &write_cmd ) < 0 ) {
        pr_err( "zproxmd: error validating command\n" );
        return buffer_size;
    }  
        
    if( write_function_handler( &write_cmd ) < 0 )      
        pr_err( "time_delay_write: write exited with non-zero code\n" );
   
    return buffer_size;
}

static int procfs_open( struct inode *inode, struct file *file )
{
    try_module_get(THIS_MODULE);
    return 0;
}

static int procfs_close( struct inode *inode, struct file *file )
{
    module_put(THIS_MODULE);
    return 0;
}

const struct file_operations zproxmd_range_fops = {
    .owner = THIS_MODULE,
    .read = range_setting_read,    
    .write = range_setting_write,
    .open = procfs_open,
    .release = procfs_close,
};

const struct file_operations zproxmd_sensitivity_fops = {
    .owner = THIS_MODULE,
    .read = sensitivity_read,    
    .write = sensitivity_write,
    .open = procfs_open,
    .release = procfs_close,
};

const struct file_operations zproxmd_fresp_fops = {
    .owner = THIS_MODULE,
    .read = freq_response_read,    
    .write = freq_response_write,
    .open = procfs_open,
    .release = procfs_close,
};

const struct file_operations zproxmd_version_fops = {
    .owner = THIS_MODULE,
    .read = version_read,
    .open = procfs_open,
    .release = procfs_close,
};

const struct file_operations zproxmd_motion_detected_fops = {
    .owner = THIS_MODULE,
    .read = motion_detection_read,
    .open = procfs_open,
    .release = procfs_close,
};

const struct file_operations zproxmd_time_remaining_fops = {
    .owner = THIS_MODULE,
    .read = time_remaining_read,
    .write = time_delay_write,
    .open = procfs_open,
    .release = procfs_close,
};

static int read_function_handler( const read_func_t *cmd )
{
    int read_length;
    
    switch( *cmd ) {
        case READ_VERSION:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_VERSION_READ, 1 );
            read_length = 6;
            break;
               
        case READ_TIME_REMAINING:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_MD_OUT_STATE_READ, 1 );
            read_length = 3;
            break;
        
        case READ_MOTION_DETECTION:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_MD_STATUS_READ, 1 );
            read_length = 1;
            break;
            
        case READ_RANGE:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_RANGE_CONTROL_READ, 1 );
            read_length = 3;
            break;
            
        case READ_SENSITIVITY:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_SENS_READ, 1 );
            read_length = 3;
            break;
            
        case READ_FREQ_RESPONSE:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( ( unsigned char * ) CMD_FREQ_RESP_READ, 1 );
            read_length = 1;
            break;
        
        default:
            pr_err( "zproxmd: unrecognized read_func\n" );
            return -EBADRQC;
            break;
        
    };
    
    mdelay(20);
    
    uart_read_value( global_resp_buf, read_length );
    
    return 0;  
}

static int write_function_handler( const write_func_t *cmd )
{  
    char local_buffer[BUFFER_MAX_SIZE];
    int retval = 0;
    int read_length;
    
    clear_buffer( local_buffer );
    
    switch( *cmd ) {
        case WRITE_RANGE:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( CMD_RANGE_CONTROL_WRITE, strlen( CMD_RANGE_CONTROL_WRITE ) );
            read_length = 3;
            break;
            
        case WRITE_FREQ_RESPONSE:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( CMD_FREQ_RESP_WRITE, strlen( CMD_FREQ_RESP_WRITE ) );
            read_length = 1;
            break;
            
        case WRITE_SENSITIVITY:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( CMD_SENS_WRITE, strlen( CMD_SENS_WRITE ) );
            read_length = 3;
            break;
            
        case WRITE_TIME_DELAY:
            uart_filp = filp_open( portname, O_RDWR | O_NOCTTY, 0 ); 
            clear_rx_circ_buf();
            uart_write_value( CMD_DELAY_TIME_WRITE, strlen( CMD_DELAY_TIME_WRITE ) );
            read_length = 3;
            break;    
            
        default:
            pr_err( "zproxmd: unrecognized write_func\n" );
            return -EBADRQC;        
            break;
        
    };
    
    mdelay(20);
    uart_read_value( local_buffer, read_length );
    
    clear_buffer( local_buffer );
    trim_to_local_buf( local_buffer, procfs_input_buffer );
    
    uart_write_value( local_buffer, strlen( local_buffer ) );
    
    mdelay(20);
    clear_buffer( local_buffer );
    uart_read_value( local_buffer, 1 );
    
    if( local_buffer[0] != SENSOR_ACK) {
        pr_err( "zproxmd: sensor did not respond with ACK\n" );
        retval = -1;
    }
    
    return retval;    
}

static int validate_entry( const write_func_t *cmd )
{
    int expected_length = 0;
    int retval = 0;
    long result = 0;
    
    switch( *cmd ) {
        case WRITE_FREQ_RESPONSE:
            expected_length = 1;
            if( strlen( procfs_input_buffer ) != expected_length ) {
                pr_err( "zproxmd: string too long or empty, expecting 'H' or 'L'" );
                retval = -EBADRQC;
            }
            else {
                if( procfs_input_buffer[0] != 'H' && procfs_input_buffer[0] != 'L' ) {
                    pr_err( "zproxmd: invalid setting, expecting 'H' or 'L'" );
                    retval = -EBADRQC;
                }
            }
            break;
            
        case WRITE_RANGE:
            expected_length = 3;
            expected_length = 3;
            if( strlen( procfs_input_buffer ) != expected_length ) {
                pr_err( "zproxmd: string too long or too short/empty, expecting three digit decimal value" );
                retval = -EBADRQC;
            }
            else {
                retval = kstrtol( procfs_input_buffer, 10, &result );
                if( ( result < 0 || result > 7 ) || ( retval > 0 ) ) {
                    pr_err( "zproxmd: invalid setting, expecting decimal value between 000 ~ 007" );
                    retval = -EBADRQC;
                }
            }
            break;
            
        case WRITE_SENSITIVITY:
            expected_length = 3;
            expected_length = 3;
            if( strlen( procfs_input_buffer ) != expected_length ) {
                pr_err( "zproxmd: string too long or too short/empty, expecting three digit decimal value" );
                retval = -EBADRQC;
            }
            else {
                retval = kstrtol( procfs_input_buffer, 10, &result );
                if( ( result < 0 || result > 255 ) || ( retval > 0 ) ) {
                    pr_err( "zproxmd: invalid setting, expecting decimal value between 000 ~ 255" );
                    retval = -EBADRQC;
                }
            }
            break;
            
        case WRITE_TIME_DELAY:
            expected_length = 3;
            if( strlen( procfs_input_buffer ) != expected_length ) {
                pr_err( "zproxmd: string too long or too short/empty, expecting three digit decimal value" );
                retval = -EBADRQC;
            }
            else {
                retval = kstrtol( procfs_input_buffer, 10, &result );
                if( ( result < 0 || result > 255 ) || ( retval > 0 ) ) {
                    pr_err( "zproxmd: invalid setting, expecting decimal value between 000 ~ 255" );
                    retval = -EBADRQC;
                }
            }
            break;
            
        default:
            pr_err( "zproxmd: unrecognized write_func\n" );
            return -EBADRQC; 
            break;
        
    };
    
    return retval;
    
}

static void clear_buffer( char *buf )
{
    int i;
    for( i = 0; i < BUFFER_MAX_SIZE; i++ ) {
        buf[i] = (unsigned char) 0;
    }   
}

static void trim_to_local_buf( char *new_buf, char *old_buf )
{
    int i;
    
    for( i = 0; i < BUFFER_MAX_SIZE; i++ ) {
        if( old_buf[i] < '/' ) {
            new_buf[i] = '\n';
            break;
        }
        else {
            new_buf[i] = old_buf[i];
        }
    }
}

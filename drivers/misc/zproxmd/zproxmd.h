/*
 * Proximity & Motion Detector header
 *
 * Copyleft 2016 Tudor Design Systems, LLC.
 *
 * Author: Cody Tudor <cody.tudor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ZPROXMD_H_INCLUDED
#define __ZPROXMD_H_INCLUDED

#include <linux/device.h>
#include <linux/kernel.h>     /* This is a linux kernel driver*/
#include <linux/module.h>     /* This will be a .ko */
#include <linux/string.h>     /* Used for string literals*/
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/fs.h>         /* Used for the filp_open, filp_close, vfs_read, etc. functions */ 

#include <asm/irq.h>
#include <linux/platform_data/zproxmd-serial.h>
#include <linux/platform_data/dma-imx.h>

/* Register definitions */
#define URXD0                 0x0  /* Receiver Register */
#define URTX0                 0x40 /* Transmitter Register */
#define UCR1                  0x80 /* Control Register 1 */
#define UCR2                  0x84 /* Control Register 2 */
#define UCR3                  0x88 /* Control Register 3 */
#define UCR4                  0x8c /* Control Register 4 */
#define UFCR                  0x90 /* FIFO Control Register */
#define USR1                  0x94 /* Status Register 1 */
#define USR2                  0x98 /* Status Register 2 */
#define UBIR                  0xa4 /* BRM Incremental Register */
#define UBMR                  0xa8 /* BRM Modulator Register */
#define IMX21_ONEMS           0xb0 /* One Millisecond register */

/* UART Control Register Bit Fields.*/
#define URXD_ERR              (1<<14)
#define URXD_OVRRUN           (1<<13)
#define URXD_FRMERR           (1<<12)
#define URXD_BRK              (1<<11)
#define URXD_PRERR            (1<<10)
#define UCR1_TRDYEN           (1<<13) /* Transmitter ready interrupt enable */
#define UCR1_IDEN             (1<<12) /* Idle condition interrupt */
#define UCR1_ICD_REG(x)       (((x) & 3) << 10) /* idle condition detect */
#define UCR1_RRDYEN           (1<<9)  /* Recv ready interrupt enable */
#define UCR1_RDMAEN           (1<<8)  /* Recv ready DMA enable */
#define UCR1_IREN             (1<<7)  /* Infrared interface enable */
#define UCR1_TXMPTYEN         (1<<6)  /* Transimitter empty interrupt enable */
#define UCR1_RTSDEN           (1<<5)  /* RTS delta interrupt enable */
#define UCR1_SNDBRK           (1<<4)  /* Send break */
#define UCR1_TDMAEN           (1<<3)  /* Transmitter ready DMA enable */
#define UCR1_ATDMAEN          (1<<2)  /* Aging DMA Timer Enable */
#define UCR1_DOZE             (1<<1)  /* Doze */
#define UCR1_UARTEN           (1<<0)  /* UART enabled */
#define UCR2_ESCI             (1<<15) /* Escape seq interrupt enable */
#define UCR2_IRTS             (1<<14) /* Ignore RTS pin */
#define UCR2_CTSC             (1<<13) /* CTS pin control */
#define UCR2_CTS              (1<<12) /* Clear to send */
#define UCR2_ESCEN            (1<<11) /* Escape enable */
#define UCR2_PREN             (1<<8)  /* Parity enable */
#define UCR2_PROE             (1<<7)  /* Parity odd/even */
#define UCR2_STPB             (1<<6)  /* Stop */
#define UCR2_WS               (1<<5)  /* Word size */
#define UCR2_RTSEN            (1<<4)  /* Request to send interrupt enable */
#define UCR2_ATEN             (1<<3)  /* Aging Timer Enable */
#define UCR2_TXEN             (1<<2)  /* Transmitter enabled */
#define UCR2_RXEN             (1<<1)  /* Receiver enabled */
#define UCR2_SRST             (1<<0)  /* SW reset */
#define UCR3_DTREN            (1<<13) /* DTR interrupt enable */
#define UCR3_PARERREN         (1<<12) /* Parity enable */
#define UCR3_FRAERREN         (1<<11) /* Frame error interrupt enable */
#define UCR3_DSR              (1<<10) /* Data set ready */
#define UCR3_DCD              (1<<9)  /* Data carrier detect */
#define UCR3_RI               (1<<8)  /* Ring indicator */
#define UCR3_ADNIMP           (1<<7)  /* Autobaud Detection Not Improved */
#define UCR3_RXDSEN           (1<<6)  /* Receive status interrupt enable */
#define UCR3_AIRINTEN         (1<<5)  /* Async IR wake interrupt enable */
#define UCR3_AWAKEN           (1<<4)  /* Async wake interrupt enable */
#define IMX21_UCR3_RXDMUXSEL  (1<<2)  /* RXD Muxed Input Select */
#define UCR3_INVT             (1<<1)  /* Inverted Infrared transmission */
#define UCR3_BPEN             (1<<0)  /* Preset registers enable */
#define UCR4_CTSTL_SHF        10  /* CTS trigger level shift */
#define UCR4_CTSTL_MASK       0x3F    /* CTS trigger is 6 bits wide */
#define UCR4_INVR             (1<<9)  /* Inverted infrared reception */
#define UCR4_ENIRI            (1<<8)  /* Serial infrared interrupt enable */
#define UCR4_WKEN             (1<<7)  /* Wake interrupt enable */
#define UCR4_REF16            (1<<6)  /* Ref freq 16 MHz */
#define UCR4_IDDMAEN          (1<<6)  /* DMA IDLE Condition Detected */
#define UCR4_IRSC             (1<<5)  /* IR special case */
#define UCR4_TCEN             (1<<3)  /* Transmit complete interrupt enable */
#define UCR4_BKEN             (1<<2)  /* Break condition interrupt enable */
#define UCR4_OREN             (1<<1)  /* Receiver overrun interrupt enable */
#define UCR4_DREN             (1<<0)  /* Recv data ready interrupt enable */
#define UFCR_RXTL_SHF         0   /* Receiver trigger level shift */
#define UFCR_DCEDTE           (1<<6)  /* DCE/DTE mode select */
#define UFCR_RFDIV            (7<<7)  /* Reference freq divider mask */
#define UFCR_RFDIV_REG(x)     (((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF         10  /* Transmitter trigger level shift */
#define USR1_PARITYERR        (1<<15) /* Parity error interrupt flag */
#define USR1_RTSS             (1<<14) /* RTS pin status */
#define USR1_TRDY             (1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RTSD             (1<<12) /* RTS delta */
#define USR1_ESCF             (1<<11) /* Escape seq interrupt flag */
#define USR1_FRAMERR          (1<<10) /* Frame error interrupt flag */
#define USR1_RRDY             (1<<9)   /* Receiver ready interrupt/dma flag */
#define USR1_TIMEOUT          (1<<7)   /* Receive timeout interrupt status */
#define USR1_RXDS             (1<<6)  /* Receiver idle interrupt flag */
#define USR1_AIRINT           (1<<5)  /* Async IR wake interrupt flag */
#define USR1_AWAKE            (1<<4)  /* Aysnc wake interrupt flag */
#define USR2_ADET             (1<<15) /* Auto baud rate detect complete */
#define USR2_TXFE             (1<<14) /* Transmit buffer FIFO empty */
#define USR2_DTRF             (1<<13) /* DTR edge interrupt flag */
#define USR2_IDLE             (1<<12) /* Idle condition */
#define USR2_IRINT            (1<<8)  /* Serial infrared interrupt flag */
#define USR2_WAKE             (1<<7)  /* Wake */
#define USR2_RTSF             (1<<4)  /* RTS edge interrupt flag */
#define USR2_TXDC             (1<<3)  /* Transmitter complete */
#define USR2_BRCD             (1<<2)  /* Break condition */
#define USR2_ORE              (1<<1)   /* Overrun error */
#define USR2_RDR              (1<<0)   /* Recv data ready */
#define UTS_LOOP              (1<<12)  /* Loop tx and rx */
#define UTS_TXEMPTY           (1<<6)  /* TxFIFO empty */
#define UTS_RXEMPTY           (1<<5)  /* RxFIFO empty */
#define UTS_TXFULL            (1<<4)  /* TxFIFO full */

/* Application settings defines */
#define BUFFER_MAX_SIZE             512
#define DEF_RANGE_SETTING           "007"
#define DEF_FREQ_RESP_SETTING       "H"
#define DEF_SENSITIVITY_SETTING     "255"
#define SERIAL_COMMAND_MODE         "A"
#define RESET_CMD_SEQ               "1234"
#define SENSOR_ACK                  0x06
#define SENSOR_NACK                 0x15
#define MCTRL_TIMEOUT               (250*HZ/1000)
#define CTSTL                       16

/* Serial Command list for the ZEPIR0BAS02MODG module */
#define CMD_MD_STATUS_READ          "a"
#define CMD_LG_AMBIENT_READ         "b"
#define CMD_MD_CONFIG_STATUS_READ   "c"
#define CMD_MD_CONFIG_STATUS_WRITE  "C"
#define CMD_DELAY_TIME_READ         "d"
#define CMD_DELAY_TIME_WRITE        "D"
#define CMD_HYPERSENSE_READ         "e"
#define CMD_HYPERSENSE_WRITE        "E"
#define CMD_FREQ_RESP_READ          "f"
#define CMD_FREQ_RESP_WRITE         "F"
#define CMD_HYPER_LEVEL_READ        "g"
#define CMD_HYPER_LEVEL_WRITE       "G"
#define CMD_MD_SUSPEND_READ         "h"
#define CMD_MD_SUSPEND_WRITE        "H"
#define CMD_VERSION_READ            "i"
#define CMD_SER_INTERFACE_READ      "k"
#define CMD_SER_INTERFACE_WRITE     "K"
#define CMD_LG_THRESH_READ          "l"
#define CMD_LG_THRESH_WRITE         "L"
#define CMD_REAL_TIME_MD_READ       "m"
#define CMD_REAL_TIME_MD_WRITE      "M"
#define CMD_MD_OUT_STATE_READ       "o"
#define CMD_MD_OUT_STATE_WRITE      "O"
#define CMD_PING_READ               "p"
#define CMD_PING_WRITE              "P"
#define CMD_RAM_READ                "q"
#define CMD_RAM_WRITE               "Q"
#define CMD_RANGE_CONTROL_READ      "r"
#define CMD_RANGE_CONTROL_WRITE     "R"
#define CMD_SENS_READ               "s"
#define CMD_SENS_WRITE              "S"
#define CMD_DUAL_DIRECTION_READ     "u"
#define CMD_DUAL_DIRECTION_WRITE    "U"
#define CMD_DIRECTION_READ          "v"
#define CMD_DIRECTION_WRITE         "V"
#define CMD_RESET_REQUEST           "X"
#define CMD_SLEEP_TIME_READ         "y"
#define CMD_SLEEP_TIME_WRITE        "Y"
#define CMD_SLEEP_REQUEST           "Z"

#endif  /* __ZPROXMD_H_INCLUDED */

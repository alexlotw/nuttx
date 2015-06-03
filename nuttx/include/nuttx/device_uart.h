/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_ARM_DEVICE_UART_H
#define __ARCH_ARM_DEVICE_UART_H

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/ring_buf.h>

#define DEVICE_TYPE_UART_HW                     "UART"

/* Baudrate settings */
#define BAUD_115200     115200
#define BAUD_57600      57600
#define BAUD_38400      38400
#define BAUD_19200      19200
#define BAUD_9600       9600
#define BAUD_4800       4800
#define BAUD_2400       2400
#define BAUD_1800       1800

/* Parity */
#define NO_PARITY       0x00        /* No Parity */
#define ODD_PARITY      0x01        /* Odd Parity */
#define EVEN_PARITY     0x02        /* Even Parity */
#define MARK_PARITY     0x03        /* Mark Parity */
#define SPACE_PARITY    0x04        /* Space Parity */

/* Stopbits */
#define ONE_STOP_BIT    0x00        /* 1 Stop Bit */
#define ONE5_STOP_BITS  0x01        /* 1.5 Stop Bit */
#define TWO_STOP_BITS   0x02        /* 2 Stop bit */

/* Modem control */
#define MCR_DTR         (1<<0)      /* Data Terminal Ready*/
#define MCR_RTS         (1<<1)      /* Request to Send */
#define MCR_OUT1        (1<<2)      /* Out 1 */
#define MCR_OUT2        (1<<3)      /* Out 2 */
#define MCR_LPBK        (1<<4)      /* Loop */

/* Line status */
#define LSR_DR          (1<<0)      /* Data Ready */
#define LSR_OE          (1<<1)      /* Overrun Error */
#define LSR_PE          (1<<2)      /* Parity Error */
#define LSR_FE          (1<<3)      /* Framing Error */
#define LSR_BI          (1<<4)      /* Break Interrupt */
#define LSR_THRE        (1<<5)      /* Transmitter Holding Register */
#define LSR_TEMT        (1<<6)      /* Transmitter Empty */
#define LSR_RXFE        (1<<7)      /* Error in RCVR FIFO */

/* Modem status */
#define MSR_DCTS        (1<<0)      /* Delta Clear to Send */
#define MSR_DDSR        (1<<1)      /* Delta Data Set Ready */
#define MSR_TERI        (1<<2)      /* Trailing Edge Ring Indicator */
#define MSR_DDCD        (1<<3)      /* Delta Data Carrier Detect */
#define MSR_CTS         (1<<4)      /* Clear to Send */
#define MSR_DSR         (1<<5)      /* Data Set Ready */
#define MSR_RI          (1<<6)      /* Ring Indicator */
#define MSR_DCD         (1<<7)      /* Data Carrier Detect */

/**
 * struct device_uart_type_ops - UART device driver operations
 *
 * @param set_configuration: UART set_configuration() function pointer
 * @param get_modem_ctrl: UART get_modem_ctrl() function pointer
 * @param set_modem_ctrl: UART set_modem_ctrl() function pointer
 * @param get_modem_status: UART get_modem_status() function pointer
 * @param get_line_status: UART get_line_status() function pointer
 * @param set_break: UART set_break() function pointer
 * @param attach_ms_callback: UART attach_ms_callback() function pointer
 * @param attach_ls_callback: UART attach_ls_callback() function pointer
 * @param start_transmitter: UART start_transmitter() function pointer
 * @param stop_transmitter: UART stop_transmitter() function pointer
 * @param start_receiver: UART start_receiver() function pointer
 * @param stop_receiver: UART stop_receiver() function pointer
 */
struct device_uart_type_ops {
    int (*set_configuration)(struct device *dev, int baud, int parity,
                             int databits, int stopbit, int flow);
    int (*get_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    int (*set_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    int (*get_modem_status)(struct device *dev, uint8_t *modem_status);
    int (*get_line_status)(struct device *dev, uint8_t *line_status);
    int (*set_break)(struct device *dev, uint8_t break_on);
    int (*attach_ms_callback)(struct device *dev, void (*callback)(uint8_t ms));
    int (*attach_ls_callback)(struct device *dev, void (*callback)(uint8_t ls));
    int (*start_transmitter)(struct device *dev, uint8_t *buffer, int length,
                             void *dma, int *sent,
                             void (*callback)(uint8_t *buffer, int length,
                                              int error));
    int (*stop_transmitter)(struct device *dev);
    int (*start_receiver)(struct device *dev, uint8_t*buffer, int length,
                          void *dma, int *got,
                          void (*callback)(uint8_t *buffer, int length,
                                           int error));
    int (*stop_receiver)(struct device *dev);
};


/**
 * @brief UART set_configuration function
 */
static inline int device_uart_set_configuration(struct device *dev,
                                                int baud, int parity,
                                                int databits, int stopbit,
                                                int flow)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->set_configuration)
        return dev->driver->ops->type_ops.uart->
                    set_configuration(dev, baud, parity, databits, stopbit,
                                      flow);

    return -EOPNOTSUPP;
}


/**
 * @brief UART get_modem_ctrl function
 */
static inline int device_uart_get_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->get_modem_ctrl)
        return dev->driver->ops->type_ops.uart->
                    get_modem_ctrl(dev, modem_ctrl);

    return -EOPNOTSUPP;
}


/**
 * @brief UART set_modem_ctrl function
 */
static inline int device_uart_set_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->set_modem_ctrl)
        return dev->driver->ops->type_ops.uart->
                    set_modem_ctrl(dev, modem_ctrl);

    return -EOPNOTSUPP;
}


/**
 * @brief UART get_modem_status function
 */
static inline int device_uart_get_modem_status(struct device *dev,
                                               uint8_t *modem_status)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->get_modem_status)
        return dev->driver->ops->type_ops.uart->
                    get_modem_status(dev, modem_status);

    return -EOPNOTSUPP;
}


/**
 * @brief UART get_line_status function
 */
static inline int device_uart_get_line_status(struct device *dev,
                                              uint8_t *line_status)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->get_line_status)
        return dev->driver->ops->type_ops.uart->get_line_status(dev,
                                                                line_status);

    return -EOPNOTSUPP;
}


/**
 * @brief UART set_break function
 */
static inline int device_uart_set_break(struct device *dev, uint8_t break_on)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->set_break)
        return dev->driver->ops->type_ops.uart->set_break(dev, break_on);

    return -EOPNOTSUPP;
}


/**
 * @brief UART attach_ms_callback function
 */
static inline int device_uart_attach_ms_callback(struct device *dev,
                                                 void (*callback)(uint8_t ms))
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->attach_ms_callback)
        return dev->driver->ops->type_ops.uart->
                    attach_ms_callback(dev, callback);

    return -EOPNOTSUPP;
}


/**
 * @brief UART attach_ls_callback function
 */
static inline int device_uart_attach_ls_callback(struct device *dev,
                                                 void (*callback)(uint8_t ls))
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->attach_ls_callback)
        return dev->driver->ops->type_ops.uart->
                    attach_ls_callback(dev, callback);

    return -EOPNOTSUPP;
}


/**
 * @brief UART start_transmitter function
 */
static inline int device_uart_start_transmitter(struct device *dev,
                        uint8_t *buffer, int length, void *dma,
                        int *sent, void (*callback)(uint8_t *buffer, int length,
                                                    int error))
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->start_transmitter)
        return dev->driver->ops->type_ops.uart->
                    start_transmitter(dev, buffer, length, dma, sent, callback);

    return -EOPNOTSUPP;
}


/**
 * @brief UART stop_transmitter function
 */
static inline int device_uart_stop_transmitter(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->stop_transmitter)
        return dev->driver->ops->type_ops.uart->stop_transmitter(dev);

    return -EOPNOTSUPP;
}


/**
 * @brief UART start_receiver function
 */
static inline int device_uart_start_receiver(struct device *dev,
                        uint8_t* buffer, int length, void *dma,
                        int *got, void (*callback)(uint8_t *buffer, int length,
                                                   int error))
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->start_receiver)
        return dev->driver->ops->type_ops.uart->
                    start_receiver(dev, buffer, length, dma, got, callback);

    return -EOPNOTSUPP;
}


/**
 * @brief UART stop_receiver function
 */
static inline int device_uart_stop_receiver(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->type_ops.uart->stop_receiver)
        return dev->driver->ops->type_ops.uart->stop_receiver(dev);

    return -EOPNOTSUPP;
}

#endif /* __ARCH_ARM_DEVICE_UART_H */

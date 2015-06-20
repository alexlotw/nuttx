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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_uart.h>

#include "up_arch.h"
#include "tsb_scm.h"

#define SUCCESS     0

struct tsb_uart_info {
    /** device information */
    struct device   *dev;
    /** device flages */
    uint32_t        flags;
    /** for test only */
    uint8_t         fake_modem_ctrl;
    /** modem status callback function pointer */
    void            (*ms_callback)(uint8_t ms);
    /** line status callback function pointer */
    void            (*ls_callback)(uint8_t ls);
    /** receiver callback function pointer */
    void            (*rx_callback)(uint8_t *buffer, int length, int error);
    /** transmitter callback function pointer */
    void            (*tx_callback)(uint8_t *buffer, int length, int error);
};

/**
 * @brief Set UART configurations for baudrate, parity, etc.
 *
 * This function is used to set the baud rate, parity, data bit and stop bit
 * settings in the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param baud - the baud rate definition in Baud rate definition.
 * @param parity - the value of parity defined in Parity definition
 * @param databits - the number of data bits between 5 to 8 bits.
 * @param stopbits - the value stop bit defined in Stopbits definition
 * @param 0 for disable flow control, 1 for enable flow control.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_set_configuration(struct device *dev, int baud, int parity,
                                      int databits, int stopbit, int flow)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}

/**
 * @brief Get Modem control state
 *
 * This function is to get modem control state from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_ctrl output value as bitmask of Modem control definition.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_get_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    struct tsb_uart_info *info = dev->private;

    /* TODO: to implement the function body */
    *modem_ctrl = info->fake_modem_ctrl;
    return SUCCESS;
}


/**
 * @brief Set Modem control state
 *
 * This function is to write modem control settings to UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_ctrl set value as bitmask of Modem control definition.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_set_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    struct tsb_uart_info *info = dev->private;

    /* TODO: to implement the function body */
    info->fake_modem_ctrl = *modem_ctrl;
    return SUCCESS;
}


/**
 * @brief Get modem status
 *
 * This function is to get modem status from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_status output value as bitmask of Modem status definition.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_get_modem_status(struct device *dev, uint8_t *modem_status)
{
    /* TODO: to implement the function body */
    *modem_status = MSR_RI | MSR_DDCD; /* put fake data for test purpose */
    return SUCCESS;
}

/**
 * @brief Get line status
 *
 * The function is to get line status from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param line_status output value as bitmask of Line status definition.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_get_line_status(struct device *dev, uint8_t *line_status)
{
    /* TODO: to implement the function body */
    *line_status = LSR_OE | LSR_FE; /* put fake data for test purpose */
    return SUCCESS;
}


/**
 * @brief Control break state
 *
 * The function is to control break state of the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param break_on break state value, it should be 0 or 1.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_set_break(struct device *dev, uint8_t break_on)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}

/**
 * @brief Attach the modem status change callback
 *
 * This function registers a modem status (ms) callback function into the driver.
 *
 * @param dev pointer to the UART device structure
 * @param callback null means caller doesn’t need this event.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_attach_ms_callback(struct device *dev,
                                       void (*callback)(uint8_t ms))
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
 * @brief Attach the line status change callback
 *
 * The function is to register a line status (ls) callback function into the
 * driver.
 *
 * @param dev pointer to the UART device structure
 * @param callback null means caller doesn’t need this event.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_attach_ls_callback(struct device *dev,
                                       void (*callback)(uint8_t ls))
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
 * @brief Start the transmitter
 *
 * This function is to transmit data through the UART controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to send data to UART port.
 * @param length length of data.
 * @param dma DMA handle.
 * @param sent the length of transmitted data in block mode.
 * @param callback a callback function called when transmitting finished, timeout
 *                 or errors.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_start_transmitter(struct device *dev, uint8_t *buffer,
                                      int length, void *dma, int *sent,
                                      void (*callback)(uint8_t *buffer,
                                                       int length, int error))
{
    /* TODO: to implement the function body */
    return SUCCESS;
}

/**
 * @brief Stop the transmitter
 *
 * This function is to stop the data transmit in blocking or non-blocking mode.
 *
 * @param dev pointer to the UART device structure
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_stop_transmitter(struct device *dev)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
 * @brief Start the receiver
 *
 * The function is to receive data from UART controller. It could be
 * blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to receive data from UART port.
 * @param length length of data.
 * @param dma DMA handle.
 * @param got the length of received data in blocking mode.
 * @param callback a callback function called when receiving finished, timeout
 *                 or errors.
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_start_receiver(struct device *dev, uint8_t *buffer,
                                   int length, void *dma, int *got,
                                   void (*callback)(uint8_t *buffer, int length,
                                                    int error))
{
    /* TODO: to implement the function body */
    return SUCCESS;
}

/**
 * @brief Stop the receiver
 *
 * The function is to stop the data receiving in blocking or non-blocking mode.
 *
 * @param dev pointer to the UART device structure
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_stop_receiver(struct device *dev)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
 * @brief The device open function.
 *
 * This function is called when protocol preparing to use the driver ops
 * in initial stage. It is called after probe() was invoked by the system.
 * The function checks whether the driver is already open or not. If it was
 * opened, it returns driver busy error number, otherwise it keeps a flag to
 * identify the driver was opened and returns success.
 *
 * @param dev pointer to the UART device structure
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_dev_open(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;
    int ret = SUCCESS;

    flags = irqsave();

    /* TODO: to implement the function body */

    irqrestore(flags);

    return ret;
}


/**
 * @brief The device close function
 *
 * This function is called when protocol no longer using this driver.
 * The driver must be opened before calling this function.
 *
 * @param dev pointer to the UART device structure
 * @return None.
 */
static void tsb_uart_dev_close(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    /* TODO: to implement the function body */

    info->flags = 0;

    irqrestore(flags);
}


/**
 * @brief The device probe function
 *
 * This function is called by the system to register this driver when the
 * system boots up.This function allocates memory for saving driver internal
 * information data, attaches the interrupt handlers to IRQs of the controller
 * and configures the pin settings of the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @return SUCCESS on success, error code on failure.
 */
static int tsb_uart_dev_probe(struct device *dev)
{
    struct tsb_uart_info *info;
    irqstate_t flags;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    lldbg("LL uart info struct: 0x%08p\n", info);

    flags = irqsave();

    /* TODO: to implement the function body */

    info->dev = dev;
    dev->private = info;

    irqrestore(flags);

    return SUCCESS;
}


/**
 * @brief The device remove function
 *
 * This function is called by the system to unregister this driver. It
 * must be called after probe() and open(). It detaches IRQ handlers and frees
 * the internal information memory space.
 *
 * @param dev pointer to the UART device structure
 * @return None.
 */
static void tsb_uart_dev_remove(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    /* TODO: to implement the function body */

    dev->private = NULL;

    irqrestore(flags);

    free(info);
}


static struct device_uart_type_ops tsb_uart_type_ops = {
    .set_configuration  = tsb_uart_set_configuration,
    .get_modem_ctrl     = tsb_uart_get_modem_ctrl,
    .set_modem_ctrl     = tsb_uart_set_modem_ctrl,
    .get_modem_status   = tsb_uart_get_modem_status,
    .get_line_status    = tsb_uart_get_line_status,
    .set_break          = tsb_uart_set_break,
    .attach_ms_callback = tsb_uart_attach_ms_callback,
    .attach_ls_callback = tsb_uart_attach_ls_callback,
    .start_transmitter  = tsb_uart_start_transmitter,
    .stop_transmitter   = tsb_uart_stop_transmitter,
    .start_receiver     = tsb_uart_start_receiver,
    .stop_receiver      = tsb_uart_stop_receiver,
};


static struct device_driver_ops tsb_uart_driver_ops = {
    .probe          = tsb_uart_dev_probe,
    .remove         = tsb_uart_dev_remove,
    .open           = tsb_uart_dev_open,
    .close          = tsb_uart_dev_close,
    .type_ops.uart = &tsb_uart_type_ops,
};


struct device_driver tsb_uart_driver = {
    .type      = DEVICE_TYPE_UART_HW,
    .name       = "tsb_uart",
    .desc       = "TSB UART Driver",
    .ops        = &tsb_uart_driver_ops,
};


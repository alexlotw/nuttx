/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google Inc. or Linaro Ltd. nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE INC. OR
 * LINARO LTD. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "tsb_uart.h"

#define SUCCESS     0

/**
 * struct tsb_uart_info - the driver internal variable
 *
 * @param dev: device information
 * @param flags: device flages
 * @param ms_callback: modem status callback function pointer
 * @param ls_callback: line status callback function pointer
 * @param rx_callback: receiver callback function pointer
 * @param tx_callback: transmitter callback function pointer
 */
struct tsb_uart_info {
    struct device   *dev;
    uint32_t        flags;
    void            (*ms_callback)(uint8_t ms);
    void            (*ls_callback)(uint8_t ls);
    void            (*rx_callback)(uint8_t *buffer, int length, int error);
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
*/
static int tsb_uart_get_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
* @brief Set Modem control state
*
* This function is to write modem control settings to UART controller.
*
* @param dev pointer to the UART device structure
* @param modem_ctrl set value as bitmask of Modem control definition.
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
*/
static int tsb_uart_set_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
* @brief Get modem status
*
* This function is to get modem status from the UART controller.
*
* @param dev pointer to the UART device structure
* @param modem_status output value as bitmask of Modem status definition.
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
*/
static int tsb_uart_get_modem_status(struct device *dev, uint8_t *modem_status)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}

/**
* @brief Get line status
*
* The function is to get line status from the UART controller.
*
* @param dev pointer to the UART device structure
* @param line_status output value as bitmask of Line status definition.
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
*/
static int tsb_uart_get_line_status(struct device *dev, uint8_t *line_status)
{
    /* TODO: to implement the function body */
    return SUCCESS;
}


/**
* @brief Control break state
*
* The function is to control break state of the UART controller.
*
* @param dev pointer to the UART device structure
* @param break_on break state value, it should be 0 or 1.
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @param timeout the timeout value in millisecond. If 0, it waits for all data
*                fully send out.
* @param dma tells the driver use DMA for transmitting or not, 0 means no use
*            DMA.
* @param sent the length of transmitted data in block mode.
* @param callback a callback function called when transmitting finished, timeout
*                 or errors.
* @return
* @retval SUCCESS Success.
* @retval EBUSY The DMA function has error.
* @retval EINVAL Invalid parameters.
* @retval ETIMEDOUT Timer out error.
* @retval EIO Line or modem error in controller.
*/
static int tsb_uart_start_transmitter(struct device *dev, uint8_t *buffer,
                                      int length, int timeout, int dma,
                                      int *sent,
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @param timeout the timeout value in millisecond. If 0, it waits for request
*                length is full.
* @param dma tells the driver use DMA for receiving or not, 0 means no use DMA.
* @param got the length of received data in blocking mode.
* @param callback a callback function called when receiving finished, timeout
*                 or errors.
* @return
* @retval SUCCESS Success.
* @retval EBUSY The DMA function has error.
* @retval EINVAL Invalid parameters.
* @retval EIO Line or modem error in controller.
*/
static int tsb_uart_start_receiver(struct device *dev, uint8_t *buffer,
                                   int length, int timeout, int dma, int *got,
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
* @return
* @retval SUCCESS Success.
* @retval EINVAL Invalid parameters.
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
* @return
* @retval SUCCESS Success.
* @retval EBUSY Driver is already opened.
*/
static int tsb_uart_dev_open(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;
    int ret = SUCCESS;

    flags = irqsave();

    /* TODO: to implement the function body */

err_irqrestore:
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

err_irqrestore:
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
* @return
* @retval SUCCESS Success.
* @retval ENOMEM Fail to allocate the memory for driver information.
* @retval EINTR Fail to attach IRQ handler.
*/
static int tsb_uart_dev_probe(struct device *dev)
{
    struct tsb_uart_info *info;
    struct tsb_uart_init_data *init_data = dev->init_data;
    irqstate_t flags;
    int ret;

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

err_irqrestore:
    irqrestore(flags);
err_free_info:
    free(info);

    return ret;
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

/**
 * Copyright (c) 2015 Google Inc.
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
 *
 * @author Alex Lo
 * @brief TSB UART device driver
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

struct tsb_uart_info {
    struct device   *dev;
    uint32_t        flags;
    void            (*ms_callback)(void);
    void            (*ls_callback)(void);
    void            (*rx_callback)(uint8_t *buffer, int length, int error);
    void            (*tx_callback)(uint8_t *buffer, int length, int error);
};

static int tsb_uart_irq(int irq, void *context)
{
}

/*
 * This function is used to set the baud rate, parity, data bit and stop bit
 * settings in the UART controller.
 */
static int tsb_uart_set_configuration(struct device *dev, int baud, int parity,
                                      int databits, int stopbit, int flow)
{
    return 0;
}

/*
 * This function is to get modem control state from the UART controller.
 */
static int tsb_uart_get_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
	 lldbg("tsb_uart_get_modem_ctrl   \n"); /* XXX */
    return 0;
}

/*
 * This function is to write modem control settings to UART controller.
 */
static int tsb_uart_set_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    return 0;
}

/*
 * This function is to get modem status from the UART controller.
 */
static int tsb_uart_get_modem_status(struct device *dev, uint8_t *modem_status)
{
    return 0;
}

/*
 * The function is to get line status from the UART controller.
 */
static int tsb_uart_get_line_status(struct device *dev, uint8_t *line_status)
{
    return 0;
}

/*
 * The function is to control break state of the UART controller.
 */
static int tsb_uart_set_break(struct device *dev, uint8_t break_on)
{
    return 0;
}

/*
 * This function registers a modem status (ms) callback function into the
 * driver. 
 */
static int tsb_uart_attach_ms_callback(struct device *dev,
                                       void (*callback)(void))
{
    return 0;
}

/*
 * The function is to register a line status (ls) callback function into the
 * driver.
 */
static int tsb_uart_attach_ls_callback(struct device *dev,
                                       void (*callback)(void))
{
    return 0;
}

/*
 * This function is to transmit data through the UART controller. It could be
 * blocking or non-blocking and through DMA or PIO mode.
 */
static int tsb_uart_start_transmitter(struct device *dev, uint8_t *buffer,
                                      int length, int timeout, int dma,
                                      int *sent,
                                      void (*callback)(uint8_t *buffer,
                                                       int length, int error))
{
    return 0;
}

/*
 * This function is to stop the data transmit in blocking or non-blocking mode.
 */
static int tsb_uart_stop_transmitter(struct device *dev)
{
    return 0;
}

/*
 * The function is to receive data from UART controller. It could be blocking
 * or non-blocking and through DMA or PIO mode.
 */
static int tsb_uart_start_receiver(struct device *dev, uint8_t *buffer,
                                   int length, int timeout, int dma, int *got,
                                   void (*callback)(uint8_t *buffer, int length,
                                                    int error))
{
    return 0;
}

/*
 * The function is to stop the data receiving in blocking or non-blocking mode.
 */
static int tsb_uart_stop_receiver(struct device *dev)
{
    return 0;
}


/*
 * 
 */
static int tsb_uart_dev_open(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    /*if (tsb_i2s_device_is_open(info)) {
        ret = -EBUSY;
        goto err_irqrestore;
    }*/

    //info->flags = TSB_I2S_FLAG_OPEN;

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static void tsb_uart_dev_close(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    /*if (!tsb_i2s_device_is_open(info))
        goto err_irqrestore;

    if (tsb_i2s_rx_is_active(info))
        tsb_i2s_op_stop_receiver(dev);

    if (tsb_i2s_tx_is_active(info))
        tsb_i2s_op_stop_transmitter(dev);

    if (tsb_i2s_rx_is_prepared(info))
        tsb_i2s_op_shutdown_receiver(dev);

    if (tsb_i2s_tx_is_prepared(info))
        tsb_i2s_op_shutdown_transmitter(dev);
    */
    
    info->flags = 0;

err_irqrestore:
    irqrestore(flags);
}

static int tsb_uart_dev_probe(struct device *dev)
{
    struct tsb_uart_info *info;
    //struct tsb_uart_init_data *init_data = dev->init_data;
    irqstate_t flags;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

	lldbg("tsb_uart_dev_probe  +++ \n"); /* XXX */
	lldbg("LL uart info struct: 0x%08p\n", info); /* XXX */

    flags = irqsave();

    //ret = irq_attach(info->soerr_irq, tsb_i2s_irq_so_err_handler);
    //if (ret != OK)
    //    goto err_irqrestore;

//    ret = irq_attach(TSB_IRQ_UART, tsb_uart_irq);
//    if (ret != OK)
//        goto err_irqrestore;
    
    //tsb_clr_pinshare(TSB_PIN_ETM);

    info->dev = dev;
    dev->private = info;
    //saved_dev = dev;

    irqrestore(flags);
	lldbg("tsb_uart_dev_probe ---  \n"); /* XXX */
    return OK;

//err_detach_sierr_irq:
//    irq_detach(info->sierr_irq);
err_irqrestore:
    irqrestore(flags);
err_free_info:
    free(info);

    return ret;
}

static void tsb_uart_dev_remove(struct device *dev)
{
    struct tsb_uart_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    //irq_detach(info->si_irq);

    //saved_dev = NULL;
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




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
#include <nuttx/serial/uart_16550.h>

#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_uart.h"

#define SUCCESS     0

#define TSB_UART_FLAG_OPEN          BIT(0)
#define TSB_UART_FLAG_XMIT          BIT(1)
#define TSB_UART_FLAG_RECV          BIT(2)

struct uart_buffer
{
  volatile int16_t head;   /* Index to the head [IN] index in the buffer */
  volatile int16_t tail;   /* Index to the tail [OUT] index in the buffer */
  int16_t          size;   /* The allocated size of the buffer */
  uint8_t          *buffer; /* Pointer to the allocated buffer memory */
};

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

    uint32_t        reg_base;
    int             uart_irq;

    uint32_t        baud;
    uint32_t        uartclk;
    uint8_t         parity;
    uint8_t         bits;
    uint8_t         stopbits2;

    uart_datawidth_t    ier;

    struct uart_buffer xmit;
    struct uart_buffer recv;

    void            (*ms_callback)(uint8_t ms);
    void            (*ls_callback)(uint8_t ls);
    void            (*rx_callback)(uint8_t *buffer, int length, int error);
    void            (*tx_callback)(uint8_t *buffer, int length, int error);
};

static struct device *saved_dev;

/**
* @brief uart_divisor()
*/
static inline uint32_t uart_divisor(uint32_t baud, uint32_t uartclk)
{
  return (uartclk + (baud << 3)) / (baud << 4);
}


/**
* @brief uart_receive()
*/
static int uart_receive(struct tsb_uart_info *info, uint32_t *status)
{
    uint32_t rdr;

    *status = uart_getreg(info->reg_base, UART_LSR_OFFSET);
    rdr = uart_getreg(info->reg_base, UART_RBR_OFFSET);

    return rbr & 0xff;
}

/**
* @brief uart_rxint()
*/
static void uart_rxint(struct tsb_uart_info *info, uint32_t enable)
{
    if (enable) {
        info->ier |= UART_IER_ERBFI;
    } else {
        info->ier &= ~UART_IER_ERBFI;
    }
    uart_putreg(info->reg_base, UART_IER_OFFSET, info->ier);
}

/**
* @brief uart_rxavailable()
*/
static uint32_t uart_rxavailable(struct tsb_uart_info *info)
{
    return (uart_getreg(info->reg_base, UART_LSR_OFFSET) & UART_LSR_DR) != 0;
}

/**
* @brief uart_send()
*/
static void uart_send(struct tsb_uart_info *info, uint32_t ch)
{
    uart_putreg(info->reg_base, UART_THR_OFFSET, (uart_datawidth_t)ch);
}

/**
* @brief uart_txint()
*/
static void uart_txint(struct tsb_uart_info *info, uint32_t enable)
{
    irqstate_t flags;

    flags = irqsave();
    if (enable) {
        info->ier |= UART_IER_ETBEI;
        uart_putreg(info->reg_base, UART_IER_OFFSET, info->ier);

        uart_xmitchars(dev);
    } else {
      priv->ier &= ~UART_IER_ETBEI;
      uart_putreg(info->reg_base, UART_IER_OFFSET, priv->ier);
    }

  irqrestore(flags);
}

/**
* @brief uart_txready()
*/
static uint32_t uart_txready(struct tsb_uart_info *info)
{
    return ((uart_getreg(info->reg_base, UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/**
* @brief uart_txready()
*/
static uint32_t uart_txempty(struct tsb_uart_info *info)
{
    return ((uart_getreg(info->reg_base, UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/**
* @brief uart_ms_ls()
*/
static void uart_ms_ls(struct tsb_uart_info *info)
{

}

/**
* @brief uart_xmitchars()
*/
static void uart_xmitchars(struct tsb_uart_info *info)
{
    uint16_t nbytes = 0;

    while (info->xmit.head != info->xmit.tail && uart_txready(info)) {
        uart_send(dev, dev->xmit.buffer[dev->xmit.tail]);
        nbytes++;

        if (++(dev->xmit.tail) >= dev->xmit.size) {
          dev->xmit.tail = 0;
        }
    }

    if (dev->xmit.head == dev->xmit.tail) {
        uart_disabletxint(dev);
    }

    if (nbytes) {
      uart_datasent(dev);
    }
}

/**
* @brief uart_recv()
*/
static void uart_recvchars(struct tsb_uart_info *info)
{
    unsigned int status;
    int nexthead = dev->recv.head + 1;
    uint16_t nbytes = 0;

    if (nexthead >= dev->recv.size) {
        nexthead = 0;
    }

    while (uart_rxavailable(dev))
    {
        bool is_full = (nexthead == dev->recv.tail);
        char ch;

        if (is_full) {
            if (uart_rxflowcontrol(dev)) {
              /* Low-level driver activated RX flow control, exit loop now. */

              break;
            }
        }

        ch = uart_receive(dev, &status);

        if (!is_full) {
          /* Add the character to the buffer */

            dev->recv.buffer[dev->recv.head] = ch;
            nbytes++;

          /* Increment the head index */

            dev->recv.head = nexthead;
            if (++nexthead >= dev->recv.size) {
                nexthead = 0;
            }
        }
    }

    if (nbytes) {
      uart_datareceived(dev);
    }
}


/**
* @brief uart_irq_handler()
*/
static int uart_irq_handler(int irq, void *context)
{
    struct struct tsb_i2s_info *info = saved_dev->private;
    uint8_t iir;

    iir = uart_getreg(info->reg_base, UART_IIR_OFFSET);
    /* Modem status*/ /* Line status */
    if (iir & (UART_IIR_INTID_MSI | UART_IIR_INTID_RLS)) {
        uart_ms_ls();
    }
    /* Thr empty */
    if (iir & UART_IIR_INTID_THRE ) {
        uart_xmit();
    }
    /* Receive Data Available (RDA) */  /* Character Time-out Indicator (CTI) */
    if (iir & (UART_IIR_INTID_RDA | UART_IIR_INTID_CTI) ) {
        uart_recv();
    }
}

/**
* @brief tsb_uart_extract_resource()
*/
static int tsb_uart_extract_resource(struct device *dev,
                                     struct tsb_uart_info * info)
{
    struct device_resource *r = NULL;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "reg_base");
    if (!r)
        return -EINVAL;

    info->reg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_IRQ, "irq_uart");
    if (!r)
        return -EINVAL;

    info->uart_irq = (int)r->start;

    return SUCCESS;
}


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
    /* Clear fifo */
    uart_putreg(info->reg_base, UART_FCR_OFFSET, (UART_FCR_RXRST|UART_FCR_TXRST));

    /* Set trigger */
    uart_putreg(info->reg_base, UART_FCR_OFFSET, (UART_FCR_RXRST|UART_FCR_TXRST));

    /* Set up the LCR */
    lcr = 0;
    switch (databits) {
    case 5:
        lcr |= UART_LCR_WLS_5BIT;
        break;
    case 6:
        lcr |= UART_LCR_WLS_6BIT;
        break;
    case 7 :
        lcr |= UART_LCR_WLS_7BIT;
        break;
    default:
    case 8 :
        lcr |= UART_LCR_WLS_8BIT;
        break;
    }

    if (stopbit) {
        lcr |= UART_LCR_STB;
    }

    if (parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
     else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN|UART_LCR_EPS);
    }

    /* Enter DLAB=1 */
    uart_putreg(info->reg_base, UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

    /* Set the BAUD divisor */
    div = uart_divisor(priv);
    uart_putreg(info->reg_base, UART_DLM_OFFSET, div >> 8);
    uart_putreg(info->reg_base, UART_DLL_OFFSET, div & 0xff);

    /* Clear DLAB */
    uart_putreg(info->reg_base, UART_LCR_OFFSET, lcr);

    /* Configure the FIFOs */
    uart_putreg(info->reg_base, UART_FCR_OFFSET,
                (UART_FCR_RXTRIGGER_8|UART_FCR_TXRST|UART_FCR_RXRST|UART_FCR_FIFOEN));

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
    modem_ctrl = uart_getreg(info->reg_base, UART_MCR_OFFSET);
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
    uart_putreg(info->reg_base, UART_MCR_OFFSET, *modem_ctrl);
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
    modem_status = uart_getreg(info->reg_base, UART_MSR_OFFSET);
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
    line_status = uart_getreg(info->reg_base, UART_LSR_OFFSET);
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
* @retval -EINVAL Invalid parameters.
*/
static int tsb_uart_set_break(struct device *dev, uint8_t break_on)
{
    struct tsb_uart_info *info = NULL;
    uint32_t lcr = 0;

    if (dev == NULL) {
        return -EINVAL;
    }

    info = dev->private;
    lcr = uart_getreg(info->reg_base, UART_LCR_OFFSET);
    if (break_on != 0) {
        lcr |= UART_LCR_BRK;
    } else {
        lcr &= ~UART_LCR_BRK;
    }
    uart_putreg(info->reg_base, UART_LCR_OFFSET, lcr);
    
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
    struct tsb_uart_info *info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    info = dev->private;
    
    if (callback == NULL) {
        info->ier &= ~UART_IER_EDSSI;
        info->ms_callback = NULL;
        return SUCCESS;
    }

    info->ier |= UART_IER_EDSSI;
    info->ms_callback = callback;
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
    struct tsb_uart_info *info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    info = dev->private;
    
    if (callback == NULL) {
        info->ier &= ~UART_IER_ELSI;
        info->ms_callback = NULL;
        return SUCCESS;
    }

    info->ier |= UART_IER_ELSI;
    info->ls_callback = callback;
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
* @return
* @retval SUCCESS Success.
* @retval EBUSY The DMA function has error.
* @retval EINVAL Invalid parameters.
* @retval ETIMEDOUT Timer out error.
* @retval EIO Line or modem error in controller.
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
* @param dma DMA handle.
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

    if (info->flags & TSB_UART_FLAG_OPEN) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    info->flags = TSB_UART_FLAG_OPEN;

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

    if (!(info->flags & TSB_UART_FLAG_OPEN)) {
        goto err_irqrestore;
    }

    if (info->flags & TSB_UART_FLAG_XMIT) {
        tsb_uart_stop_transmitter(dev);
    }

    if (info->flags & TSB_UART_FLAG_XMIT) {
        tsb_uart_stop_receiver(dev);
    }

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
    irqstate_t flags;
    int ret = SUCCESS;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    lldbg("LL uart info struct: 0x%08p\n", info);

    ret = tsb_uart_extract_resources(dev, info);
    if (ret)
        goto err_free_info;

    flags = irqsave();

    ret = irq_attach(info->uart_irq, tsb_uart_irq_handler);
    if (ret != SUCCESS) {
        goto err_free_info;
    }

    info->dev = dev;
    dev->private = info;
    saved_dev = dev;

    irqrestore(flags);

    return SUCCESS;

err_attach_irq:
    irq_detach(info->uart_irq);
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

    irq_detach(info->uart_irq);

    saved_dev = NULL;
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


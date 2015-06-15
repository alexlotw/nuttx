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
    uint8_t         flow;

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
* @brief ua_getreg()
*/
static uint32_t ua_getreg(uint32_t base, uint32_t offset)
{
    return getreg32(base + offset);
}

/**
* @brief ua_putreg()
*/
static void ua_putreg(uint32_t base, uint32_t offset, uint32_t value)
{
    putreg32(value, base + offset);
}

/**
* @brief ua_reg_bit_value()
*/
static void ua_reg_bit_value(uint32_t reg, uint32_t offset, uint8_t bitmask,
                           uint8_t bitvalue)
{
    uint8_t regvalue = ua_getreg(reg, offset);
    if (bitvalue == 1) {
        regvalue |= bitmask;
    } else {
        regvalue &= ~bitmask;
    }
    ua_putreg(reg, offset, regvalue);
}

/**
* @brief ua_reg_bit_set()
*/
static void ua_reg_bit_set(uint32_t reg, uint32_t offset, uint8_t bitmask)
{
    uint8_t regvalue = ua_getreg(reg, offset);
    regvalue |= bitmask;
    ua_putreg(reg, offset, regvalue);
}

/**
* @brief ua_reg_bit_clr()
*/
static void ua_reg_bit_clr(uint32_t reg, uint32_t offset, uint8_t bitmask)
{
    uint8_t regvalue = ua_getreg(reg, offset);
    regvalue &= ~bitmask;
    ua_putreg(reg, offset, regvalue);
}


/**
* @brief ua_set_dlab()
*/
static void ua_set_dlab(uint32_t base, uint8_t value)
{
    ua_reg_bit_set(base, UA_LCR, UA_DLAB, value);
}

/**
* @brief ua_get_char()
*/
static uint8_t ua_get_char(uint32_t base)
{
    return ua_getreg(base, UA_RBR_THR_DLL);
}

/**
* @brief ua_put_char()
*/
static void ua_put_char(uint32_t base, uint8_t ch)
{
    ua_putreg(base, UA_RBR_THR_DLL, ch);
}

/**
* @brief ua_enable_interrupt()
*/
static void ua_enable_interrupt(uint32_t base, uint8_t interrupt)
{
    ua_reg_bit_set(base, UA_IER_DLH, UA_IER_ERBFI);
}

/**
* @brief ua_disable_interrupt()
*/
static void ua_disable_interrupt(uint32_t base, uint8_t interrupt)
{
    ua_reg_bit_clr(base, UA_IER_DLH, UA_IER_ERBFI);
}

/**
* @brief ua_put_char()
*/
static void ua_set_divisor(uint32_t base, uint8_t divisor)
{
    ua_set_dlab(1);
    ua_putreg(base, UA_IER_DLH, divisor);
    ua_set_dlab(0);
}

/**
* @brief ua_set_fifo_trigger()
*/
static void ua_set_fifo_trigger(uint32_t base, uint8_t rx_level,
                                uint8_t tx_level)
{
    ua_putreg(base, UA_FCR_IIR, rx_level << UA_RX_FIF0_TRIGGER_SHIFT |
              tx_level << UA_TX_FIF0_TRIGGER_SHIFT);
}

/**
* @brief ua_get_interrupt_id()
*/
static uint8_t ua_get_interrupt_id(uint32_t base)
{
    return (ua_getreg(base, UA_FCR_IIR) & UA_INTERRUPT_ID_MASK);
}

static void ua_fifo_reset(uint32_t base)
{
    ua_putreg(base, UA_FCR_IIR, UA_TX_FIFO_RESET | UA_RX_FIFO_RESET);
}

static void ua_fifo_enable(uint32_t base)
{
    ua_putreg(base, UA_FCR_IIR, UA_FIFO_ENABLE);
}

/**
* @brief ua_set_data_bits()
*/
static void ua_set_data_bits(uint32_t base, uint8_t databits)
{
    uint8_t lcr = ua_getreg(base, UA_LCR);
    lcr &= ~UA_DLS_MASK;
    lcr |= databits;
    ua_putreg(base, UA_LCR, lcr);
}

/**
* @brief ua_set_stop_bit()
*/
static void ua_set_stop_bit(uint32_t base, uint8_t stopbit)
{
    ua_reg_bit_set(base, UA_LCR, UA_LCR_STOP, stopbit);
}

/**
* @brief ua_set_parity_bits()
*/
static void ua_set_parity_bits(uint32_t base, uint8_t paritybits)
{
    uint8_t lcr = ua_getreg(base, UA_LCR);
    lcr &= ~(UART_LCR_PEN|UART_LCR_EPS);
    lcr |= paritybits;
    ua_putreg(base, UA_LCR, lcr);
}

/**
* @brief ua_set_line_ctrl()
*/
static void ua_set_line_ctrl(uint32_t base, uint8_t ctrl)
{
    uint8_t lcr = ua_getreg(base, UA_LCR);
    lcr &= ~UA_LCR_MASK;
    lcr |= ctrl;
    ua_putreg(base, UA_LCR, lcr);
}

/**
* @brief ua_get_line_ctrl()
*/
static uint8_t ua_get_line_ctrl(uint32_t base)
{
    return ua_getreg(base, UA_LCR) & UA_LCR_MASK;
}

/**
* @brief ua_set_auto_flow()
*/
static void ua_set_auto_flow(uint32_t base, uint8_t autoflow)
{
    ua_reg_bit_set(base, UA_MCR, UA_AUTO_FLOW_ENABLE, autoflow);
}

/**
* @brief ua_is_tx_fifo_full()
*/
static uint8_t ua_is_tx_fifo_full(uint32_t base)
{
    return (ua_getreg(base, UA_USR) & UA_USR_TFNF) ? 0 : 1;
}

/**
* @brief ua_is_tx_fifo_empty()
*/
static uint8_t ua_is_tx_fifo_empty(uint32_t base)
{
    return (ua_getreg(base, UA_USR) & UA_USR_TFE) ? 1 : 0;
}

/**
* @brief ua_is_rx_fifo_empty()
*/
static uint8_t ua_is_rx_fifo_empty()
{
    return (ua_getreg(base + UA_USR) & UA_USR_RFNE) ? 0 : 1;
}

/**
* @brief uart_xmitchars()
*/
static void uart_xmitchars(struct tsb_uart_info *info)
{
    while (!ua_is_tx_fifo_full(uart_info->reg_base)) {
        ua_put_char(uart_info->reg_base,
                    uart_info->xmit.buffer[uart_info->xmit.head++])
        if (uart_info->xmit.head == uart_info->xmit.tail) {
            ua_disable_interrupt(uart_info->reg_base, UA_IER_ETBEI);
            if (uart_info->tx_callback) {
                uart_info->tx_callback(uart_info->xmit.buffer,
                                       uart_info->xmit.tail, SUCCESS)
            }
        }
    }
}

/**
* @brief uart_recv()
*/
static void uart_recvchars(struct tsb_uart_info *uart_info, uint8_t int_id)
{
    while (!ua_is_rx_fifo_empty(uart_info->reg_base)) {
        uart_info->recv.buffer[uart_info->recv.head++] =
                        ua_get_char(uart_info->reg_base);
                        
        if (uart_info->recv.head == uart_info->recv.tail ||
            int_id == UA_INTERRUPT_ID_TO) {
                
            ua_disable_interrupt(uart_info->reg_base, UA_IER_ERBFI);
            if (uart_info->rx_callback) {
                uart_info->rx_callback(uart_info->recv.buffer,
                                       uart_info->recv.tail, SUCCESS)
            }
        }
    }
}

/**
* @brief uart_irq_handler()
*/
static int uart_irq_handler(int irq, void *context)
{
    struct struct tsb_uart_info *uart_info = saved_dev->private;
    uint8_t interrupt_id;
    uint8_t status;

    while (1) {
        interrupt_id = ua_get_interrupt_id(uart_info->reg_base);
        if (interrupt_id == UA_INTERRUPT_ID_NO) {
            /* no interrupt pending */
            break;
        }
        
        switch (interrupt_id) {
        case UA_INTERRUPT_ID_MS:
            status = ua_getreg(uart_info->reg_base, UA_MSR);
            if (uart_info->ms_callback) {
                uart_info->ms_callback(status);
            }
            break;
        case UA_INTERRUPT_ID_LS:
            status = ua_getreg(uart_info->reg_base, UA_LSR);
            if (uart_info->ls_callback) {
                uart_info->ls_callback(status);
            }
            break;
        case UA_INTERRUPT_ID_TX:
            uart_xmitchars(uart_info);
            break;
        case UA_INTERRUPT_ID_TO:
        case UA_INTERRUPT_ID_RX:
            uart_recvchars(uart_info, interrupt_id);
            break;
        }
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
    struct tsb_uart_info *uart_info = NULL;
    uint8_t lcr = 0;
    
    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;

    ua_fifo_reset(uart_info->reg_base);

    ua_set_fifo_trigger(uart_info->reg_base, UA_RX_FIFO_TRIGGER_1_2,
                        UA_TX_FIFO_TRIGGER_1_2);

    lcr = 0;
    switch (databits) {
    case 5:
        lcr = UA_DLS_5_BITS;
        break;
    case 6:
        lcr = UA_DLS_6_BITS;
        break;
    case 7 :
        lcr = UA_DLS_7_BITS;
        break;
    default:
    case 8 :
        lcr = UA_DLS_8_BITS;
        break;
    }
    ua_set_data_bits(uart_info->reg_base, lcr);

    ua_set_stop_bit(uart_info->reg_base, stopbit);
    
    lcr = 0;
    if (parity == ODD_PARITY) {
        lcr |= UA_LCR_PEN;
    } else if (priv->parity == EVEN_PARITY) {
        lcr |= (UA_LCR_PEN|UA_LCR_EPS);
    }
    ua_set_parity_bits(uart_info->reg_base, lcr);

    uint32_t divisor = (48000000 >> 4) / baud;

    ua_set_divisor(uart_info->reg_base, divisor);

    ua_fifo_enable(uart_info->reg_base);

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
    struct tsb_uart_info *uart_info = NULL;
    
    if (dev == NULL || modem_ctrl == NULL) {
        return -EINVAL;
    }
    
    uart_info = dev->private;
    
    *modem_ctrl = ua_getreg(uart_info->reg_base, UA_MSR);

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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || modem_ctrl == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;

    ua_putreg(uart_info->reg_base, UA_MCR, *modem_ctrl);
    
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || modem_status == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;
    
    *modem_status = ua_getreg(uart_info->reg_base, UA_MSR);
    
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || line_status == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;
    
    *line_status = ua_getreg(uart_info->reg_base, UA_LSR);
    
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;

    ua_reg_bit_set(info->reg_base, UA_LCR, UA_LCR_BREAK, break_on);
    
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;
    
    if (callback == NULL) {
        uart_info->ier &= ~UART_IER_ELSI;
        uart_info->ms_callback = NULL;
        return SUCCESS;
    }

    uart_info->ier |= UART_IER_ELSI;
    uart_info->ls_callback = callback;
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;

    uart_info->xmit.buffer = buffer;
    uart_info->xmit.head = 0;
    uart_info->xmit.tail = size;
    uart_info->xmit.size = size;
    uart_info->tx_callback = callback;
    
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
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = dev->private;
    
    uart_info->recv.buffer = buffer;
    uart_info->recv.head = 0;
    uart_info->recv.tail = size;
    uart_info->recv.size = size;
    uart_info->rx_callback = callback;

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

    ret = irq_attach(info->uart_irq, uart_irq_handler);
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


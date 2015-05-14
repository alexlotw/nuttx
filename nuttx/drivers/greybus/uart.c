/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/greybus/greybus.h>

#include "uart-gb.h"

#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

struct gb_uart_info {
    uint16_t        cport;
    uint32_t        flags;
    pthread_t       uart_thread;
    
    struct device   *dev;
};

struct gb_uart_info *info;

void gb_uart_ms_ls_callback(void)
{
    
}

void gb_uart_rx_callback()
{

}


/*
 *  This thread for pumping receiving data to peer.
 */
static void *gb_uart_thread(void *data)
{
    struct gb_operation *operation;
    void *request;
    int ret;

    while (1) {
        msglen = mq_receive(priv->mq, &msg, sizeof(msg), &prio);
        
    }

    /* NOTREACHED */
    return NULL;
}

/*
 * Returns the major and minor Greybus UART protocol version number supported
 * by the UART device.
 */
static uint8_t gb_uart_protocol_version(struct gb_operation *operation)
{
    struct gb_uart_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/*
 * Requests that the UART device begin transmitting characters. One or more
 * bytes to be transmitted will be supplied.
 */
static uint8_t gb_uart_send_data(struct gb_operation *operation)
{
    int ret;
    int sent;
    struct gb_uart_send_data_request *request;

    request = (struct gb_uart_send_data_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_start_transmitter(info->dev, request->data,
                                        request->size, 0, 0, &sent, null);
    if (ret) {
        
    }
    
    return GB_OP_SUCCESS;
}

/*
 * Receive data from the UART. One or more bytes will be supplied.
 */
static uint8_t gb_uart_receive_data(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/*
 * Sets the line settings of the UART to the specified baud rate, format,
 * parity, and data bits.
 */
static uint8_t gb_uart_set_line_coding(struct gb_operation *operation)
{
    int ret;
    int baud, parity, databits, stopbit, flow;
    struct gb_serial_line_coding_request *request;
    
    request = (struct gb_serial_line_coding_request *)
                  gb_operation_get_request_payload(operation);

    baud = request->rate;

    if (request->format == GB_SERIAL_1_STOP_BITS) {
        stopbit = ONE_STOP_BIT;
    } else if (request->format == GB_SERIAL_1_5_STOP_BITS) {
        stopbit = ONE5_STOP_BITS;
    } else if (request->format == GB_SERIAL_2_STOP_BITS) {
        stopbit = TWO_STOP_BITS;
    } else {
        /* error in stop bit */
    }

    if (request->parity == GB_SERIAL_NO_PARITY) {
        parity = NO_PARITY;
    } else if (request->parity == GB_SERIAL_ODD_PARITY) {
        parity = ODD_PARITY;
    } else if (request->parity == GB_SERIAL_EVEN_PARITY) {
        parity = EVEN_PARITY;
    } else if (request->parity == GB_SERIAL_MARK_PARITY) {
        parity = MARK_PARITY;
    } else if (request->parity == GB_SERIAL_SPACE_PARITY) {
        parity = SPACE_PARITY;
    } else {
        /* error in parity */
    }

    databits = request->data;

    flow = 0;

    ret = device_uart_set_configuration(info->dev, baud, parity, databits,
                                        stopbit, flow);
    if (ret) {
        
    }
    
    return ret;
}

/*
 * Controls RTS and DTR line states of the UART.
 */
static uint8_t gb_uart_set_control_line_state(struct gb_operation *operation)
{
    int ret;
    uint8_t modem_ctrl;
    struct gb_uart_set_control_line_state_request *request;
    
    request = (struct gb_uart_set_control_line_state_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_get_modem_ctrl(info->dev, &modem_ctrl);
    
    if (request->control & GB_UART_CTRL_DTR) {
        modem_ctrl |= MCR_DTR;
    }
    else {
        modem_ctrl &= ~MCR_DTR;
    }
    if (request->control & GB_UART_CTRL_RTS) {
        modem_ctrl |= MCR_RTS;
    }
    else {
        modem_ctrl &= ~MCR_RTS;
    }

    ret = device_uart_set_modem_ctrl(info->dev, &modem_ctrl);
    if () {
        
    }

    return GB_OP_SUCCESS;
}

/*
 * Requests that the UART generate a break condition on its transmit line.
 */
static uint8_t gb_uart_send_break(struct gb_operation *operation)
{
    int ret;
    uint8_t modem_ctrl;
    struct gb_uart_set_control_line_state_request *request;
    
    request = (struct gb_uart_set_control_line_state_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(request->state);
    if (ret) {
        
    }
    
    return GB_OP_SUCCESS;
}

/*
 * Receives the state of the UART's control lines and any line errors that
 * might have occurred.
 */
static uint8_t gb_uart_serial_state(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/*
 * 
 */
static int gb_uart_init(unsigned int cport)
{
    struct gb_uart_dev_info *dev_info;
    struct gb_uart_info *info;
    unsigned int i;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

lldbg("GB uart info struct: 0x%08p\n", info); /* XXX */

    info->cport = cport

    ret = pthread_create(&info->uart_thread, NULL, gb_uart_thread, info);    

    if (ret) {
        ret = -ret;
        goto err_free_wd_info;
    }
    
    info->dev = device_open(dev_info->dev_class, dev_info->dev_id);
    if (!info->dev) {
        ret = -EIO;
        goto err_kill_pthread;
    }

    return 0;

err_kill_pthread:
    pthread_kill(info->rx_thread, SIGKILL);
    
    return rel;
}

static int gb_uart_exit(unsigned int cport)
{
    return 0;
}

static struct gb_operation_handler gb_uart_handlers[] = {
    GB_HANDLER(GB_UART_TYPE_PROTOCOL_VERSION, gb_uart_protocol_version),
    GB_HANDLER(GB_UART_TYPE_SEND_DATA, gb_uart_send_data),
    GB_HANDLER(GB_UART_TYPE_RECEIVE_DATA, gb_uart_receive_data),
    GB_HANDLER(GB_UART_TYPE_SET_LINE_CODING, gb_uart_set_line_coding),
    GB_HANDLER(GB_UART_TYPE_SET_CONTROL_LINE_STATE, gb_uart_set_control_line_state),
    GB_HANDLER(GB_UART_TYPE_SEND_BREAK, gb_uart_send_break),
    GB_HANDLER(GB_UART_TYPE_SERIAL_STATE, gb_uart_serial_state),
};

struct gb_driver uart_driver = {
    .init = gb_uart_init,
    .exit = gb_uart_exit,
    .op_handlers = (struct gb_operation_handler*) gb_uart_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_uart_handlers),
};

void gb_uart_register(int cport)
{
lldbg("gb_uart_register: cport %d\n", cport);
    gb_register_driver(cport, &uart_driver);
}

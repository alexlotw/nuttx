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

#define FLAGS_DEVICE_INUSE   0x01
#define FLAGS_STATUS_CHANGE  0x02
#define FLAGS_DATA_COMEIN    0x04

#define NODE_STATUS_AVAIL       0x00
#define NODE_STATUS_INUSE       0x01

struct list_node {
    struct gb_operation                 *operation;
    struct gb_uart_receive_data_request *request;
    uint8_t                             *buffer;
    int                                 status;
    struct list_node                    *next;
};

struct list_node_cb {
    struct list_node    *head;
    struct list_node    *tail;
    struct list_node    *available;
    struct list_node    *used;
    int                 total;
    int                 remainder;
}

struct gb_uart_info {
    uint16_t        cport;
    uint32_t        flags;
    pthread_t       uart_thread;
    sem_t           uart_sem;
    /* ms_ls status data */
    struct gb_operation *operation;
    struct gb_uart_serial_state_request *request;
    uint16_t        last_serial_state;
    /* receiving op data */
    list_node_cb   node_cb
    /* device */
    struct device   *dev;
};

struct gb_uart_info *info;

int list_add_node(struct list_node_cb node_cb, list_note *note)
{
    if (node_cb->head == NULL) {
        node_cb->head = note;
        note->next = note;
        node_cb->tail = note;
        note_cb->avail = note;
        note_cb->used = NULL;
        note_cb->total = 1;
        note_cb->remainder = 1;
    }
    else {
        node_cb->tail.next = note;
        note->next = node_cb->head;
        note_cb->total++;
        note_cb->remainder++;
    }
}

/*
 *  call this funciton will consume the node.
 */
struct list_node *list_get_avail_node(struct list_node_cb *node_cb)
{
    struct list_node *node;

    if (node_cb->remainder) {
        return NULL;
    }
    else {
        node = node_cb->available;
        node->status = NODE_STATUS_INUSE;
        node_cb->available = node->next;
        node_cb->remainder--;
        return node;
    }
}

/*
 *  call this function will release the node.
 */
struct list_node *list_get_used_node(struct list_node_cb *node_cb)
{
    struct list_node *node;

    if (node_cb->total == node_cb->remainder) {
        return NULL;
    }
    else {
        node = node_cb->used;
        node->status = NODE_STATUS_AVAIL;
        node_cb->used = node->next;
        node_cb->remainder++;
        return node;
    }
}


void gb_uart_ms_ls_callback(void)
{
    sem_post(&info->uart_sem);
}

void gb_uart_rx_callback(uint8_t *buffer, int length, int error)
{
    struct list_node *node;
    
    sem_post(&info->uart_sem);

    
    
    /* switch free buffer to start new receiving */
}

void gb_uart_ms_ls_proc(void)
{
    struct gb_operation *operation;
    
    int ret;
    uint16_t current_state = 0;
    uint8_t ms_uart, ls_uart;
    
    operation = gb_operation_create(info->cport,
                                    GB_UART_TYPE_SERIAL_STATE,
                                    sizeof(*request));
    
    request = (struct gb_uart_set_control_line_state_request *)
                    gb_operation_get_request_payload(operation);

    device_uart_get_modem_status(info->dev, &ms_uart);
    device_uart_get_line_status(info->dev, &ls_uart);
    
    if (ms_data & MSR_DCD) {
        current_state |= GB_UART_CTRL_DCD;
    }
    if (ms_data & MSR_DSR) {
        current_state |= GB_UART_CTRL_DSR;
    }
    if (ls_data & LSR_BI) {
        current_state |= GB_UART_CTRL_BRK;
    }
    if (ms_data & MSR_RI) {
        current_state |= GB_UART_CTRL_RI;
    }
    if (ls_data & LSR_FE) {
        current_state |= GB_UART_CTRL_FRAMING;
    }
    if (ls_data & LSR_PE) {
        current_state |= GB_UART_CTRL_PARITY;
    }
    if (ls_data & LSR_OE) {
        current_state |= GB_UART_CTRL_OVERRUN;
    }

    if (info->last_serial_state ^ current_state) {
        info->last_serial_state = current_state;
        
        request->control = 0;
        request->data = current_state;
        ret = gb_operation_send_request(operation, NULL, false);
        if (ret)
            lldbg("--- Can't report event : %d\n", ret); /* XXX */
    }
    // gb_operation_destroy(operation);    
}

void gb_uart_rx_proc(void)
{
    struct gb_operation *operation;

    
    
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
        sem_wait(&info->uart_sem);

        if (info->flags & FLAGS_STATUS_CHANGE) {
            info->flags &= ~FLAGS_STATUS_CHANGE;
            gb_uart_ms_ls_proc();
        }

        /* Don't use flag for checking data come in, use link_list */
        /*if (info->flags & FLAGS_DATA_COMEIN) {
            info->flags &= ~FLAGS_STATUS_CHANGE;
            gb_uart_rx_proc();
        }*/
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
                                        request->size, 0, 0, &sent, NULL);
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
    if (ret) {
        
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
    struct gb_uart_set_break_request *request;
    
    request = (struct gb_uart_set_break_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(info->dev, request->state);
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
    int ret;
    unsigned int i;
    struct gb_uart_info *info;
    struct gb_operation *operation;
    struct list_node *node;
    
    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    lldbg("GB uart info struct: 0x%08p\n", info); /* XXX */

    info->cport = cport;

    /* operation of serial status */
    operation = gb_operation_create(info->cport,
                                    GB_UART_TYPE_SERIAL_STATE,
                                    sizeof(*request));
    if (operation) {
        /* handle operation error */
    }
    
    request = (struct gb_uart_set_control_line_state_request *)
                    gb_operation_get_request_payload(operation);
    

    /* create operations of receiving */
    for (i = 0; i < 5; i++) {
        operation = gb_operation_create(info->cport,
                                        GB_UART_TYPE_RECEIVE_DATA,
                                        sizeof(*request) + 2 + 128);
        if (operation) {
            node = (struct list_node *)malloc(list_node);
            node->operation = operation;
            node->request = (struct gb_uart_receive_data_request *)
                                gb_operation_get_request_payload(operation);
            node->buffer = node->request->data;
            node->status = NODE_STATUS_AVAIL;
            list_add_node(node);
        }
        else {
            // do something if error.
        }
        // add to the link list.
    }

    

    ret = sem_init(&info->uart_sem, 0, 0);
    
    ret = pthread_create(&info->uart_thread, NULL, gb_uart_thread, info);    

    if (ret) {
        ret = -ret;
        //goto err_free_wd_info;
    }
    
    info->dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!info->dev) {
        ret = -EIO;
        goto err_kill_pthread;
    }

    return 0;

err_kill_pthread:
    //pthread_kill(info->uart_thread, SIGKILL);
    
    return ret;
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

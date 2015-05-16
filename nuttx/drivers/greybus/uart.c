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
#include <queue.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/greybus/greybus.h>

#include "uart-gb.h"

#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

#define NUM_SMALL_OPERATION     5
#define NUM_LARGE_OPERATION     5

#define OP_BUF_SIZE_SMALL           32
#define OP_BUF_SIZE_LARGE           128

#ifndef SIGKILL
#define SIGKILL     9
#endif

struct op_node {
    sq_entry_t              entry;
    struct gb_operation     *operation;
    uint16_t                *size;
    uint8_t                 *buffer;
    int                     op_buf_size;
};

struct gb_uart_info {
    uint16_t        cport;
    uint32_t        flags;

    uint8_t         updated_ms;
    uint8_t         updated_ls;
    struct gb_operation *ms_ls_operation;
    struct gb_uart_serial_state_request *ms_ls_request;
    uint16_t        last_serial_state;
    sem_t           status_sem;
    pthread_t       status_thread;

    sq_queue_t      small_op_queue;
    sq_queue_t      large_op_queue;
    sq_queue_t      received_op_queue;
    struct op_node  *working_op_node;
    int             need_free_node;
    int             last_op_size;
    sem_t           rx_sem;
    pthread_t       rx_thread;

    struct device   *dev;
};

struct gb_uart_info *info;

/*
 *  greybus protocol callback to device driver
 */
void uart_ms_callback(uint8_t ms)
{
    info->updated_ms = ms;
    sem_post(&info->status_sem);
}

/*
 *  greybus protocol callback to device driver
 */
void uart_ls_callback(uint8_t ls)
{
    info->updated_ls = ls;
    sem_post(&info->status_sem);
}

/*
 *  greybus protocol callback to device driver
 */

static struct sq_entry_s *get_free_entry(sq_queue_t *first, sq_queue_t *second)
{
    if (!sq_empty(first)) {
        return sq_remfirst(&info->small_op_queue);
    } else if (!sq_empty(second)) {
        return sq_remfirst(&info->small_op_queue);
    } else {
        return NULL;
    }
}
 
void uart_rx_callback(uint8_t *buffer, int length, int error)
{
    sq_entry_t *entry;
    struct op_node *node;

    if (error) {
        /* error handle */ 
    }

    *info->working_op_node->size = length;
    sq_addlast(&info->working_op_node->entry, &info->received_op_queue);

    if (length < OP_BUF_SIZE_SMALL) {
        entry = get_free_entry(&info->small_op_queue, &info->large_op_queue);
    } else {
        entry = get_free_entry(&info->large_op_queue, &info->small_op_queue);
    }

    if (entry) {
        node = (struct op_node *) entry;
        device_uart_start_receiver(info->dev, node->buffer, node->op_buf_size,
                                   10, 0, NULL, uart_rx_callback);
        info->working_op_node = node;
    } else {
        info->need_free_node = 1;
        if (length > OP_BUF_SIZE_SMALL)
            info->last_op_size = OP_BUF_SIZE_LARGE;
        else
            info->last_op_size = OP_BUF_SIZE_SMALL;
    }

    sem_post(&info->rx_sem);
}

/*
 *  status change process
 */
static void uart_ms_ls_proc(void)
{
    uint16_t current_state = 0;
    int ret;

    if (info->updated_ms & MSR_DCD) {
        current_state |= GB_UART_CTRL_DCD;
    }
    if (info->updated_ms & MSR_DSR) {
        current_state |= GB_UART_CTRL_DSR;
    }
    if (info->updated_ms & MSR_RI) {
        current_state |= GB_UART_CTRL_RI;
    }
    if (info->updated_ls & LSR_BI) {
        current_state |= GB_UART_CTRL_BRK;
    }
    if (info->updated_ls & LSR_FE) {
        current_state |= GB_UART_CTRL_FRAMING;
    }
    if (info->updated_ls & LSR_PE) {
        current_state |= GB_UART_CTRL_PARITY;
    }
    if (info->updated_ls & LSR_OE) {
        current_state |= GB_UART_CTRL_OVERRUN;
    }

    if (info->last_serial_state ^ current_state) {
        info->last_serial_state = current_state;

        info->ms_ls_request->control = 0;
        info->ms_ls_request->data = current_state;
        ret = gb_operation_send_request(info->ms_ls_operation, NULL, false);
        if (ret)
            lldbg("--- Can't report event : %d\n", ret); /* XXX */
    }
    // gb_operation_destroy(operation);
}

/*
 *  Data receiving process
 */
static void uart_rx_proc(void)
{
    struct op_node *node, *small_node;
    sq_entry_t *entry, *small_entry;
    int ret;

    if (!sq_empty(&info->received_op_queue)) {
        entry = sq_remfirst(&info->received_op_queue);
        node = (struct op_node *) entry;
        if ((node->op_buf_size == OP_BUF_SIZE_LARGE) &&
            (*node->size <= OP_BUF_SIZE_SMALL) &&
            (!sq_empty(&info->small_op_queue))) {

            small_entry = sq_remfirst(&info->small_op_queue);
            small_node = (struct op_node *) small_entry;
            memcpy(small_node->buffer, node->buffer, node->size);
            *small_node->size = *node->size;
            sq_addlast(entry, &info->large_op_queue);
            
            ret = gb_operation_send_request(small_node->operation, NULL, false);
            if (ret) {
                /* gb_operation handler */
            }
            sq_addlast(small_entry, &info->small_op_queue);
        } else {
            ret = gb_operation_send_request(node->operation, NULL, false);
            if (ret) {
                /* gb_operation handler */
            }
            sq_addlast(entry, &info->large_op_queue);
        }
    }

    if (info->need_free_node) {
        if (info->last_op_size == OP_BUF_SIZE_LARGE) {
            entry = get_free_entry(&info->large_op_queue,
                                   &info->small_op_queue);
        }
        else {
            entry = get_free_entry(&info->small_op_queue,
                                   &info->large_op_queue);
        }
        if (entry) {
            node = (struct op_node *) entry;
            ret = device_uart_start_receiver(info->dev, node->buffer,
                                             node->op_buf_size, 10, 0, NULL,
                                             uart_rx_callback);
            if (ret) {
                /* fatal error */
            }
        }
        else {
            /* fatal error */
        }
    }
}

/*
 *  This thread for notifying status change to peer.
 */
static void *status_thread(void *data)
{
    while (1) {
        sem_wait(&info->status_sem);
        uart_ms_ls_proc();
    }
    return NULL;
}

/*
 *  This thread for pumping receiving data to peer.
 */
static void *rx_thread(void *data)
{
    while (1) {
        sem_wait(&info->rx_sem);
        uart_rx_proc();
    }
    return NULL;
}

static int uart_status_change_deinit(void)
{
    if (info->status_thread) {
        pthread_kill(info->status_thread, SIGKILL);
    }

    if (info->ms_ls_operation) {
        gb_operation_destroy(info->ms_ls_operation);
    }

    return OK;
}

static int uart_status_change_init(void)
{
    struct gb_operation *operation;
    int ret;

    info->ms_ls_operation = gb_operation_create(info->cport,
                                    GB_UART_TYPE_SERIAL_STATE,
                                    sizeof(*info->ms_ls_request));
    if (operation) {
        goto init_fail;
    }

    info->ms_ls_request = (struct gb_uart_serial_state_request *)
                    gb_operation_get_request_payload(operation);

    ret = sem_init(&info->status_sem, 0, 0);
    if (ret != OK) {
        ret = -ret;
        goto init_fail;
    }

    ret = pthread_create(&info->status_thread, NULL, status_thread, info);
    if (ret) {
        ret = -ret;
        goto init_fail;
    }

init_fail:
    return ret;
}

static int uart_rx_deinit(void)
{
    return 0;
}

static int create_operations(int op_size, sq_queue_t *queue, int num)
{
    struct gb_operation *operation;
    struct gb_uart_receive_data_request *request;
    struct op_node *node;
    int ret = OK;
    int i;

    for (i = 0; i < num; i++) {
        operation = gb_operation_create(info->cport,
                                        GB_UART_TYPE_RECEIVE_DATA,
                                        2 + op_size);
        if (operation) {
            node = (struct op_node *)malloc(sizeof(struct op_node));
            node->operation = operation;

            request = (struct gb_uart_receive_data_request *)
                                gb_operation_get_request_payload(operation);
            node->size   = &request->size;
            node->buffer = request->data;
            node->op_buf_size = op_size;
            sq_addlast(&node->entry, queue);
        }
        else {
            ret = -ENOMEM;
            goto create_err;
        }
    }

create_err:
    return ret;
}

static int uart_rx_init(void)
{
    int ret = OK;

    ret = create_operations(OP_BUF_SIZE_LARGE, &info->large_op_queue,
                            NUM_LARGE_OPERATION);
    if (ret) {
        uart_rx_deinit();
        return ret;
    }

    ret = create_operations(OP_BUF_SIZE_SMALL, &info->small_op_queue,
                            NUM_SMALL_OPERATION);
    if (ret) {
        uart_rx_deinit();
        return ret;
    }

    ret = sem_init(&info->rx_sem, 0, 0);
    if (ret != OK) {
        uart_rx_deinit();
        return ret;
    }

    ret = pthread_create(&info->rx_thread, NULL, rx_thread, info);
    if (ret) {
        uart_rx_deinit();
        return ret;
    }
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
        return GB_OP_MALFUNCTION;
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
        ret = GB_OP_INVALID;
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
        ret = GB_OP_INVALID;
    }

    databits = request->data;

    flow = 0;

    ret = device_uart_set_configuration(info->dev, baud, parity, databits,
                                        stopbit, flow);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
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
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

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
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}

/*
 * Requests that the UART generate a break condition on its transmit line.
 */
static uint8_t gb_uart_send_break(struct gb_operation *operation)
{
    int ret;
    struct gb_uart_set_break_request *request;

    request = (struct gb_uart_set_break_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(info->dev, request->state);
    if (ret) {
        return GB_OP_MALFUNCTION;
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
    //struct gb_uart_info *info;

    info = zalloc(sizeof(*info));
    if (!info)
        return GB_OP_NO_MEMORY;

    lldbg("GB uart info struct: 0x%08p\n", info);

    info->cport = cport;

    ret = uart_status_change_init();
    if (ret) {
        uart_status_change_deinit();
        return GB_OP_MALFUNCTION;
    }

    sq_init(&info->small_op_queue);
    sq_init(&info->large_op_queue);
    sq_init(&info->received_op_queue);

    ret = uart_rx_init();
    if (ret) {
        uart_status_change_deinit();
        uart_rx_deinit();
        return GB_OP_MALFUNCTION;
    }

    info->dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!info->dev) {
        uart_status_change_deinit();
        uart_rx_deinit();
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}

static int gb_uart_exit(unsigned int cport)
{
    return GB_OP_SUCCESS;
}

static struct gb_operation_handler gb_uart_handlers[] = {
    GB_HANDLER(GB_UART_TYPE_PROTOCOL_VERSION, gb_uart_protocol_version),
    GB_HANDLER(GB_UART_TYPE_SEND_DATA, gb_uart_send_data),
    GB_HANDLER(GB_UART_TYPE_RECEIVE_DATA, gb_uart_receive_data),
    GB_HANDLER(GB_UART_TYPE_SET_LINE_CODING, gb_uart_set_line_coding),
    GB_HANDLER(GB_UART_TYPE_SET_CONTROL_LINE_STATE,
               gb_uart_set_control_line_state),
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

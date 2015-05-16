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

struct op_node {
    sq_entry_s              entry;
    struct gb_operation     *operation;
    uint8_t                 *size;
    uint8_t                 *buffer;
    uint16_t                op_buf_size;
};

struct gb_uart_info {
    uint16_t        cport;
    uint32_t        flags;

    uint8_t         updated_ms;
    uint8_t         updated_ls;
    struct gb_operation *ms_ls_operation;
    struct gb_uart_serial_state_request *ms_ls_request;
    sem_t           status_sem;
    pthread_t       status_thread;

    sq_queue_t      small_op_queue;
    sq_queue_t      large_op_queue;
    sq_queue_t      received_op_queue;
    op_node         working_op_node;
    int             need_free_node;
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

static struct sq_entry_s *get_free_entry(sq_queue_t first, sq_queue_t second)
{
    if (!sq_empty(first)) {
        return sq_remfirst(small_op_queue);
    } else if (!sq_empty(second)) {
        return sq_remfirst(small_op_queue);
    } else {
        return NULL;
    }
}
 
void uart_rx_callback(uint8_t *buffer, int length, int error)
{
    sq_entry_s *entry;
    struct op_node *node;

    if (error) {
        /* error handle */ 
    }

    info->working_op_node->size = length;
    sq_addlast(info->working_op_node->entry, received_op_queue);

    if (length < OP_BUF_SIZE_SMALL) {
        entry = get_free_entry(small_op_queue, large_op_queue)
    } else {
        entry = get_free_entry(large_op_queue, small_op_queue)
    }

    if (entry) {
        node = (struct op_node *) entry;
        device_uart_start_receiver(info->dev, node->buffer, node->op_buf_size,
                                   10, NULL, gb_uart_rx_callback);
        info->working_op_node = node;
    } else {
        info->need_free_node = 1;
    }

    sem_post(&info->rx_sem);
}

/*
 *  status change process
 */
void gb_uart_ms_ls_proc(void)
{
    struct gb_operation *operation;
    int ret;
    uint16_t current_state = 0;
    uint8_t ms_uart, ls_uart;

    ret = device_uart_get_modem_status(info->dev, &ms_uart);
    if (ms_data & MSR_DCD) {
        current_state |= GB_UART_CTRL_DCD;
    }
    if (ms_data & MSR_DSR) {
        current_state |= GB_UART_CTRL_DSR;
    }
    if (ms_data & MSR_RI) {
        current_state |= GB_UART_CTRL_RI;
    }

    ret = device_uart_get_line_status(info->dev, &ls_uart);
    if (ls_data & LSR_BI) {
        current_state |= GB_UART_CTRL_BRK;
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

/*
 *  Data receiving process
 */
void gb_uart_rx_proc(void)
{
    struct gb_operation *operation;
    struct rx_buffer *buf, *small_buf;
    sq_entry_s *entry, *small_entry;

    if (!sq_empty(data_q)) {
        entry = sq_remfirst(data_q);
        buf = (struct rx_buffer *) entry;
        if ((buf->type ==NODE_TYPE_LARGE) && (buf->size < NODE_SIZE_SMALL) &&
                    (!sq_empty(small_buf_q))) {
            small_entry = sq_remfirst(small_buf_q);
            small_buf = (struct rx_buffer *) small_entry;
            memcpy(small_buf->request, buf->request, buf->size);
            sq_addlast(entry, large_buf_q);
            ret = gb_operation_send_request(small_buf->operation, NULL, false);
            sq_addlast(entry, small_buf_q);
        } else {
            ret = gb_operation_send_request(buf->operation, NULL, false);
        }
    }

    if (info->need_free_node) {
        if ((info->last_size > NODE_TYPE_SMALL) && (!sq_empty(large_buf_q))) {
                entry = sq_remfirst(large_buf_q);
                buf = (struct rx_buffer *) entry;
        }
        else {
                if (!sq_empty(small_buf_q)) {
                        entry = sq_remfirst(small_buf_q);
                        buf = (struct rx_buffer *) entry;
                }
                else {
                        buf = NULL;
                }
        }

        if (buf) {
            device_uart_start_receiver(info->dev, buf->buffer, buf->type, 10,
                                       NULL, gb_uart_rx_callback);
        }
    }
}

/*
 *  This thread for notifying status change to peer.
 */
static void *gb_status_thread(void *data)
{
    int ret;

    while (1) {
        sem_wait(&info->status_sem);
        gb_uart_ms_ls_proc();
    }
    return NULL;
}

/*
 *  This thread for pumping receiving data to peer.
 */
static void *gb_rx_thread(void *data)
{
    int ret;

    while (1) {
        sem_wait(&info->rx_sem);
        gb_uart_rx_proc();
    }

    return NULL;
}

status int uart_status_change_exit(void)
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
    int ret;

    info->ms_ls_operation = gb_operation_create(info->cport,
                                    GB_UART_TYPE_SERIAL_STATE,
                                    sizeof(*info->ms_ls_request));
    if (operation) {
        goto init_fail;
    }

    info->ms_ls_request = (struct gb_uart_set_control_line_state_request *)
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

static int uart_rx_exit(void)
{
}

static int create_operations(int type, struct sq_queue_t queue, int num)
{
    struct gb_operation *operation;
    struct gb_uart_receive_data_request *request;
    struct rx_buffer *buf;
    int ret = OK;
    int i;

    for (i = 0; i < num; i++) {
        operation = gb_operation_create(info->cport,
                                        GB_UART_TYPE_RECEIVE_DATA,
                                        sizeof(*request) + 2 + type);
        if (operation) {
            buf = (struct rx_buffer *)malloc(rx_buffer);
            buf->operation = operation;

            request = (struct gb_uart_receive_data_request *)
                                gb_operation_get_request_payload(operation);
            buf->size   = &node->request->size;
            buf->buffer = node->request->data;
            buf->type = type;
            sq_addlast(buf->entry, queue);
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

    ret = create_operations(NODE_TYPE_LARGE, large_op_entry,
                            NUM_LARGE_OPERATION);
    if (ret) {
        uart_rx_exit();
        return ret;
    }

    ret = create_operations(NODE_TYPE_SMALL, small_op_entry,
                            NUM_SMALL_OPERATION);
    if (ret) {
        uart_rx_exit();
        return ret;
    }

    ret = sem_init(&info->rx_sem, 0, 0);
    if (ret != OK) {
        uart_rx_exit();
        return ret;
    }

    ret = pthread_create(&info->rx_thread, NULL, rx_thread, info);
    if (ret) {
        uart_rx_exit();
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
    uint8_t modem_ctrl;
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
    unsigned int i;
    struct gb_uart_info *info;
    struct gb_operation *operation;
    struct list_node *node;

    info = zalloc(sizeof(*info));
    if (!info)
        return GB_OP_NO_MEMORY;

    lldbg("GB uart info struct: 0x%08p\n", info);

    info->cport = cport;

    ret = uart_status_change_init();
    if (ret) {
        uart_status_change_exit();
        return GB_OP_MALFUNCTION;
    }

    sq_init(&info->small_op_queue);
    sq_init(&info->large_op_queue);
    sq_init(&info->received_op_queue);

    ret = uart_rx_init();
    if (ret) {
        uart_status_change_exit();
        uart_rx_exit();
        return GB_OP_MALFUNCTION;
    }

    info->dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!info->dev) {
        uart_status_change_exit();
        uart_rx_exit();
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

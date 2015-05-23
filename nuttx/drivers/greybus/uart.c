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


#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/util.h>
#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>
#include <arch/tsb/unipro.h>
#include <arch/atomic.h>
#include <apps/greybus-utils/utils.h>

#include "uart-gb.h"

#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

/* reserved operations for data buffer */
#define NUM_SMALL_OPERATION     5
#define NUM_LARGE_OPERATION     5

#define OP_BUF_SIZE_SMALL       32
#define OP_BUF_SIZE_LARGE       128

/* whether protocol request DMA support */
#define DMA_OFF                 0
#define DMA_ON                  1

/* error code */
#define SUCCESS                 0

/* signal */
#ifndef SIGKILL
#define SIGKILL     9
#endif

/**
 * struct op_node - the operation control block
 *
 * @param entry: queue entry
 * @param operation: pointer to operation
 * @param size: pointer to size of request in operation
 * @param buffer: pointer to buffer of request in operation
 * @param op_buf_size: buffer size in operation.
 */
struct op_node {
    sq_entry_t          entry;
    struct gb_operation *operation;
    uint16_t            *data_size;
    uint8_t             *buffer;
    int                 buf_size;
};

/**
 * struct gb_uart_info - uart protocol global information
 *
 * @param cport: cport from greybus
 * @param updated_ms: updated modem status from uart through callback
 * @param updated_ls: updated line status from uart through callback
 * @param last_serial_state: the last status sent to the peer.
 * @param ms_ls_operation: reserved operation for status report.
 * @param status_sem: status change semaphore
 * @param status_thread: status change thread handle
 * @param small_op_queue: available small operation queue
 * @param large_op_queue: available large operation queue
 * @param received_op_queue: received data operation queue
 * @param rx_node: op_node in receiving
 * @param need_free_node: flag for requesting a free operation in callback
 * @param last_op_size: keeps the last operation size
 * @param rx_sem: semaphore for notifying data received
 * @param rx_thread: receiving data process threed
 * @param baudrate: saving the baudrate setting for timeout computing
 * @param dev: uart driver handle
 */
struct gb_uart_info {
    uint16_t            cport;

    uint8_t             updated_ms;
    uint8_t             updated_ls;
    uint16_t            last_serial_state;
    struct gb_operation *ms_ls_operation;
    sem_t               status_sem;
    pthread_t           status_thread;

    sq_queue_t          small_op_queue;
    sq_queue_t          large_op_queue;
    sq_queue_t          received_op_queue;
    struct op_node      *rx_node;
    int                 require_node;
    int                 last_op_size;
    sem_t               rx_sem;
    pthread_t           rx_thread;

    int                 baudrate;

    struct device   *dev;
};

/* the structure for keeping protocol global data */
static struct gb_uart_info *info = NULL;


/**
* @brief Compute a timeout value for data transfer
*
* According to the bautrate, the byte/msec is
* baudrate/ 1000 / 8 for speeding up the compute, use the 1024 instead of 1000.
*
* @param length the buffer length
* @return a timeout value.
*/
static int compute_timeout(uint8_t length)
{
    int byte_per_msec = info->baudrate >> 13; /* 1024 x 8 */

    return 2 * length / byte_per_msec ; /* 2 for buffering */
}


/**
* @brief Callback for modem status change
*
* Callback for device driver modem status changes. This function can be called
* when device driver detect modem status changes.
*
* @param ms the updated modem status
* @return none
*/
static void uart_ms_callback(uint8_t ms)
{
    info->updated_ms = ms;

    sem_post(&info->status_sem);
}


/**
* @brief Callback for line status change
*
* Callback for device driver line status changes. This function can be called
* when device driver detect line status changes.
*
* @param ms the updated modem status
* @return none
*/
static void uart_ls_callback(uint8_t ls)
{
    info->updated_ls = ls;

    sem_post(&info->status_sem);
}


/**
* @brief Get a node from received queue
*
* This function return a node from the received queue. It will dequeue the node
* if there is a node for using.
*
* @param queue the received queue to ask.
* @return a node pointer or NULL for no node to return.
* @retval NULL for no node.
*/

static struct op_node *get_node_from(sq_queue_t *queue)
{
    if (sq_empty(queue)) {
        return NULL;
    } else {
        return (struct op_node *)sq_remfirst(queue);
    }
}

/**
* @brief Free entry to two queues.
*
* This function put the node back to small or large queue for reuse.
*
* @param node the node to reuse
* @return None
*/
static void recycle_op_node(struct op_node *node)
{
    sq_queue_t *queue = (node->buf_size == OP_BUF_SIZE_SMALL) ?
                            &info->small_op_queue : &info->large_op_queue;

    sq_addlast(&node->entry, queue);
}


/**
* @brief Callback for data receiving
*
* The callback function provided to device driver for being notified when
* driver received a data stream.
* It put the current operation to received queue and gets another operation to
* continue receiving. Then notifies rx thread to process.
*
* @param buffer data buffer.
* @param length received data length.
* @param error error code if the driver encontering.
* @return None.
*/
void uart_rx_callback(uint8_t *buffer, int length, int error)
{
    struct op_node *node = NULL;
    int ret = SUCCESS;
    int timeout = 0;

    if (error == -EIO) {
        /*
         * Don't need to process, the line or modem error will process
         * in ms & ls callback, anyway it just need to send the received data
         * to peer
         */
    }

    *info->rx_node->data_size = length;
    sq_addlast(&info->rx_node->entry, &info->received_op_queue);

    /*
     * Request a free operation due to the last data length
     */
    if (length < OP_BUF_SIZE_SMALL) {
        node = get_node_from(&info->small_op_queue);
        if (node == NULL) {
            node = get_node_from(&info->large_op_queue);
        }
    } else {
        node = get_node_from(&info->large_op_queue);
        if (node == NULL) {
            node = get_node_from(&info->small_op_queue);
        }
    }

    if (node != NULL) {
        timeout = compute_timeout(node->buf_size);
        ret = device_uart_start_receiver(info->dev, node->buffer,
                                         node->buf_size, timeout, DMA_OFF,
                                         NULL, uart_rx_callback);
        if (ret == SUCCESS) {
            info->rx_node = node;
        }
        else {
            recycle_op_node(node);
            node = NULL;
        }
    }

    if (node == NULL) {
        /*
         * if there is no free operation, the rx thread will engage another
         * uart receiver.
         */
        info->require_node = 1;
        info->last_op_size = (length > OP_BUF_SIZE_SMALL) ? OP_BUF_SIZE_LARGE :
                                                            OP_BUF_SIZE_SMALL;
    }

    sem_post(&info->rx_sem);
}


/**
* @brief Parse the modem and line stauts
*
* This function parses the UART modem and line status to the bitmask of
* protocol serial state.
*
* @param data the regular thread data.
* @return the parsed value of protocol serial state bitmask.
*/
static uint16_t parse_ms_ls_registers(uint8_t modem_status, uint8_t line_status)
{
    uint16_t status = 0;

    if (modem_status & MSR_DCD) {
        status |= GB_UART_CTRL_DCD;
    }
    if (modem_status & MSR_DSR) {
        status |= GB_UART_CTRL_DSR;
    }
    if (modem_status & MSR_RI) {
        status |= GB_UART_CTRL_RI;
    }
    if (line_status & LSR_BI) {
        status |= GB_UART_CTRL_BRK;
    }
    if (line_status & LSR_FE) {
        status |= GB_UART_CTRL_FRAMING;
    }
    if (line_status & LSR_PE) {
        status |= GB_UART_CTRL_PARITY;
    }
    if (line_status & LSR_OE) {
        status |= GB_UART_CTRL_OVERRUN;
    }

    return status;
}


/**
* @brief Modem and line status process thread
*
* This function is the thread for processing modem and line status change. It
* uses the operation to send the event to the peer. It only sends the required
* status for protocol, not the all status in UART.
*
* @param data the regular thread data.
* @return None.
*/
static void *uart_status_thread(void *data)
{
    uint16_t updated_status = 0;
    struct gb_uart_serial_state_request *request;
    int ret = SUCCESS;

    while (1) {
        sem_wait(&info->status_sem);

        updated_status = parse_ms_ls_registers(info->updated_ms,
                                               info->updated_ls);
        /*
         * Only send the status bits which protocol need to know to peer
         */
        if (info->last_serial_state ^ updated_status) {
            info->last_serial_state = updated_status;
            request = (struct gb_uart_serial_state_request *)
                        gb_operation_get_request_payload(info->ms_ls_operation);
            request->control = 0; /* TODO: no info in spec */
            request->data = updated_status;
            ret = gb_operation_send_request(info->ms_ls_operation, NULL, false);
            if (ret != SUCCESS) {
                /*
                 *  TODO: heandle this error
                 */
                gb_info("%s(): operation send error \n", __func__);
            }
        }
    }
    return NULL;
}


/**
* @brief Data receiving process thread
*
* This function is the thread for processing data receiving tasks. When
* it wake up, it checks the receiving queue for processing the come in data.
* If the operation is large size but the data is small, it requests a small
* operation to tranfer for saving time.
* If protocol is running out of operation, once it gets a free operation,
* it passes to driver for continuing the receiving.
*
* @param data the regular thread data.
* @return None.
*/
static void *uart_rx_thread(void *data)
{
    struct op_node *node = NULL, *small_node = NULL;
    int ret = SUCCESS;
    int timeout = 0;
    irqstate_t flags = 0;


    while (1) {
        sem_wait(&info->rx_sem);

        node = get_node_from(&info->received_op_queue);
        if (node != NULL) {
            /*
             * if the operation is large size but the data length is small,
             * change the operation for saving transfer time
             */
            if ((node->buf_size == OP_BUF_SIZE_LARGE) &&
                (*node->data_size <= OP_BUF_SIZE_SMALL) &&
                (!sq_empty(&info->small_op_queue))) {
                /* prevent callback request op at same time */
                flags = irqsave();
                small_node = get_node_from(&info->small_op_queue);
                irqrestore(flags);

                memcpy(small_node->buffer, node->buffer, *node->data_size);
                *small_node->data_size = *node->data_size;
                recycle_op_node(node);
                node = small_node;
            }
            ret = gb_operation_send_request(node->operation, NULL,
                                            false);
            if (ret != SUCCESS) {
                /*
                 * TODO: need to handle this error
                 */
                gb_info("%s(): operation send error \n", __func__);
            }
            recycle_op_node(node);
        }

        /*
         * In case the callback can't get free node
         */
        if (info->require_node == 1) {
            if (info->last_op_size == OP_BUF_SIZE_LARGE) {
                node = get_node_from(&info->large_op_queue);
                if (node == NULL) {
                    node = get_node_from(&info->small_op_queue);
                }
            }
            else {
                node = get_node_from(&info->small_op_queue);
                if (node == NULL) {
                    node = get_node_from(&info->large_op_queue);
                }
            }

            if (node != NULL) {
                timeout = compute_timeout(node->buf_size);
                ret = device_uart_start_receiver(info->dev, node->buffer,
                                                 node->buf_size, timeout,
                                                 DMA_OFF, NULL,
                                                 uart_rx_callback);
                info->require_node = 0;
                if (ret) {
                    /*
                     * TODO: need to hanlde this error
                     */
                    gb_info("%s(): driver error in receiving \n", __func__);
                }
            }
            else {
                /*
                 * TODO: need to handle this error
                 */
                gb_info("%s(): driver error in receiving \n", __func__);
            }
        }
    }

    return NULL;
}


/**
* @brief This function releases system and greybus resouce.
*
* @param None.
* @return return error code.
* @retval 0 sussess to operate.
* @retval EIO driver driver errors.
*/
static void uart_status_cb_deinit(void)
{
    if (info->status_thread) {
        pthread_kill(info->status_thread, SIGKILL);
    }

    if (info->ms_ls_operation) {
        gb_operation_destroy(info->ms_ls_operation);
    }
}


/**
* @brief Modem and line status event init process
*
* This function creates one operations and uses that request of operation
* for sending the status change event to peer.
*
* @param None.
* @return return error code.
* @retval 0 sussess to operate.
* @retval ENOMEM no memory to allicate.
* @retval EBUSY OS thread resource is busy.
*/
static int uart_status_cb_init(void)
{
    int ret = SUCCESS;

    info->ms_ls_operation = gb_operation_create(info->cport,
                                GB_UART_TYPE_SERIAL_STATE,
                                sizeof(struct gb_uart_serial_state_request));
    if (!info->ms_ls_operation) {
        return -ENOMEM;
    }

    ret = sem_init(&info->status_sem, 0, 0);
    if (ret != SUCCESS) {
        return -ret;
    }

    ret = pthread_create(&info->status_thread, NULL, uart_status_thread, info);
    if (ret != SUCCESS) {
        return -ret;
    }

    return SUCCESS;
}


/**
* @brief This function releases system and greybus resouce.
*
* @param None.
* @return return error code.
* @retval 0 sussess to operate.
* @retval EIO driver driver errors.
*/
static void uart_receiver_cb_deinit(void)
{
    struct op_node *node = NULL;

    if (info->rx_thread != 0) {
        pthread_kill(info->rx_thread, SIGKILL);
    }

    while (!sq_empty(&info->small_op_queue)) {
        node = get_node_from(&info->small_op_queue);
        gb_operation_destroy(node->operation);
        free(node);
    }

    while (!sq_empty(&info->large_op_queue)) {
        node = get_node_from(&info->large_op_queue);
        gb_operation_destroy(node->operation);
        free(node);
    }

    while (!sq_empty(&info->received_op_queue)) {
        node = get_node_from(&info->received_op_queue);
        gb_operation_destroy(node->operation);
        free(node);
    }
}


/**
* @brief Reserve operations for receiving data
*
* This function creates operations and uses those request of operation as
* data buffer prevent the data copy. It adds those operations into a queue.
*
* @param op_size the size of request in operation.
* @param queue which quese to store these operations.
* @param num how many operation in the queue.
* @return return error code.
* @retval 0 sussess to operate.
* @retval ENOMEM no memory to allicate.
*/
static int create_operations(int op_size, sq_queue_t *queue, int num)
{
    struct gb_operation *operation = NULL;
    struct gb_uart_receive_data_request *request = NULL;
    struct op_node *node = NULL;
    int i = 0;

    for (i = 0; i < num; i++) {
        operation = gb_operation_create(info->cport,
                                        GB_UART_TYPE_RECEIVE_DATA,
                                        sizeof(uint16_t) + op_size);
        if (operation) {
            node = (struct op_node *)malloc(sizeof(struct op_node));
            node->operation = operation;

            request = (struct gb_uart_receive_data_request *)
                                gb_operation_get_request_payload(operation);
            node->data_size = &request->size;
            node->buffer = request->data;
            node->buf_size = op_size;
            sq_addlast(&node->entry, queue);
        }
        else {
            return -ENOMEM;
        }
    }
    return SUCCESS;
}


/**
* @brief Receiving data process initialization
*
* This function allocates OS resource to support the data receiving
* function. It allocates two types of operations for undetermined length of
* data. The semaphore works as message queue and all tasks are done in the
* thread.
*
* @param None.
* @return return error code.
* @retval 0 sussess to operate.
* @retval ENOMEM no memory to allicate.
* @retval EBUSY OS resource is busy.
* @retval EIO OS resource is error.
*/
static int uart_receiver_cb_init(void)
{
    int ret = SUCCESS;

    sq_init(&info->small_op_queue);
    sq_init(&info->large_op_queue);
    sq_init(&info->received_op_queue);

    ret = create_operations(OP_BUF_SIZE_LARGE, &info->large_op_queue,
                            NUM_LARGE_OPERATION);
    if (ret != SUCCESS) {
        return -ENOMEM;
    }

    ret = create_operations(OP_BUF_SIZE_SMALL, &info->small_op_queue,
                            NUM_SMALL_OPERATION);
    if (ret != SUCCESS) {
        return -ENOMEM;
    }

    ret = sem_init(&info->rx_sem, 0, 0);
    if (ret != SUCCESS) {
        return ret;
    }

    ret = pthread_create(&info->rx_thread, NULL, uart_rx_thread, info);
    if (ret != SUCCESS) {
        return ret;
    }

    return SUCCESS;
}


/**
* @brief Protocol get version function.
*
* Returns the major and minor Greybus UART protocol version number supported
* by the UART device.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
*/
static uint8_t gb_uart_protocol_version(struct gb_operation *operation)
{
    struct gb_uart_proto_version_response *response = NULL;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    return GB_OP_SUCCESS;
}


/**
* @brief Protocol send data function.
*
* Requests that the UART device begin transmitting characters. One or more
* bytes to be transmitted will be supplied.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
* @retval GB_OP_MALFUNCTION the driver operation errors.
*/
static uint8_t gb_uart_send_data(struct gb_operation *operation)
{
    int ret = SUCCESS;
    int sent = 0;
    struct gb_uart_send_data_request *request;

    request = (struct gb_uart_send_data_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_start_transmitter(info->dev, request->data,
                                        request->size, 0, 0, &sent, NULL);
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}


/**
* @brief Protocol receive data function.
*
* Receive data from the UART. One or more bytes will be supplied.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
*/
static uint8_t gb_uart_receive_data(struct gb_operation *operation)
{
    /*
     * TODO: In spec, the Greybus UART serial state operation is initiated by
     * the Module implementing the UART Protocol, needs to clarify.
     */
    return GB_OP_SUCCESS;
}


/**
* @brief Protocol set line coding function.
*
* Sets the line settings of the UART to the specified baud rate, format,
* parity, and data bits.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
* @retval GB_OP_INVALID invalid input parameter.
* @retval GB_OP_MALFUNCTION the driver operation errors.
*/
static uint8_t gb_uart_set_line_coding(struct gb_operation *operation)
{
    int ret = SUCCESS;
    int baud, parity, databits, stopbit, flow;
    struct gb_serial_line_coding_request *request = NULL;

    request = (struct gb_serial_line_coding_request *)
                  gb_operation_get_request_payload(operation);

    baud = request->rate;
    info->baudrate = baud;

    switch (request->format) {
    case GB_SERIAL_1_STOP_BITS:
        stopbit = ONE_STOP_BIT;
        break;
    case GB_SERIAL_1_5_STOP_BITS:
        stopbit = ONE5_STOP_BITS;
        break;
    case GB_SERIAL_2_STOP_BITS:
        stopbit = TWO_STOP_BITS;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    switch (request->parity) {
    case GB_SERIAL_NO_PARITY:
        parity = NO_PARITY;
        break;
    case GB_SERIAL_ODD_PARITY:
        parity = ODD_PARITY;
        break;
    case GB_SERIAL_EVEN_PARITY:
        parity = EVEN_PARITY;
        break;
    case GB_SERIAL_MARK_PARITY:
        parity = MARK_PARITY;
        break;
    case GB_SERIAL_SPACE_PARITY:
        parity = SPACE_PARITY;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    if (request->data > 8 || request->data < 5) {
        return GB_OP_INVALID;
    }

    databits = request->data;

    flow = 0;

    ret = device_uart_set_configuration(info->dev, baud, parity, databits,
                                        stopbit, flow);
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}


/**
* @brief Protocol set RTS & DTR line status function.
*
* Controls RTS and DTR line states of the UART.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
* @retval GB_OP_MALFUNCTION the driver operation errors.
*/
static uint8_t gb_uart_set_control_line_state(struct gb_operation *operation)
{
    int ret = SUCCESS;
    uint8_t modem_ctrl = 0;
    struct gb_uart_set_control_line_state_request *request = NULL;

    request = (struct gb_uart_set_control_line_state_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_get_modem_ctrl(info->dev, &modem_ctrl);
    if (ret != SUCCESS) {
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
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}


/**
* @brief Protocol send break function.
*
* Requests that the UART generate a break condition on its transmit line.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
* @retval GB_OP_MALFUNCTION the driver operation errors.
*/
static uint8_t gb_uart_send_break(struct gb_operation *operation)
{
    int ret = SUCCESS;
    struct gb_uart_set_break_request *request = NULL;

    request = (struct gb_uart_set_break_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(info->dev, request->state);
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}


/**
* @brief Protocol serial state function.
*
* Receives the state of the UART's control lines and any line errors that
* might have occurred.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to operate.
*/
static uint8_t gb_uart_serial_state(struct gb_operation *operation)
{
    /*
     * TODO: In spec, the Greybus UART serial state operation is initiated by
     * the Module implementing the UART Protocol.
     */
    return GB_OP_SUCCESS;
}


/**
* @brief Protocol initialization function.
*
* This function perform the protocto initialization function, such as open
* the cooperation device driver, launch threads, create buffers etc.
*
* @param cport the number of cport
* @return return error code
* @retval GB_OP_SUCCESS sussess to initialization
* @retval GB_OP_NO_MEMORY no memory to allicate.
* @retval GB_OP_MALFUNCTION driver operation errors.
*/
static int gb_uart_init(unsigned int cport)
{
    int ret = SUCCESS;
    uint8_t ms = 0, ls = 0;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return GB_OP_NO_MEMORY;
    }

    gb_info("%s(): GB uart info struct: 0x%08p \n", __func__, info);

    info->cport = cport;

    ret = uart_status_cb_init();
    if (ret != SUCCESS) {
        goto init_err;
    }

    ret = uart_receiver_cb_init();
    if (ret != SUCCESS) {
        goto init_err;
    }

    info->dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!info->dev) {
        goto init_err;
    }

    /* update serial status */
    ret = device_uart_get_modem_status(info->dev, &ms);
    if (ret != SUCCESS) {
        goto init_err;
    }

    ret = device_uart_get_modem_status(info->dev, &ls);
    if (ret != SUCCESS) {
        goto init_err;
    }

    info->last_serial_state = parse_ms_ls_registers(ms, ls);

    ret = device_uart_attach_ms_callback(info->dev, uart_ms_callback);
    if (ret != SUCCESS) {
        goto init_err;
    }

    ret = device_uart_attach_ls_callback(info->dev, uart_ls_callback);
    if (ret != SUCCESS) {
        goto init_err;
    }

    /* trigger the first receiving */
    info->require_node = 1;
    info->last_op_size = OP_BUF_SIZE_LARGE;
    sem_post(&info->rx_sem);

    return GB_OP_SUCCESS;

init_err:
    uart_status_cb_deinit();
    uart_receiver_cb_deinit();
    return GB_OP_MALFUNCTION;
}


/**
* @brief Protocol exit function.
*
* This function can be called when protocol terminated.
*
* @param cport the number of cport.
* @return return error code.
* @retval GB_OP_SUCCESS sussess to get capabilities.
*/
static void gb_uart_exit(unsigned int cport)
{
    device_uart_attach_ms_callback(info->dev, NULL);

    device_uart_attach_ls_callback(info->dev, NULL);

    uart_status_cb_deinit();

    uart_receiver_cb_deinit();

    device_close(info->dev);
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


/**
* @brief Protocol registering function.
*
* This function can be called by greybus to register the UART protocol.
*
* @param cport the number of cport.
* @return none.
*/
void gb_uart_register(int cport)
{
    gb_info("%s(): cport %d \n", __func__, cport);
    gb_register_driver(cport, &uart_driver);
}

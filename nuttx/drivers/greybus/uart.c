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

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/util.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>

#include "uart-gb.h"


#include <nuttx/kmalloc.h>


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
    uint16_t            *size;
    uint8_t             *buffer;
    int                 op_buf_size;
};

/**
 * struct gb_uart_info - uart protocol global information
 *
 * @param cport: cport from greybus
 * @param updated_ms: updated modem status from uart through callback
 * @param updated_ls: updated line status from uart through callback
 * @param last_serial_state: the last status sent to the peer.
 * @param ms_ls_operation: reserved operation for status report.
 * @param ms_ls_request: request in reserved operation.
 * @param status_sem: status change semaphore
 * @param status_thread: status change thread handle
 * @param small_op_queue: available small operation queue
 * @param large_op_queue: available large operation queue
 * @param received_op_queue: received data operation queue
 * @param working_op_node: op_node working in uart driver
 * @param need_free_node: flag for requesting a free operation in callback
 * @param last_op_size: keeps the last operation size
 * @param rx_sem: semaphore for notifying data received
 * @param rx_thread: receiving data process threed
 * @param dev: uart driver handle
 */
struct gb_uart_info {
    uint16_t            cport;

    uint8_t             updated_ms;
    uint8_t             updated_ls;
    uint16_t            last_serial_state;
    struct gb_operation *ms_ls_operation;
    struct gb_uart_serial_state_request *ms_ls_request;
    sem_t               status_sem;
    pthread_t           status_thread;

    sq_queue_t          small_op_queue;
    sq_queue_t          large_op_queue;
    sq_queue_t          received_op_queue;
    struct op_node      *working_op_node;
    int                 need_free_node;
    int                 last_op_size;
    sem_t               rx_sem;
    pthread_t           rx_thread;

    struct device   *dev;
};

/* the structure for keeping protocol global data */
static struct gb_uart_info *info;


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
* @brief Get a free entry from two queues.
*
* This function return a available entry from one of the two queues. If first
* has free entry, it dequeue the entry and return. If first queue is empty, it
* looks for second. If both doesn't have free entry. it return NULL.
*
* @param first the first queue for looking.
* @param second the second queue for looking.
* @return an available entry
* @retval NULL for no entry available.
*/
static sq_entry_t *get_free_entry(sq_queue_t *first, sq_queue_t *second)
{
    return sq_empty(first) ? sq_empty(second) ?  NULL : sq_remfirst(second) :
                             sq_remfirst(first);
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
    sq_queue_t *queue = (node->op_buf_size == OP_BUF_SIZE_SMALL) ?
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
    sq_entry_t *entry = NULL;
    struct op_node *node = NULL;
    int ret = SUCCESS;

    if (error == -EIO) {
        /*
         * Don't need to process, the line or modem error will process
         * in ms & ls callback, anyway it just need to send the received data
         * to peer
         */
    }

    *info->working_op_node->size = length;
    sq_addlast(&info->working_op_node->entry, &info->received_op_queue);

    /*
     * Request a free operation due to the last data length
     */
    if (length < OP_BUF_SIZE_SMALL) {
        entry = get_free_entry(&info->small_op_queue, &info->large_op_queue);
    } else {
        entry = get_free_entry(&info->large_op_queue, &info->small_op_queue);
    }

    if (entry != NULL) {
        node = (struct op_node *) entry;
        ret = device_uart_start_receiver(info->dev, node->buffer,
                                         node->op_buf_size,
                                         10, DMA_OFF, NULL, uart_rx_callback);
        if (ret == SUCCESS) {
            info->working_op_node = node;
        }
        else {
            recycle_op_node(node);
            entry = NULL;
        }
    }

    if (entry == NULL) {
        /*
         * if there is no free operation, the rx thread will engage another
         * uart receiver.
         */
        info->need_free_node = 1;
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
* This function is the thread for processing modem and line status
* change. It uses the operation to send the event to the peer. It only sends the
* required status for protocol, not the all status in UART.
*
* @param data the regular thread data.
* @return None.
*/
static void *uart_status_thread(void *data)
{
    uint16_t updated_status = 0;
    int ret = SUCCESS;

    while (1) {
        sem_wait(&info->status_sem);

        updated_status = parse_ms_ls_registers(info->updated_ms,
                                               info->updated_ls);
        /*
         * Only send the status bits protocol cares and have changed to peer
         */
        if (info->last_serial_state ^ updated_status) {
            info->last_serial_state = updated_status;

            info->ms_ls_request->control = 0; /* TODO: no info in spec */
            info->ms_ls_request->data = updated_status;
            ret = gb_operation_send_request(info->ms_ls_operation, NULL, false);
            if (ret != SUCCESS) {
                /*
                 *  TODO: heandle this error
                 */
                lldbg("-status_thread: Can't report event : %d\n", ret);
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
    sq_entry_t *entry = NULL, *small_entry = NULL;
    int ret = SUCCESS;
    irqstate_t flags = 0;

    while (1) {
        sem_wait(&info->rx_sem);

        if (!sq_empty(&info->received_op_queue)) {
            entry = sq_remfirst(&info->received_op_queue);
            node = (struct op_node *) entry;
            /*
             * if the operation is large size but the data length is small,
             * change the operation for saving transfer time
             */
            if ((node->op_buf_size == OP_BUF_SIZE_LARGE) &&
                (*node->size <= OP_BUF_SIZE_SMALL) &&
                (!sq_empty(&info->small_op_queue))) {
                /* prevent callback request op at same time */
                flags = irqsave();
                small_entry = sq_remfirst(&info->small_op_queue);
                irqrestore(flags);

                small_node = (struct op_node *) small_entry;
                memcpy(small_node->buffer, node->buffer, node->size);
                *small_node->size = *node->size;
                recycle_op_node(node);

                ret = gb_operation_send_request(small_node->operation, NULL,
                                                false);
                if (ret != SUCCESS) {
                    /*
                     * TODO: need to handle this error
                     */
                    lldbg("-rx_thread: Can't send request : %d\n", ret);
                }
                recycle_op_node(small_node);
            } else {
                ret = gb_operation_send_request(node->operation, NULL, false);
                if (ret != SUCCESS) {
                    /*
                     * TODO: need to handle this error
                     */
                    lldbg("-rx_thread: Can't send request : %d\n", ret);
                }
                recycle_op_node(node);
            }
        }

        /*
         * In case the callback can't get free node
         */
        if (info->need_free_node == 1) {
            if (info->last_op_size == OP_BUF_SIZE_LARGE) {
                entry = get_free_entry(&info->large_op_queue,
                                       &info->small_op_queue);
            }
            else {
                entry = get_free_entry(&info->small_op_queue,
                                       &info->large_op_queue);
            }
            if (entry != NULL) {
                node = (struct op_node *) entry;
                ret = device_uart_start_receiver(info->dev, node->buffer,
                                                 node->op_buf_size, 10, 0, NULL,
                                                 uart_rx_callback);
                info->need_free_node = 0;
                if (ret) {
                    /*
                     * TODO: need to hanlde this error
                     */
                    lldbg("-rx_thread: driver error in receiving : %d\n", ret);
                }
            }
            else {
                /*
                 * TODO: need to handle this error
                 */
                lldbg("-rx_thread: No entry to continue : %d\n", ret);
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
                                    sizeof(*info->ms_ls_request));
    if (!info->ms_ls_operation) {
        return -ENOMEM;
    }

    info->ms_ls_request = (struct gb_uart_serial_state_request *)
                    gb_operation_get_request_payload(info->ms_ls_operation);

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
    sq_entry_t *entry = NULL;

    if (info->rx_thread != NULL) {
        pthread_kill(info->rx_thread, SIGKILL);
    }

    while (!sq_empty(&info->small_op_queue)) {
        entry = sq_remfirst(&info->small_op_queue);
        node = (struct op_node *) entry;
        gb_operation_destroy(node->operation);
        free(node);
    }

    while (!sq_empty(&info->large_op_queue)) {
        entry = sq_remfirst(&info->large_op_queue);
        node = (struct op_node *) entry;
        gb_operation_destroy(node->operation);
        free(node);
    }

    while (!sq_empty(&info->received_op_queue)) {
        entry = sq_remfirst(&info->received_op_queue);
        node = (struct op_node *) entry;
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
                                        2 + op_size);
        if (operation) {
            node = (struct op_node *)malloc(sizeof(struct op_node));
            node->operation = operation;

            request = (struct gb_uart_receive_data_request *)
                                gb_operation_get_request_payload(operation);
            node->size = &request->size;
            node->buffer = request->data;
            node->op_buf_size = op_size;
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
	lldbg("gb_uart_protocol_version +++  \n"); /* XXX */
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    lldbg("gb_uart_protocol_version ---  \n"); /* XXX */
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
	lldbg("gb_uart_send_data +++  \n"); /* XXX */
    request = (struct gb_uart_send_data_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_start_transmitter(info->dev, request->data,
                                        request->size, 0, 0, &sent, NULL);
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }
	lldbg("gb_uart_send_data ---  \n"); /* XXX */
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
    
    lldbg("gb_uart_set_line_coding +++  \n"); /* XXX */
    
    request = (struct gb_serial_line_coding_request *)
                  gb_operation_get_request_payload(operation);

    baud = request->rate;

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
        ret = GB_OP_INVALID;
        /*
         * The spec doesn't describe how to response the invalid value
         * need to clarify
         */
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
        ret = GB_OP_INVALID;
        /*
         * The spec doesn't describe how to response the invalid value
         * need to clarify
         */
        break; 
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
    
     lldbg("gb_uart_set_control_line_state +++  \n"); /* XXX */

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

	lldbg("gb_uart_set_control_line_state ---  \n"); /* XXX */
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
	lldbg("gb_uart_send_break +++  \n"); /* XXX */
	
    request = (struct gb_uart_set_break_request *)
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(info->dev, request->state);
    if (ret != SUCCESS) {
        return GB_OP_MALFUNCTION;
    }
	lldbg("gb_uart_send_break ---  \n"); /* XXX */
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

    lldbg("GB uart info struct: 0x%08p\n", info);

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
    info->need_free_node = 1;
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
static int gb_uart_exit(unsigned int cport)
{
    device_uart_attach_ms_callback(info->dev, NULL);

    device_uart_attach_ls_callback(info->dev, NULL);

    uart_status_cb_deinit();

    uart_receiver_cb_deinit();

    device_close(info->dev);

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
    lldbg("gb_uart_register: cport %d\n", cport);
    gb_register_driver(cport, &uart_driver);
}

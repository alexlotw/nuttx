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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include <arch/byteorder.h>

#include "sdio-gb.h"

#define GB_SDIO_VERSION_MAJOR 0
#define GB_SDIO_VERSION_MINOR 1

/**
 * SDIO protocol private information.
 */
struct gb_sdio_info {
    /** CPort from greybus */
    uint16_t cport;
    /** SDIO driver handle */
    struct device *dev;
};

static struct gb_sdio_info *info = NULL;

/**
 * @brief Event callback function for SDIO host controller driver
 *
 * @param event Event type.
 * @return 0 on success, negative errno on error.
 */
static int event_callback(uint8_t event)
{
    struct gb_operation *operation;
    struct gb_sdio_event_request *request;

    //lldbg("event_callback() \n");
    
    operation = gb_operation_create(info->cport, GB_SDIO_TYPE_EVENT,
                                    sizeof(*request));
    if (!operation) {
        return -EIO;
    }
    
    request = gb_operation_get_request_payload(operation);
    request->event = event;

    gb_operation_send_request(operation, NULL, false);

    //lldbg("event_callback sent out 0x%x \n", request->event);
    
    gb_operation_destroy(operation);

    return 0;
}

/**
 * @brief Protocol get version function.
 *
 * Returns the major and minor Greybus SDIO protocol version number
 * supported by the SDIO.
 *
 * @param operation The pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_version(struct gb_operation *operation)
{
    struct gb_sdio_proto_version_response *response = NULL;

    //lldbg("gb_sdio_protocol_version() \n");

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SDIO_VERSION_MAJOR;
    response->minor = GB_SDIO_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol gets capabilities of the SDIO host controller.
 *
 * Protocol to get the capabilities of SDIO host controller such as support bus
 * width, VDD value and clock.
 *
 * @param operation Pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_get_capabilities(struct gb_operation *operation)
{
    struct gb_sdio_get_capabilities_response *response;
    struct sdio_cap cap;
    int ret;

    //lldbg("gb_sdio_protocol_get_capabilities() \n");
    
    ret = device_sdio_get_capabilities(info->dev, &cap);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /**
     * Currently, the protocol buffer size must be under 1024 bytes, with the
     * buffer information, therefore only 512 bytes is allowed for one block.
     */
    if (cap.max_blk_count * cap.max_blk_size > 1024) {
        cap.max_blk_count = 1;
        cap.max_blk_size = 512;
    }

    //cap.max_blk_count = 60000;
    //cap.max_blk_count = 65535;
    cap.max_blk_count = 1;
    cap.max_blk_size = 512;

    cap.ocr = 0xFFFFFFFF;
    response->caps = cpu_to_le32(cap.caps);
    response->ocr = cpu_to_le32(cap.ocr);
    response->max_blk_count = cpu_to_le16(cap.max_blk_count);
    response->max_blk_size = cpu_to_le16(cap.max_blk_size);

    //lldbg("get_capabilities: caps: 0x%x \n", response->caps);
    //lldbg("get_capabilities: ocr: 0x%x \n", response->ocr);
    //lldbg("get_capabilities: max_blk_count: %d \n", response->max_blk_count);
    //lldbg("get_capabilities: max_blk_size: %d \n", response->max_blk_size);
    
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set the SDIO host configuration.
 *
 * Set ios operation allows the request to setup parameters lo SDIO controller.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_set_ios(struct gb_operation *operation)
{
    struct gb_sdio_set_ios_request *request;
    struct sdio_ios ios;
    int ret;

    //lldbg("gb_sdio_protocol_set_ios() \n");

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    ios.clock = le32_to_cpu(request->clock);
    ios.vdd = le32_to_cpu(request->vdd);
    ios.bus_mode = request->bus_mode;
    ios.power_mode = request->power_mode;
    ios.bus_width = request->bus_width;
    ios.timing = request->timing;
    ios.signal_voltage = request->signal_voltage;
    ios.drv_type = request->drv_type;

    //lldbg("set_ios: clock: %d \n", ios.clock);
    //lldbg("set_ios: vdd: 0x%x \n", ios.vdd);
    //lldbg("set_ios: bus_mode: 0x%x \n", ios.bus_mode);
    //lldbg("set_ios: power_mode: 0x%x \n", ios.power_mode);
    //lldbg("set_ios: bus_width: %d \n", ios.bus_width);
    //lldbg("set_ios: signal_voltage: 0x%x \n", ios.signal_voltage);
    //lldbg("set_ios: drv_type: 0x%x \n", ios.drv_type);
    
    ret = device_sdio_set_ios(info->dev, &ios);
    if (ret) {
        //lldbg("device_sdio_set_ios ERR \n");
        return GB_OP_UNKNOWN_ERROR;
    }

    //lldbg("device_sdio_set_ios OK \n");
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol requests to send command.
 *
 * Sending a single command to the SD card through the SDIO host controller.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_command(struct gb_operation *operation)
{
    struct gb_sdio_command_request *request;
    struct gb_sdio_command_response *response;
    struct sdio_cmd cmd;
    uint32_t resp[4];
    int i, ret;

    //lldbg("gb_sdio_protocol_command() ++++++++++++++++++++++++++\n");

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    for (i = 0; i < 4; i++) {
        resp[i] = 0;
    }

    cmd.cmd = request->cmd;
    cmd.cmd_flags = request->cmd_flags;
    cmd.cmd_type = request->cmd_type;
    cmd.cmd_arg = le32_to_cpu(request->cmd_arg);
    cmd.data_cmd = request->data_cmd;
    cmd.data_blocks = le16_to_cpu(request->data_blocks);
    cmd.data_blksz = le16_to_cpu(request->data_blksz);
    cmd.resp = resp;

    //lldbg("cmd:cmd = %d \n", cmd.cmd);
    //lldbg("cmd:cmd_flags = 0x%x \n", cmd.cmd_flags);
    //lldbg("cmd:cmd_type = 0x%x \n", cmd.cmd_type);
    //lldbg("cmd:cmd_arg = 0x%x \n", cmd.cmd_arg);
    
    ret = device_sdio_send_cmd(info->dev, &cmd);
    if (ret == -EINVAL) {
        //lldbg("device_sdio_send_cmd ERR \n");
        return GB_OP_UNKNOWN_ERROR;
    }
    if (ret == -ETIMEDOUT) {
        //lldbg("device_sdio_send_cmd TIMEOUT \n");
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        //lldbg("gb_operation_alloc_response ERR \n");
        return GB_OP_NO_MEMORY;
    }

    /*
     * Per the discussion for the order of the 32 bits response
     * value, the host kernel treats order of response as SD
     * spec defined, for example:
     *
     * bit 31: out of range
     * bit 8 : ready for data
     *
     * The SD controller driver must keep this order in the
     * returned resp.
     */
    for (i = 0; i < 4; i++) {
        response->resp[i] = cpu_to_le32(resp[i]);
        //lldbg("response->resp[%d] = 0x%x \n", i, resp[i]);
    }

    //lldbg("gb_sdio_protocol_command() OK -------------------------- \n");
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol request to send and receive data.
 *
 * SDIO transfer operation allows the requester to send or receive data blocks
 * and shall be preceded by a Greybus Command Request for data transfer command
 *
 * @param operation The pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_transfer(struct gb_operation *operation)
{
    struct gb_sdio_transfer_request *request;
    struct gb_sdio_transfer_response *response;
    struct sdio_transfer transfer;
    int ret;

    //lldbg("gb_sdio_protocol_transfer() ================================= \n");

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    //lldbg("request->data_blocks = %d \n", request->data_blocks);
    //lldbg("request->data_blksz = %d \n", request->data_blksz);

    transfer.blocks = le16_to_cpu(request->data_blocks);
    transfer.blksz = le16_to_cpu(request->data_blksz);
    transfer.dma = NULL; /* NO DMA so far */
    transfer.callback = NULL; /* NO non-blocking transfer */

    if (request->data_flags & GB_SDIO_DATA_WRITE) {
        if (!request->data) {
            return GB_OP_INVALID;
        }
        transfer.data = request->data;
        ret = device_sdio_write(info->dev, &transfer);
        if (ret) {
            return GB_OP_UNKNOWN_ERROR;
        }
        response = gb_operation_alloc_response(operation, sizeof(*response));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        response->data_blocks = cpu_to_le16(transfer.blocks);
        response->data_blksz = cpu_to_le16(transfer.blksz);
    }
    else if (request->data_flags & GB_SDIO_DATA_READ) {
        response = gb_operation_alloc_response(operation, sizeof(*response) +
                                                          transfer.blocks *
                                                          transfer.blksz);
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        //lldbg("start to read... \n");
        transfer.data = response->data;
        ret = device_sdio_read(info->dev, &transfer);
        if (ret) {
            //lldbg("device_sdio_read ERR \n");
            return GB_OP_UNKNOWN_ERROR;
        }
        //lldbg("device_sdio_read OK \n");

        /*{
            uint32_t *datap = (uint32_t *)transfer.data;

            lldbg("******** data 0x%08x, 0x%08x \n", datap[0], datap[1]);
        }*/
        
        response->data_blocks = cpu_to_le16(transfer.blocks);
        response->data_blksz = cpu_to_le16(transfer.blksz);
    }
    else {
        return GB_OP_UNKNOWN_ERROR;
    }

    //lldbg("gb_sdio_protocol_transfer()------------------------------ \n");

    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus SDIO protocol initialize function
 *
 * This function performs the protocol initialization function, such as open
 * the cooperation device driver, attach callback, create buffers etc.
 *
 * @param cport CPort number.
 * @return 0 on success, negative errno on error.
 */
static int gb_sdio_init(unsigned int cport)
{
    int ret;

    //lldbg("gb_sdio_init() \n");

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    info->cport = cport;

    info->dev = device_open(DEVICE_TYPE_SDIO_HW, 0);
    if (!info->dev) {
        ret = -ENODEV;
        goto err_free_info;
    }

    ret = device_sdio_attach_callback(info->dev, event_callback);
    if (ret) {
        goto err_close_device;
    }

    //lldbg("gb_sdio_init() success \n");
        
    return 0;

err_close_device:
    device_close(info->dev);
err_free_info:
    free(info);

    return ret;
}

/**
 * @brief Protocol exit function.
 *
 * This function can be called when protocol terminated.
 *
 * @param cport CPort number.
 * @return None.
 */
static void gb_sdio_exit(unsigned int cport)
{
    device_sdio_attach_callback(info->dev, NULL);

    device_close(info->dev);

    free(info);
}

/**
 * @brief Greybus SDIO protocol operation handler
 */
static struct gb_operation_handler gb_sdio_handlers[] = {
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_VERSION, gb_sdio_protocol_version),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_GET_CAPABILITIES,
               gb_sdio_protocol_get_capabilities),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_SET_IOS, gb_sdio_protocol_set_ios),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_COMMAND, gb_sdio_protocol_command),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_TRANSFER, gb_sdio_protocol_transfer),
};

static struct gb_driver sdio_driver = {
    .init              = gb_sdio_init,
    .exit              = gb_sdio_exit,
    .op_handlers       = (struct gb_operation_handler *)gb_sdio_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_sdio_handlers),
};

/**
 * @brief Register Greybus SDIO protocol
 *
 * @param cport cport number
 */
void gb_sdio_register(int cport)
{
    gb_register_driver(cport, &sdio_driver);
}

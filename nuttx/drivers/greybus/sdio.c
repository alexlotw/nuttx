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

#include <apps/greybus-utils/utils.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio.h>
#include <nuttx/greybus/greybus.h>
#include <arch/byteorder.h>
#include "sdio-gb.h"

#define GB_SDIO_VERSION_MAJOR 0
#define GB_SDIO_VERSION_MINOR 1
#define GB_SDIO_DEV_ID 0

static struct device *sdio_dev = NULL;

/**
 * @brief Returns the major and minor Greybus SDIO protocol version number
 * supported by the SDIO.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_type_protocol_version(struct gb_operation *operation)
{
    struct gb_sdio_version_response *response = NULL;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SDIO_VERSION_MAJOR;
    response->minor = GB_SDIO_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns capabilities of the SDIO card.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_type_protocol_get_capabilities(struct gb_operation
                                                      *operation)
{
    struct gb_sdio_get_capabilities_response *response = NULL;
    uint32_t caps = 0;
    uint32_t ocr = 0;
    uint16_t max_blk_count = 0;
    uint16_t max_blk_size = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_sdio_get_capabilities(sdio_dev, &caps, &ocr, &max_blk_count,
                                       &max_blk_size);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response->caps = cpu_to_le32(caps);
    response->ocr = cpu_to_le32(ocr);
    response->max_blk_count = cpu_to_le16(max_blk_count);
    response->max_blk_size = cpu_to_le16(max_blk_size);

    return GB_OP_SUCCESS;
}

/**
 * @brief Set Ios operation allows the requester to setup parameters listed in
 * to SDIO controller.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_type_protocol_set_ios(struct gb_operation *operation)
{
    struct gb_sdio_set_ios_request *request =
                                    gb_operation_get_request_payload(operation);
    int ret = 0;

    ret = device_sdio_set_ios(sdio_dev, le32_to_cpu(request->clock),
                              le32_to_cpu(request->vdd), request->bus_mode,
                              request->power_mode, request->bus_width,
                              request->timing, request->signal_voltage,
                              request->drv_type);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Request of sending commands.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_type_protocol_command(struct gb_operation *operation)
{
    struct gb_sdio_command_request *request =
                                    gb_operation_get_request_payload(operation);
    struct gb_sdio_command_response *response;
    uint32_t resp[4];
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_sdio_sendcmd(sdio_dev, request->cmd, request->cmd_flags,
                              request->cmd_type, le32_to_cpu(request->cmd_arg),
                              resp);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response->resp[0] = cpu_to_le32(resp[0]);
    response->resp[1] = cpu_to_le32(resp[1]);
    response->resp[2] = cpu_to_le32(resp[2]);
    response->resp[3] = cpu_to_le32(resp[3]);

    return GB_OP_SUCCESS;
}

/**
 * @brief Request of sending data and receiving data.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_type_protocol_transfer(struct gb_operation *operation)
{
    struct gb_sdio_transfer_request *request =
                                    gb_operation_get_request_payload(operation);
    struct gb_sdio_transfer_response *response;
    uint32_t len = 0;
    int ret = 0;
    uint16_t data_blocks;
    uint16_t data_blksz;

    if (request->data_flags & GB_SDIO_DATA_WRITE) {
        response = gb_operation_alloc_response(operation, sizeof(*response));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }

        data_blocks = le16_to_cpu(request->data_blocks);
        data_blksz = le16_to_cpu(request->data_blksz);

        ret = device_sdio_data_write(sdio_dev, &data_blocks, &data_blksz,
                                     request->data, 0, NULL);
        if (ret) {
            return GB_OP_UNKNOWN_ERROR;
        }

        response->data_blocks = cpu_to_le16(data_blocks);
        response->data_blksz = cpu_to_le16(data_blksz);
    } else if (request->data_flags & GB_SDIO_DATA_READ) {
        len = le16_to_cpu(request->data_blocks) *
              le16_to_cpu(request->data_blksz);
        response = gb_operation_alloc_response(operation,
                                               len + sizeof(*response));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }

        data_blocks = le16_to_cpu(request->data_blocks);
        data_blksz = le16_to_cpu(request->data_blksz);

        ret = device_sdio_data_read(sdio_dev, &data_blocks, &data_blksz,
                                    response->data, 0, NULL);
        if (ret) {
            return GB_OP_UNKNOWN_ERROR;
        }

        response->data_blocks = cpu_to_le16(data_blocks);
        response->data_blksz = cpu_to_le16(data_blksz);
    } else {
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus SDIO protocol initialize function
 *
 * @param cport - cport number
 * @return 0 on success, negative errno on error.
 */
static int gb_sdio_init(unsigned int cport)
{
    if (!sdio_dev) {
        sdio_dev = device_open(DEVICE_TYPE_SDIO_HW, 0);
        if (!sdio_dev) {
            return -EIO;
        }
    }
    return 0;
}

/**
 * @brief Greybus SDIO protocol deinitialize function
 *
 * @param cport - cport number
 */
static void gb_sdio_exit(unsigned int cport)
{
    if (sdio_dev) {
        device_close(sdio_dev);
        sdio_dev = NULL;
    }
}

/**
 * @brief Greybus SDIO protocol operation handler
 */
static struct gb_operation_handler gb_sdio_handlers[] = {
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_VERSION, gb_sdio_type_protocol_version),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_GET_CAPABILITIES,
               gb_sdio_type_protocol_get_capabilities),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_SET_IOS, gb_sdio_type_protocol_set_ios),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_COMMAND, gb_sdio_type_protocol_command),
    GB_HANDLER(GB_SDIO_TYPE_PROTOCOL_TRANSFER, gb_sdio_type_protocol_transfer),
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

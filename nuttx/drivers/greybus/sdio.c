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

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SDIO_VERSION_MAJOR;
    response->minor = GB_SDIO_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol request to get capabilities of the SDIO host.
 *
 * Protocol to get the capabilities of SDIO host controller such as support bus
 * width, VDD value and clock.
 *
 * @param operation The pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_get_capabilities(struct gb_operation *operation)
{

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol request to set the SDIO host configuration.
 *
 * Set Ios operation allows the requester to setup parameters listed in to SDIO
 * controller.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_set_ios(struct gb_operation *operation)
{

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol to request sending command.
 *
 * Sending a single command to the SD card through the SDIO host controller.
 *
 * @param operation - pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_command(struct gb_operation *operation)
{

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol request to send and receive data.
 *
 * SDIO Transfer operation allows the requester to send or receive data blocks
 * and shall be preceded by a Greybus Command Request for data transfer command
 *
 * @param operation The pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_sdio_protocol_transfer(struct gb_operation *operation)
{

    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus SDIO protocol initialize function
 *
 * This function perform the protocto initialization function, such as open
 * the cooperation device driver, launch threads, create buffers etc.
 *
 * @param cport CPort number.
 * @return 0 on success, negative errno on error.
 */
static int gb_sdio_init(unsigned int cport)
{
    int ret;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    //gb_debug("%s(): GB sdio info struct: 0x%p \n", __func__, info);

    info->cport = cport;

    info->dev = device_open(DEVICE_TYPE_SDIO_HW, 0);
    if (!info->dev) {
        ret = -ENODEV;
    }

    return 0;

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

/**
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/device.h>
#include <nuttx/device_camera.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>

#include "camera-gb.h"

#define GB_CAMERA_VERSION_MAJOR 0
#define GB_CAMERA_VERSION_MINOR 1

static struct device *camera_dev = NULL;

/**
 * @brief Returns the major and minor Greybus Camera Protocol version number
 *
 * This operation returns the major and minor version number supported by
 * Greybus Camera Protocol
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_protocol_version(struct gb_operation *operation)
{
    struct gb_camera_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_CAMERA_VERSION_MAJOR;
    response->minor = GB_CAMERA_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get Camera capabilities
 *
 * This operation retrieves the list of capabilities of the Camera Module.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_capabilities(struct gb_operation *operation)
{
    struct gb_camera_capabilities_response *response;
    struct camera_module_capabilities capabilities;
    uint16_t size = 0;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* camera module capabilities */
    ret = device_camera_capabilities(camera_dev, &size, &capabilities);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    /* Retrieves camera module capabilities */
    #if 0
    response->metadata_greybus = cpu_to_le32(capabilities.metadata_greybus);
    response->metadata_mipi    = cpu_to_le32(capabilities.metadata_mipi);
    response->still_image      = cpu_to_le32(capabilities.still_image);
    response->jpeg             = cpu_to_le32(capabilities.jpeg);
    #endif
    
    return GB_OP_SUCCESS;
}

/**
 * @brief configure streams
 *
 * The Configure Streams operation configures or unconfigures the Camera Module
 * to prepare or stop video capture.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_configure_streams(struct gb_operation *operation)
{
    struct gb_camera_configure_streams_request *request;
    struct gb_camera_configure_streams_response *response;
    struct camera_streams_cfg scfg;    
    //struct stream_config streams;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* set streams configuration */
    ret = device_camera_configure_streams(camera_dev, request->num_streams, &scfg.stream_feature);
    if (ret) {
        return GB_OP_INVALID;
    }

    /* get streams configuration */
    response->num_streams                        = cpu_to_le32(scfg.num_streams);
    response->padding                            = cpu_to_le32(scfg.padding);
    response->stream_config_resp.width           = cpu_to_le32(scfg.stream_feature.width);
    response->stream_config_resp.height          = cpu_to_le32(scfg.stream_feature.height);
    response->stream_config_resp.format          = cpu_to_le32(scfg.stream_feature.format);
    response->stream_config_resp.virtual_channel = cpu_to_le32(scfg.stream_feature.virtual_channel);
    response->stream_config_resp.data_type       = cpu_to_le32(scfg.stream_feature.data_type);
    response->stream_config_resp.max_size        = cpu_to_le32(scfg.stream_feature.max_size);

    return GB_OP_SUCCESS;
}

/**
 * @brief capture
 *
 * The Capture operation enqueues a capture request to the Camera Module.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_capture(struct gb_operation *operation)
{
    struct gb_camera_capture_request *request;
    uint16_t size = 0;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    /* capture */
    ret = device_camera_capture(camera_dev, le32_to_cpu(request->request_id),
                                request->streams, le32_to_cpu(request->num_frames),
                                &request->settings, size);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief flush
 *
 * The Flush operation removes all Capture requests from the queue and stops
 * frame transmission as soon as possible.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_flush(struct gb_operation *operation)
{
	struct gb_camera_flush_response *response;
	uint32_t request_id = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* flush */
    ret = device_camera_flush(camera_dev, &request_id);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    /* get request ID */
    response->request_id = cpu_to_le32(request_id);

    return GB_OP_SUCCESS;
}

/**
 * @brief Request meta-data
 *
 * Allows the Camera to provide meta-data associated with a frame to the AP over
 * Greybus.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_meta_data(struct gb_operation *operation)
{
    struct gb_camera_meta_data_request *request;
    uint16_t size = 0; 
    
    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    device_camera_meta_data(camera_dev, le32_to_cpu(request->request_id),
                                  le32_to_cpu(request->frame_number), request->stream,
                                  size, &request->data);
    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus Camera Protocol initialize function
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_camera_init(unsigned int cport)
{
    if (!camera_dev) {
        camera_dev = device_open(DEVICE_TYPE_CAMERA_HW, 0);
        if (!camera_dev) {
            return -EIO;
        }
    }
    return 0;
}

/**
 * @brief Greybus Camera Protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_camera_exit(unsigned int cport)
{
	if (camera_dev) {
        device_close(camera_dev);
        camera_dev = NULL;
    }
}

/**
 * @brief Greybus Camera Protocol operation handler
 */
static struct gb_operation_handler gb_camera_handlers[] = {
    GB_HANDLER(GB_CAMERA_TYPE_PROTOCOL_VERSION, gb_camera_protocol_version),
    GB_HANDLER(GB_CAMERA_TYPE_CAPABILITIES, gb_camera_capabilities),
    GB_HANDLER(GB_CAMERA_TYPE_CONFIGURE_STREAMS, gb_camera_configure_streams),
    GB_HANDLER(GB_CAMERA_TYPE_CAPTURE, gb_camera_capture),
    GB_HANDLER(GB_CAMERA_TYPE_FLUSH, gb_camera_flush),
    GB_HANDLER(GB_CAMERA_TYPE_META_DATA, gb_camera_meta_data),
};

static struct gb_driver gb_camera_driver = {
    .init = gb_camera_init,
    .exit = gb_camera_exit,
    .op_handlers = gb_camera_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_camera_handlers),
};

/**
 * @brief Register Greybus Camera Protocol
 *
 * @param cport CPort number
 */
void gb_camera_register(int cport)
{
    gb_register_driver(cport, &gb_camera_driver);
}

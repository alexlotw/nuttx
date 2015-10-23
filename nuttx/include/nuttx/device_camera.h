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

#ifndef __ARCH_ARM_DEVICE_CAMERA_H
#define __ARCH_ARM_DEVICE_CAMERA_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define DEVICE_TYPE_CAMERA_HW    "camera"

// struct camera_info {

//}

/**
 * Camera camera module capabilities
 */
struct camera_module_capabilities {
    /** The camera supports reporting image metadata through Greybus */
    uint32_t    metadata_greybus:1;

    /** The camera supports reporting image metadata through CSI-2 */
    uint32_t    metadata_mipi:1;

    /** The camera supports still image capture */
    uint32_t    still_image:1;

    /** The camera supports JPEG capture */
    uint32_t    jpeg:1;   
};

struct stream_config {
	/** Image width in pixels */
    uint16_t    width;

    /** Image height in pixels */
    uint16_t    height;

    /** Image format */
    uint16_t    format;

    /** Virtual channel number for the stream */
    uint8_t     virtual_channel;

    /** Data type for the stream */
    uint8_t     data_type;

    /** Maximum frame size in bytes */
    uint32_t    max_size;
};

struct camera_streams_cfg {
	/** Number of streams, between 0 and 4 inclusive */
    uint16_t    num_streams;

    /** Must be set to zero */
    uint16_t    padding;

    struct stream_config stream_feature;
};

/**
 * Camera device driver operations
 */
struct device_camera_type_ops {
    /** Camera capabilities */
    int (*capabilities)(struct device *dev, uint32_t *size, uint8_t *capabilities);

    /** Configure Streams */
    int (*configure_streams)(struct device *dev, uint16_t num_streams, struct stream_config *streams);

    /** Capture */
    int (*capture)(struct device *dev, uint32_t request_id, uint8_t streams, uint16_t
         num_frames, const uint8_t *settings, uint16_t size);

    /** Flush */
    int (*flush)(struct device *dev, uint32_t *request_id);

    /** Meta data request*/
    void (*meta_data)(struct device *dev, uint32_t request_id, uint16_t frame_number,
          uint8_t stream, uint16_t size, const uint8_t *meta_data);
};

/**
 * @brief Camera get capabilities wrap function
 *
 * @param dev Pointer to structure of device data
 * @param capabilities Pointer that will be stored Camera Module capabilities.
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_capabilities(struct device *dev, uint16_t *size, uint8_t *capabilities)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->capabilities) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->capabilities(dev, size,
                                                                capabilities);
    }
    return -ENOSYS;
}

/**
 * @brief Camera configure_streams wrap function
 *
 * @param dev Pointer to structure of device data
 * @param num_streams Number of streams, between 0 and 4 inclusive
 * @param stream_config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_configure_streams(struct device *dev, uint16_t num_streams, struct stream_config *streams)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->configure_streams) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->configure_streams(dev, num_streams,
                                                              streams);
    }
    return -ENOSYS;
}

/**
 * @brief Camera capture wrap function
 *
 * @param dev Pointer to structure of device data
 * @param request_id An incrementing integer to uniquely identify the capture request
 * @param streams Bitmask of the streams included in the capture request
 * @param num_frames Number of frame(Must be set to zero)
 * @param settings Number of frames to capture (0 for infinity)
 * @param size Capture request settings
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_capture(struct device *dev, uint32_t request_id, uint8_t streams, uint16_t num_frames, const uint8_t *settings, uint16_t size)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->capture) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->capture(dev, request_id, streams, num_frames, settings, size);
    }
    return -ENOSYS;
}

/**
 * @brief Camera flush wrap function
 *
 * @param dev Pointer to structure of device data
 * @param request_id The last request that will be processed before the module stops transmitting frames
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_flush(struct device *dev, uint32_t *request_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->flush) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->flush(dev, request_id);
    }
    return -ENOSYS;
}

/**
 * @brief Camera meta-data wrap function
 *
 * @param dev Pointer to structure of device data
 * @param request_id The ID of the corresponding capture request
 * @param frame_number CSI-2 frame number
 * @param stream The stream number
 * @param size Size of the meta-data block in bytes
 * @param meta_data Pointer to Meta-data block
 * @return 0 on success, negative errno on error
 */
static inline void device_camera_meta_data(struct device *dev, uint32_t request_id, uint16_t frame_number,
          uint8_t stream, uint16_t size, const uint8_t *meta_data)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->meta_data) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->meta_data(dev, request_id,
                                                             frame_number,
                                                             stream,
                                                             size,
                                                             meta_data);
    }
    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_CAMERA_H */

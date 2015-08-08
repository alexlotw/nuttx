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

#ifndef __ARCH_ARM_DEVICE_SDIO_H
#define __ARCH_ARM_DEVICE_SDIO_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>

#define DEVICE_TYPE_SDIO_HW "sdio"

/**
 * @brief SDIO event callback function
 *
 * @param event Event type.
 * @return None.
 */
typedef void (*sdio_event_callback)(uint8_t event);

/**
 * SDIO capabilities
 */
struct sdio_cap {
    /** Capabilities Bit Masks */
    uint32_t caps;
    /** Voltage Range Bit Mask */
    uint32_t ocr;
    /** Maximum Number of blocks per data command transfer */
    uint16_t max_blk_count;
    /** Maximum size of each block to transfer */
    uint16_t max_blk_size;
};

/**
 * SDIO Ios settings
 */
struct sdio_ios {
    /** clock rate in Hz */
    uint32_t clock;
    /** Voltage Range Bit Mask */
    uint32_t vdd;
    /** Bus Mode */
    uint8_t  bus_mode;
    /** SDIO Power Mode */
    uint8_t  power_mode;
    /** SDIO Bus Width */
    uint8_t  bus_width;
    /** SDIO Timing */
    uint8_t  timing;
    /** SDIO Signal Voltage */
    uint8_t  signal_voltage;
    /** SDIO Driver Type */
    uint8_t  drv_type;
};

/**
 * SDIO command parameters
 */
struct sdio_cmd {
    /** SDIO command operation code, as specified by SD Association */
    uint8_t  cmd;
    /** SDIO Command Flags */
    uint8_t  cmd_flags;
    /** SDIO Command Type */
    uint8_t  cmd_type;
    /** SDIO command arguments, as specified by SD Association */
    uint32_t cmd_arg;
    /** SDIO command response */
    uint32_t *resp;
};

/**
 * SDIO data transfer informaiton
 */
struct sdio_transfer {
    /** SDIO number of blocks of data to transfer */
    uint16_t blocks;
    /** SDIO size of the blocks of data to transfer */
    uint16_t blksz;
    /** SDIO Data pointer */
    uint8_t  *data;
    /** DMA handler */
    void     *dma;
};

/**
 * SDIO device driver operations
 */
struct device_sdio_type_ops {
    /** Get capabilitis of SDIO host controller */
    int (*get_capabilities)(struct device *dev, struct sdio_cap *cap);
    /** Set ios to SDIO host controller */
    int (*set_ios)(struct device *dev, struct sdio_ios *ios);
    /** Send command through SDIO host controller */
    int (*send_cmd)(struct device *dev, struct sdio_cmd *cmd);
    /** Write data to SDIO host controller */
    int (*write)(struct device *dev, struct sdio_transfer *transfer);
    /** Read data from SDIO host controller */
    int (*read)(struct device *dev, struct sdio_transfer *transfer);
    /** Attach callback function with SDIO host controller */
    int (*attach_callback)(struct device *dev, sdio_event_callback callback);
};

/**
 * @brief Get capabilities of SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param cap Pointer to structure of capabilities.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_get_capabilities(struct device *dev,
                                               struct sdio_cap *cap)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->get_capabilities) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->get_capabilities(dev, cap);
}

/**
 * @brief Set ios of SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_set_ios(struct device *dev, struct sdio_ios *ios)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->set_ios) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->set_ios(dev, ios);
}

/**
 * @brief Send SDIO command through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->send_cmd) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->send_cmd(dev, cmd);
}

/**
 * @brief Write data to SD card through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_write(struct device *dev,
                                    struct sdio_transfer *transfer)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->write) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->write(dev, transfer);
}

/**
 * @brief Read data from SD card through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_read(struct device *dev,
                                   struct sdio_transfer *transfer)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->read) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->read(dev, transfer);
}

/**
 * @brief Attach callback function to SDIO host controller driver
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_attach_callback(struct device *dev,
                                              sdio_event_callback callback)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.sdio->attach_callback) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.sdio->attach_callback(dev, callback);
}

#endif /* __ARCH_ARM_DEVICE_SDIO_H */

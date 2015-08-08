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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio.h>

#define SDIO_FLAG_PROBE     BIT(0)
#define SDIO_FLAG_OPEN      BIT(1)

/**
 * @brief SDIO device private information
 */
struct tsb_sdio_info {
    /** device driver handler */
    struct device   *dev;
    /** host controller state */
    uint32_t flags;
    /** event callback function */
    sdio_event_callback callback;
};

static struct device *sdio_dev = NULL;

/**
 * @brief Get capabilities of SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param cap Pointer to structure of capabilities.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_get_capability(struct device *dev, struct sdio_cap *cap)
{
    /* TODO implement the function body */
    return 0;
}

/**
 * @brief Set ios of SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_set_ios(struct device *dev, struct sdio_ios *ios)
{
    /* TODO implement the function body */
    return 0;
}

/**
 * @brief Send SDIO command through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd)
{
    /* TODO implement the function body */
    return 0;
}

/**
 * @brief Write data to SD card through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_write(struct device *dev, struct sdio_transfer *transfer)
{
    /* TODO implement the function body */
    return 0;
}

/**
 * @brief Read data from SD card through SDIO host controller
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_read(struct device *dev, struct sdio_transfer *transfer)
{
    /* TODO implement the function body */
    return 0;
}

/**
 * @brief Attach callback function to SDIO host controller driver
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_attach_callback(struct device *dev,
                                    sdio_event_callback callback)
{
    struct tsb_sdio_info *info;

    if (!dev || !dev->private) {
        return -EINVAL;
    }
    info = dev->private;

    info->callback = callback;
    return 0;
}

/**
* @brief The device open function.
*
* This function is called when protocol preparing to use the driver ops
* in initial stage. It is called after probe() was invoked by the system.
* The function checks whether the driver is already open or not. If it was
* opened, it returns driver busy error number, otherwise it keeps a flag to
* identify the driver was opened and returns success.
*
* @param dev Pointer to the SDIO device structure.
* @return 0 for success, negative errno on error
*/
static int tsb_sdio_dev_open(struct device *dev)
{
    struct tsb_sdio_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (info->flags & SDIO_FLAG_OPEN) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    info->flags = SDIO_FLAG_OPEN;

err_irqrestore:
    irqrestore(flags);

    return ret;
}

/**
* @brief The device close function.
*
* This function is called when protocol no longer using this driver.
* The driver must be opened before calling this function.
*
* @param dev Pointer to the SDIO device structure.
* @return None.
*/
static void tsb_sdio_dev_close(struct device *dev)
{
    struct tsb_sdio_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    info->flags = 0;

    irqrestore(flags);
}

/**
* @brief The device probe function.
*
* This function is called by the system to register this driver when the
* system boots up.This function allocates memory for saving driver internal
* information data, attaches the interrupt handlers to IRQs of the controller
* and configures the pin settings of the UART controller.
*
* @param dev Pointer to the SDIO device structure.
* @return 0 for success, negative errno on error.
*/
static int tsb_sdio_dev_probe(struct device *dev)
{
    struct tsb_sdio_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(info));
    if (!info) {
        return -ENOMEM;
    }

    sdio_dev = dev;
    info->dev = dev;
    info->flags = SDIO_FLAG_PROBE;
    dev->private = info;

    return 0;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It
* must be called after probe() and open(). It detaches IRQ handlers and frees
* the internal information memory space.
*
* @param dev Pointer to the SDIO device structure.
* @return None.
*/
static void tsb_sdio_dev_remove(struct device *dev)
{
    struct tsb_sdio_info *info;

    if (!dev || !dev->private) {
        return;
    }
    info = dev->private;

    if (info->flags & SDIO_FLAG_OPEN) {
        tsb_sdio_dev_close(dev);
    }
    info->flags = 0;

    free(info);
}

static struct device_sdio_type_ops tsb_sdio_type_ops = {
    .get_capabilities   = tsb_sdio_get_capability,
    .set_ios            = tsb_sdio_set_ios,
    .send_cmd           = tsb_sdio_send_cmd,
    .write              = tsb_sdio_write,
    .read               = tsb_sdio_read,
    .attach_callback    = tsb_sdio_attach_callback,
};

static struct device_driver_ops tsb_sdio_driver_ops = {
    .probe         = tsb_sdio_dev_probe,
    .remove        = tsb_sdio_dev_remove,
    .open          = tsb_sdio_dev_open,
    .close         = tsb_sdio_dev_close,
    .type_ops.sdio = &tsb_sdio_type_ops,
};

struct device_driver tsb_sdio_driver = {
    .type = DEVICE_TYPE_SDIO_HW,
    .name = "tsb_sdio",
    .desc = "TSB SDIO Driver",
    .ops  = &tsb_sdio_driver_ops,
};

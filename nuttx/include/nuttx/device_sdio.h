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

#include <errno.h>
#include <nuttx/util.h>
#include <nuttx/device.h>
#include <stdbool.h>

#define DEVICE_TYPE_SDIO_HW "sdio"

/** non-blocking transfer */
#define SDIO_FLAG_ASYNC_TRANSFER    BIT(0)
/** DMA transfer */
#define SDIO_FLAG_DMA_TRNSFER       BIT(1)

/* SDIO status bits */
/** SDIO card present */
#define SDIO_STATUS_PRESENT         BIT(0)
/** SDIO card remove */
#define SDIO_STATUS_REMOVED         BIT(1)
/** DIO card write protected */
#define SDIO_STATUS_WRPROTECTED     BIT(2)

/* SDIO Capabilities bit masks */
#define SDIO_CAP_NONREMOVABLE    0x00000001  /* Device is unremovable from
                                                the slot */
#define SDIO_CAP_4_BIT_DATA      0x00000002  /* Host support 4 bit transfers
                                              */
#define SDIO_CAP_8_BIT_DATA      0x00000004  /* Host support 8 bit transfers
                                              */
#define SDIO_CAP_MMC_HS          0x00000008  /* Host support mmc high-speed
                                                timings */
#define SDIO_CAP_SD_HS           0x00000010  /* Host support SD high-speed
                                                timings */
#define SDIO_CAP_ERASE           0x00000020  /* Host allow erase and trim
                                                commands */
#define SDIO_CAP_1_2V_DDR        0x00000040  /* Host support DDR mode at 1.2V
                                              */
#define SDIO_CAP_1_8V_DDR        0x00000080  /* Host support DDR mode at 1.8V
                                              */
#define SDIO_CAP_POWER_OFF_CARD  0x00000100  /* Host can power off card */
#define SDIO_CAP_UHS_SDR12       0x00000200  /* Host support UHS SDR12 mode
                                              */
#define SDIO_CAP_UHS_SDR25       0x00000400  /* Host support UHS SDR25 mode
                                              */
#define SDIO_CAP_UHS_SDR50       0x00000800  /* Host support UHS SDR50 mode
                                              */
#define SDIO_CAP_UHS_SDR104      0x00001000  /* Host support UHS SDR104 mode
                                              */
#define SDIO_CAP_UHS_DDR50       0x00002000  /* Host support UHS DDR50 mode
                                              */
#define SDIO_CAP_DRIVER_TYPE_A   0x00004000  /* Host support Driver Type A */
#define SDIO_CAP_DRIVER_TYPE_C   0x00008000  /* Host support Driver Type C */
#define SDIO_CAP_DRIVER_TYPE_D   0x00010000  /* Host support Driver Type D */
#define SDIO_CAP_HS200_1_2V      0x00020000  /* Host support HS200 mode at
                                                1.2V */
#define SDIO_CAP_HS200_1_8V      0x00040000  /* Host support HS200 mode at
                                                1.8V */
#define SDIO_CAP_HS400_1_2V      0x00080000  /* Host support HS400 mode at
                                                1.2V */
#define SDIO_CAP_HS400_1_8V      0x00100000  /* Host support HS400 mode at
                                                1.8V */

/* Voltage Range Bit Masks */
#define SDIO_VDD_165_195 0x00000001  /* VDD voltage 1.65 - 1.95 */
#define SDIO_VDD_20_21   0x00000002  /* VDD voltage 2.0 - 2.1 */
#define SDIO_VDD_21_22   0x00000004  /* VDD voltage 2.1 - 2.2 */
#define SDIO_VDD_22_23   0x00000008  /* VDD voltage 2.2 - 2.3 */
#define SDIO_VDD_23_24   0x00000010  /* VDD voltage 2.3 - 2.4 */
#define SDIO_VDD_24_25   0x00000020  /* VDD voltage 2.4 - 2.5 */
#define SDIO_VDD_25_26   0x00000040  /* VDD voltage 2.5 - 2.6 */
#define SDIO_VDD_26_27   0x00000080  /* VDD voltage 2.6 - 2.7 */
#define SDIO_VDD_27_28   0x00000100  /* VDD voltage 2.7 - 2.8 */
#define SDIO_VDD_28_29   0x00000200  /* VDD voltage 2.8 - 2.9 */
#define SDIO_VDD_29_30   0x00000400  /* VDD voltage 2.9 - 3.0 */
#define SDIO_VDD_30_31   0x00000800  /* VDD voltage 3.0 - 3.1 */
#define SDIO_VDD_31_32   0x00001000  /* VDD voltage 3.1 - 3.2 */
#define SDIO_VDD_32_33   0x00002000  /* VDD voltage 3.2 - 3.3 */
#define SDIO_VDD_33_34   0x00004000  /* VDD voltage 3.3 - 3.4 */
#define SDIO_VDD_34_35   0x00008000  /* VDD voltage 3.4 - 3.5 */
#define SDIO_VDD_35_36   0x00010000  /* VDD voltage 3.5 - 3.6 */

/* Bus Mode */
#define SDIO_BUSMODE_OPENDRAIN   0x00    /* SDIO open drain bus mode */
#define SDIO_BUSMODE_PUSHPULL    0x01    /* SDIO push-pull bus mode */

/* Power Mode */
#define SDIO_POWER_OFF   0x00     /* SDIO power off */
#define SDIO_POWER_UP    0x01     /* SDIO power up */
#define SDIO_POWER_ON    0x02     /* SDIO power on */
#define SDIO_POWER_UNDEFINED 0x03 /* SDIO power undefined */

/* Bus Width */
#define SDIO_BUS_WIDTH_1 0x00    /* data bus width 1 bit mode */
#define SDIO_BUS_WIDTH_4 0x02    /* data bus width 4 bit mode */
#define SDIO_BUS_WIDTH_8 0x03    /* data bus width 8 bit mode */

/* Timing */
#define SDIO_TIMING_LEGACY       0x00    /* default speed */
#define SDIO_TIMING_MMC_HS       0x01    /* MMC high speed */
#define SDIO_TIMING_SD_HS        0x02    /* SD high speed*/
#define SDIO_TIMING_UHS_SDR12    0x03    /* UHS SDR12 */
#define SDIO_TIMING_UHS_SDR25    0x04    /* UHS SDR25 */
#define SDIO_TIMING_UHS_SDR50    0x05    /* UHS SDR50 */
#define SDIO_TIMING_UHS_SDR104   0x06    /* UHS SDR104 */
#define SDIO_TIMING_UHS_DDR50    0x07    /* UHS DDR50 */
#define SDIO_TIMING_MMC_DDR52    0x08    /* MMC DDR52*/
#define SDIO_TIMING_MMC_HS200    0x09    /* MMC HS200*/
#define SDIO_TIMING_MMC_HS400    0x0A    /* MMC HS400 */

/* Signal Voltage */
#define SDIO_SIGNAL_VOLTAGE_330  0x00    /* Signal Voltage = 3.30V */
#define SDIO_SIGNAL_VOLTAGE_180  0x01    /* Signal Voltage = 1.80V */
#define SDIO_SIGNAL_VOLTAGE_120  0x02    /* Signal Voltage = 1.20V */

/* Driver Type */
#define SDIO_SET_DRIVER_TYPE_B   0x00    /* Driver Type B */
#define SDIO_SET_DRIVER_TYPE_A   0x01    /* Driver Type A */
#define SDIO_SET_DRIVER_TYPE_C   0x02    /* Driver Type C */
#define SDIO_SET_DRIVER_TYPE_D   0x03    /* Driver Type D */

/* Command Flags */
#define SDIO_RSP_NONE        0x00    /* No Response is expected by the
                                        command */
#define SDIO_RSP_PRESENT     0x01    /* Response is expected by the command
                                      */
#define SDIO_RSP_136         0x02    /* Long response is expected by the
                                        command */
#define SDIO_RSP_CRC         0x04    /* A valid CRC is expected by the
                                        command */
#define SDIO_RSP_BUSY        0x08    /* Card may send a busy response */
#define SDIO_RSP_OPCODE      0x10    /* Response contains opcode */

/* Command Type */
#define SDIO_CMD_AC      0x00    /* Addressed Command */
#define SDIO_CMD_ADTC    0x01    /* Addressed Data Transfer Command */
#define SDIO_CMD_BCR     0x02    /* Broadcasted Command, no response */
#define SDIO_CMD_BC      0x03    /* Broadcasted Command with response */

enum sdio_clock {
    /** Clock is disabled*/
    CLOCK_SDIO_DISABLED = 0,
    /** Initial ID mode clocking */
    CLOCK_IDMODE,
    /** MMC normal operation clocking */
    CLOCK_MMC_TRANSFER,
    /** SD normal operation clocking (narrow 1-bit mode) */
    CLOCK_TRANSFER_1BIT,
    /** SD normal operation clocking (wide 4-bit mode) */
    CLOCK_TRANSFER_4BIT,
    /** SD normal operation clocking (wide 8-bit mode) */
    CLOCK_TRANSFER_8BIT
};

struct device_sdio_type_ops {
    /* Mutual exclusion */
    /** Slot lock for multi-slot */
    int (*lock)(struct device *dev, bool lock);

    /* Initialization/setup */
    /** reset devices */
    int (*reset)(struct device *dev);
    int (*status)(struct device *dev, uint8_t *sdio_status);
    /** Get SDIO capability regisster value to SDIO host controller */
    int (*get_capabilities)(struct device *dev, uint32_t *caps, uint32_t *ocr,
                            uint16_t *max_blk_count, uint16_t *max_blk_size);
    /** The operation allows the requester to setup parameters listed in
        to SDIO controller */
    int (*set_ios)(struct device *dev, uint32_t clock, uint32_t vdd,
                   uint8_t bus_mode, uint8_t power_mode, uint8_t bus_width,
                   uint8_t timing, uint8_t signal_voltage, uint8_t drv_type);
    /** set bus width */
    int (*set_widebus)(struct device *dev, uint8_t bus_width);
    int (*set_clock)(struct device *dev, enum sdio_clock clock_mode);
    /** Attach the SDIO interrupt handler to process SDIO data send/receive
        and configure SDIO data interrupt */
    int (*attach)(struct device *dev);

    /* Command/Status/Data Transfer */
    /** Send the SDIO command */
    int (*sendcmd)(struct device *dev, uint8_t cmd, uint8_t cmd_flags,
                   uint8_t cmd_type, uint32_t cmd_arg, uint32_t *resp);
    /** Setup hardware for initalization of data transfer from the card */
    int (*recvsetup)(struct device *dev, uint8_t *buffer, size_t nbytes);
    /** Setup hardware for initialization of data transfer to the card */
    int (*sendsetup)(struct device *dev, const uint8_t *buffer, size_t nbytes);
    /** Cancel the data transfer setup of recvsetup() sendsetup()
        dmarecvsetup() dmasendsetup() */
    int (*cancel)(struct device *dev);
    /** Wait for response of the SDIO commands to make sure the command is
        processed */
    int (*waitresponse)(struct device *dev, uint32_t cmd);
    /** Receive response of SDIO command. SDIO response: R1 R2 R3 R4 R5 R6 R7 */
    int (*recvR1)(struct device *dev, uint32_t cmd, uint32_t *R1);
    int (*recvR2)(struct device *dev, uint32_t cmd, uint32_t *R2);
    int (*recvR3)(struct device *dev, uint32_t cmd, uint32_t *R3);
    int (*recvR4)(struct device *dev, uint32_t cmd, uint32_t *R4);
    int (*recvR5)(struct device *dev, uint32_t cmd, uint32_t *R5);
    int (*recvR6)(struct device *dev, uint32_t cmd, uint32_t *R6);
    int (*recvR7)(struct device *dev, uint32_t cmd, uint32_t *R7);

    /* Event/Callback support */
    /** Enable/Disable a set of SDIO wait events. */
    int (*waitenable)(struct device *dev, uint8_t event_set);
    /** Wait for one of the enabled events to occur (or a timeout) */
    int (*eventwait)(struct device *dev, uint32_t timeout);

    /* DMA */
    /** Describe whether or not the hardware can support DMA */
    int (*dmasupported)(struct device *dev, uint8_t *dma_status);
    /** Preflight SDIO DMA operation */
    int (*dmapreflight)(struct device *dev, const uint8_t *buffer,
                        size_t buflen);
    /** Setup DMA for read data */
    int (*dmarecvsetup)(struct device *dev, uint8_t *buffer, size_t buflen);
    /** Setup DMA for write data */
    int (*dmasendsetup)(struct device *dev, const uint8_t *buffer,
                        size_t buflen);

    /* Data read/write */
    /** Read the data from the physical SDIO device */
    int (*data_read)(struct device *dev, uint16_t *data_blocks,
                     uint16_t *data_blksz, uint8_t *buffer,
                     uint8_t data_flags, void (*complete)(void *context));
    /** Write the data to the physical SDIO device */
    int (*data_write)(struct device *dev, uint16_t *data_blocks,
                      uint16_t *data_blksz, uint8_t *buffer,
                      uint8_t data_flags, void (*complete)(void *context));
};

/**
 * @brief Lock/unlock the SDIO bus to prevent any access by other transaction.
 *
 * Due to the possible access by multiple devices, it will be necessary to
 * lock SDIO bus to have exclusive bus access for a sequence of commands
 * and data. The lock() is implemented by semaphore. There is no case/action
 * (e.g., reset()) to force the lock to be release if the caller does not
 * release the lock.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param lock - true to lock; false to unlock.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_lock(struct device *dev, bool lock)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->lock) {
        return dev->driver->ops->type_ops.sdio->lock(dev, lock);
    }

    return -ENOSYS;
}

/**
 * @brief Reset the SDIO controller.
 *
 * Configure all setup and initialization of SDIO controller to default state.
 * This function is called if SDIO slot hardware initialization is fail during
 * the system boot up. This function should be called after set_widebus()
 * and set_clock() to restore back to the default 1-bit data bus and disable
 * clock of the card.
 *
 * @param dev - pointer to the SDIO device structure.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_reset(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->reset) {
        return dev->driver->ops->type_ops.sdio->reset(dev);
    }

    return -ENOSYS;
}

/**
 * @brief Get SDIO card presence status and write protected status.
 *
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. card detection
 * 3. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param sdio_status - returns the actual SDIO status.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_status(struct device *dev, uint8_t *sdio_status)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->status) {
        return dev->driver->ops->type_ops.sdio->status(dev, sdio_status);
    }

    return -ENOSYS;
}

/**
 * @brief Get SDIO capabilities bit mask.
 *
 * The Greybus SDIO Get Capabilities operation allows the requester to fetch
 * capabilities that are supported by the Controller.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param caps - return the SDIO capability bit mask.
 * @param ocr - rerurn the SDIO voltage range bit mask.
 * @param max_blk_count - return the SDIO maximum number of blocks per data
 *                        command transfer.
 * @param max_blk_size - return the SDIO maximum number size of each block to
 *                       transfer.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_get_capabilities(struct device *dev,
                                               uint32_t *caps, uint32_t *ocr,
                                               uint16_t *max_blk_count,
                                               uint16_t *max_blk_size)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->get_capabilities) {
        return dev->driver->ops->type_ops.sdio->get_capabilities(dev, caps, ocr,
                                                                 max_blk_count,
                                                                 max_blk_size);
    }

    return -ENOSYS;
}

/**
 * @brief Change bus width of send and receive.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param clock - clock rate in Hz.
 * @param vdd - Greybus SDIO Protocol voltage range bit mask.
 * @param bus_mode - Greybus SDIO Protocol bus mode.
 * @param power_mode - Greybus SDIO Protocol power mode.
 * @param bus_width - Greybus SDIO Protocol bus width.
 * @param timing - Greybus SDIO Protocol timing.
 * @param signal_voltage - Greybus SDIO Protocol signal voltage.
 * @param drv_type - Greybus SDIO Protocol driver type.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_set_ios(struct device *dev, uint32_t clock,
                                      uint32_t vdd, uint8_t bus_mode,
                                      uint8_t power_mode, uint8_t bus_width,
                                      uint8_t timing, uint8_t signal_voltage,
                                      uint8_t drv_type)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->set_ios) {
        return dev->driver->ops->type_ops.sdio->set_ios(dev, clock, vdd,
               bus_mode, power_mode, bus_width, timing, signal_voltage,
               drv_type);
    }

    return -ENOSYS;
}

/**
 * @brief Change bus width of send and receive via command SD_ACMD6.
 *
 * Once the bus width has been set, subsequent calls to change the clock
 * and the power of SDIO is required.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param bus_width - Greybus SDIO Protocol bus width.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_set_widebus(struct device *dev, uint8_t bus_width)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->set_widebus) {
        return dev->driver->ops->type_ops.sdio->set_widebus(dev, bus_width);
    }

    return -ENOSYS;
}

/**
 * @brief Enable/disable SDIO clock.
 *
 * Due to various SDIO bus width, it needs to configure various clock used
 * by the SDIO device driver. This function is called in one of the following
 * conditions:
 * 1. SDIO hardware initialization
 * 2. card detection. Power management will also turn the clock off.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param clock_mode - the clocking mode to be used.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_set_clock(struct device *dev,
                                        enum sdio_clock clock_mode)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->set_clock) {
        return dev->driver->ops->type_ops.sdio->set_clock(dev, clock_mode);
    }

    return -ENOSYS;
}

/**
 * @brief Attach the SDIO interrupt handler to process SDIO data send/receive
 * and configure SDIO data interrupts.
 *
 * This function will be called during SDIO hardware initialization.
 *
 * @param dev - pointer to the SDIO device structure.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_attach(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->attach) {
        return dev->driver->ops->type_ops.sdio->attach(dev);
    }

    return -ENOSYS;
}

/**
 * @brief Send the SDIO command.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to be sent.
 * @param cmd_flags - the SDIO command flags to be send.
 * @param cmd_type - the SDIO command type to be sent.
 * @param cmd_arg - 32-bit argument required with commands.
 * @param resp[4] - return the SDIO command response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_sendcmd(struct device *dev, uint8_t cmd,
                                      uint8_t cmd_flags, uint8_t cmd_type,
                                      uint32_t cmd_arg, uint32_t *resp)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->sendcmd) {
        return dev->driver->ops->type_ops.sdio->sendcmd(dev, cmd, cmd_flags,
                                                        cmd_type, cmd_arg,
                                                        resp);
    }

    return -ENOSYS;
}

/**
 * @brief Setup hardware for initialization of data transfer from the card.
 *
 * The initialization includes:
 * 1. reset the DPSM (Data Path State Machine) configuration
 * 2. save the destination buffer information for using by the interrupt
 * handler
 * 3. set up the SDIO data path
 * 4. enable interrupts
 * This will be called before sending CMD13 (SEND_STATUS),
 * CMD17 (READ_SINGLE_BLOCK), CMD18 (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR).
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the receive buffer.
 * @param nbytes - the number of bytes in the transfer.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvsetup(struct device *dev, uint8_t *buffer,
                                        size_t nbytes)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvsetup) {
        return dev->driver->ops->type_ops.sdio->recvsetup(dev, buffer, nbytes);
    }

    return -ENOSYS;
}

/**
 * @brief Setup hardware for initialization of data transfer to the card.
 *
 * The initialization includes:
 * 1. reset the DPSM (Data Path State Machine) configuration
 * 2. save the source buffer information for using by the interrupt handler
 * 3. set up the SDIO data path
 * 4. enable TX interrupts
 * This will be called after sending CMD24 (WRITE_BLOCK),
 * CMD25 (WRITE_MULTIPLE_BLOCK).
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the send buffer.
 * @param nbytes - the number of bytes in the transfer.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_sendsetup(struct device *dev,
                                        const uint8_t *buffer, size_t nbytes)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->sendsetup) {
        return dev->driver->ops->type_ops.sdio->sendsetup(dev, buffer, nbytes);
    }

    return -ENOSYS;
}

/**
 * @brief Cancel the data transfer setup of recvsetup(), sendsetup(),
 * dmarecvsetup(), dmasendsetup().
 *
 * It disables pending interrupt status and cancels any watchdog timeout.
 * The function will be called if the result of recvRx() is fail within
 * function data_read() and function data_write().
 *
 * @param dev - pointer to the SDIO device structure.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_cancel(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->cancel) {
        return dev->driver->ops->type_ops.sdio->cancel(dev);
    }

    return -ENOSYS;
}

/**
 * @brief Wait for the response of the SDIO commands to make sure the
 * command is processed.
 *
 * This function should be called even after sending commands with no response
 * (such as CMD0) to make sure the hardware is ready to receive the next
 * command. There is a loop in waitresponse() to check whether the response
 * is available within the timeout defined based on various SDIO command.
 * Due to various SDIO command, it needs to configure various timeout used by
 * the loop in waitresponse(). This function is called in one of the following
 * conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 * Function sendcmd() must be called before using waitresponse() to get the
 * response from the SDIO device.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command that was sent.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_waitresponse(struct device *dev, uint32_t cmd)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->waitresponse) {
        return dev->driver->ops->type_ops.sdio->waitresponse(dev, cmd);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R1 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR1(struct device *dev, uint32_t cmd,
                                     uint32_t *R1)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR1) {
        return dev->driver->ops->type_ops.sdio->recvR1(dev, cmd, R1);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R2 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR2(struct device *dev, uint32_t cmd, uint32_t *R2)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR2) {
        return dev->driver->ops->type_ops.sdio->recvR2(dev, cmd, R2);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R3 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR3(struct device *dev, uint32_t cmd, uint32_t *R3)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR3) {
        return dev->driver->ops->type_ops.sdio->recvR3(dev, cmd, R3);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R4 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR4(struct device *dev, uint32_t cmd, uint32_t *R4)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR4) {
        return dev->driver->ops->type_ops.sdio->recvR4(dev, cmd, R4);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R5 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR5(struct device *dev, uint32_t cmd, uint32_t *R5)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR5) {
        return dev->driver->ops->type_ops.sdio->recvR5(dev, cmd, R5);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R6 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR6(struct device *dev, uint32_t cmd, uint32_t *R6)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR6) {
        return dev->driver->ops->type_ops.sdio->recvR6(dev, cmd, R6);
    }

    return -ENOSYS;
}

/**
 * @brief Receive response of SDIO command.
 *
 * The SDIO device driver implementation needs to check:
 * 1. the correct response to this command
 * 2. a timeout or CRC error occurred
 * And then returns the Rx response.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to send.
 * @param R7 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_recvR7(struct device *dev, uint32_t cmd, uint32_t *R7)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->recvR7) {
        return dev->driver->ops->type_ops.sdio->recvR7(dev, cmd, R7);
    }

    return -ENOSYS;
}

/**
 * @brief Enable/disable a set of SDIO wait events.
 *
 * The set of events are configured before calling eventwait().
 * The mechanism of using waitenable() and eventwait() will prevent race
 * condition between different calls.
 * This function includes the following operations:
 * 1. select the interrupt mask that will provide the appropriate wake-up
 * interrupts.
 * 2. enable event-related interrupts
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 *
 * @param dev - pointer to the SDIO device structure.
 * @param eventset - a bit mask of events to enable or disable. 0=disable;
                     1=enable.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_waitenable(struct device *dev, uint8_t event_set)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->waitenable) {
        return dev->driver->ops->type_ops.sdio->waitenable(dev, event_set);
    }

    return -ENOSYS;
}

/**
 * @brief Wait for one of the enabled events to occur (or a timeout).
 *
 * The SDIO device driver implementation needs to add a loop to monitor:
 * 1. the SDIO events
 * 2. the timeout occurred
 * And then disable event-related interrupts when the function is exited.
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 * It needs to call waitenable() before calling eventwait().
 *
 * @param dev - pointer to the SDIO device structure.
 * @param timeout - maximum time in milliseconds to wait.
 *                Zero means immediate timeout without waiting. The timeout
 *                value is ignored if SDIOWAIT_TIMEOUT is not included in the
 *                waited-for eventset.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_eventwait(struct device *dev, uint32_t timeout)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->eventwait) {
        return dev->driver->ops->type_ops.sdio->eventwait(dev, timeout);
    }

    return -ENOSYS;
}

/**
 * @brief Describe whether or not the hardware can support DMA.
 *
 * This function is called during SDIO slot hardware initialization.
 * It returns a value to identify whether DMA hardware is supported.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param dma_status - returns the SDIO DMA supported status. 0: DMA is not
 *                     supported; 1: DMA is supported.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_dmasupported(struct device *dev,
                                           uint8_t *dma_status)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->dmasupported) {
        return dev->driver->ops->type_ops.sdio->dmasupported(dev, dma_status);
    }

    return -ENOSYS;
}

/**
 * @brief Preflight SDIO DMA operation.
 *
 * If the buffer is not well-formed for SDIO DMA transfer
 * (alignment, size, etc.), returns an error.
 * This function is called by data_read() and data_write(). It checks wide
 * bus operation and checks whether or not the DMA controller can transfer
 * data to/from given memory address.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the send/receive buffer.
 * @param nbytes - the size of the DMA transfer in bytes.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_dmapreflight(struct device *dev,
                                           const uint8_t *buffer, size_t buflen)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->dmapreflight) {
        return dev->driver->ops->type_ops.sdio->dmapreflight(dev, buffer,
                                                             buflen);
    }

    return -ENOSYS;
}

/**
 * @brief Setup DMA for read data.
 *
 * This function includes the following operations:
 * 1. reset the DPSM (Data Path State Machine) configuration
 * 2. initialize register sampling
 * 3. save the destination buffer information for using by the interrupt
 * handler
 * 4. set up the SDIO data path
 * 5. configure the RX DMA
 * 6. start the DMA
 * This will be called before sending CMD13 (SEND_STATUS),
 * CMD17 (READ_SINGLE_BLOCK), CMD18 (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR).
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the receive buffer.
 * @param nbytes - the size of the DMA transfer in bytes.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_dmarecvsetup(struct device *dev, uint8_t *buffer,
                                           size_t buflen)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->dmarecvsetup) {
        return dev->driver->ops->type_ops.sdio->dmarecvsetup(dev, buffer,
                                                             buflen);
    }

    return -ENOSYS;
}

/**
 * @brief Setup DMA for write.
 *
 * This function includes the following operations:
 * 1. reset the DPSM (Data Path State Machine) configuration
 * 2. initialize register sampling
 * 3. save the source buffer information for using by the interrupt handler
 * 4. set up the SDIO data path
 * 5. configure the TX DMA
 * 6. start the DMA
 * 7. enable TX interrupts
 * This will be called after sending CMD24 (WRITE_BLOCK),
 * CMD25 (WRITE_MULTIPLE_BLOCK).
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the send buffer.
 * @param nbytes - the size of the DMA transfer in bytes.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_dmasendsetup(struct device *dev,
                                           const uint8_t *buffer, size_t buflen)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->dmasendsetup) {
        return dev->driver->ops->type_ops.sdio->dmasendsetup(dev, buffer,
                                                             buflen);
    }

    return -ENOSYS;
}

/**
 * @brief Read the specified number of sectors from the physical SDIO device.
 *
 * This function supports synchronous and asynchronous read method.
 * For asynchronous read, the caller sets SDIO_FLAG_ASYNC_TRNSFER flag and
 * provides the callback function. This function will start a SDIO read
 * procedure and return immediately without waiting. When SDIO read procedure
 * is completed, this function invokes the callback function to notify the
 * caller that the SDIO read procedure is done. If the caller doesn’t provide
 * the callback function, this function returns -EINVAL error code to notify
 * the problem.
 * For synchronous read, if the caller does not set SDIO_FLAG_ASYNC_TRNSFER
 * flag, it will be blocked on this function until SDIO read procedure is
 * completed or timeout.
 * This function supports DMA transfer. This function will use the DMA mode
 * if the caller sets SDIO_FLAG_DMA_TRNSFER flag.
 * This function can support asynchronous and DMA transfer at the same time.
 * Invalid value being set in the flags field will be ignored.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param data_blocks - return the SDIO numbers of blocks of data to transfer.
 * @param data_blksz - return the SDIO size of the blocks of data to transfer.
 * @param buffer - address of the receive buffer.
 * @param data_flags - SDIO transfer mode.
 * @param complete - a callback function called when SDIO read procedure is
 *                   finished. Only for asynchronous read.
 * @param context - the argument to complete() function when it's called.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_data_read(struct device *dev,
                                        uint16_t *data_blocks,
                                        uint16_t *data_blksz,
                                        uint8_t *buffer,
                                        uint8_t data_flags,
                                        void (*complete)(void *context))
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->data_read) {
        return dev->driver->ops->type_ops.sdio->data_read(dev, data_blocks,
                                                          data_blksz, buffer,
                                                          data_flags, complete);
    }

    return -ENOSYS;
}

/**
 * @brief Write the specified number of sectors to the physical SDIO device.
 *
 * This function supports synchronous and asynchronous write method.
 * For asynchronous write, the caller sets SDIO_FLAG_ASYNC_TRNSFER flag and
 * provides the callback function. This function will start a SDIO write
 * procedure (See Remark of this section for details) and return immediately
 * without waiting. When SDIO write procedure is completed, this function
 * will invoke the callback function to notify that the caller SDIO write
 * procedure is done. If the caller doesn’t provide the callback function,
 * this function returns -EINVAL error code to notify the problem.
 * For synchronous write, if the caller does not set SDIO_FLAG_ASYNC_TRNSFER
 * flag, it will be blocked on this function until SDIO write procedure is
 * completed or timeout.
 * This function supports DMA transfer. This function will use the DMA
 * mode if the caller sets SDIO_FLAG_DMA_TRNSFER flag.
 * This function can support asynchronous and DMA transfer at the same time.
 * Invalid value being set in the flags field will be ignored.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param buffer - address of the send buffer.
 * @param startsector - start sector.
 * @param nsectors - number of sectors to write.
 * @param data_flags - SDIO transfer mode.
 * @param complete - a callback function called when SDIO write procedure is
 *                   finished. Only for asynchronous write.
 * @param context - the argument to complete() function when it's called.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_data_write(struct device *dev,
                                         uint16_t *data_blocks,
                                         uint16_t *data_blksz, uint8_t *buffer,
                                         uint8_t data_flags,
                                         void (*complete)(void *context))
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.sdio);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.sdio->data_write) {
        return dev->driver->ops->type_ops.sdio->data_write(dev, data_blocks,
                                      data_blksz, buffer, data_flags, complete);
    }

    return -ENOSYS;
}
#endif /* __ARCH_ARM_DEVICE_SDIO_H */

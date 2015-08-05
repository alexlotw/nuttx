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
#include <apps/greybus-utils/utils.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio.h>

#define TSB_SDIO_DRIVER_NAME "tsb sdio driver"
#define TSB_SDIO_FLAG_OPEN BIT(0)

/**
 * @brief private SDIO device information
 */
struct tsb_sdio_info {
    /** Driver model representation of the device */
    struct device *dev;
    /** SDIO device state flasgs */
    uint32_t flags;
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
static int tsb_sdio_op_lock(struct device *dev, bool lock)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_reset(struct device *dev)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_status(struct device *dev, uint8_t *sdio_status)
{
    /* TODO Implement body of the function */
    *sdio_status = 0;
    return 0;
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
static int tsb_sdio_op_get_capabilities(struct device *dev, uint32_t *caps,
                                        uint32_t *ocr, uint16_t *max_blk_count,
                                        uint16_t *max_blk_size)
{
    /* TODO Implement body of the function */
    *caps = 0x00080000;
    *ocr = 0x00000800;
    *max_blk_count = 6;
    *max_blk_size = 512;
    return 0;
}

/**
 * @brief  Setup parameters listed in to SDIO controller.
 *
 * Set Ios operation allows the requester to setup parameters listed in to
 * SDIO controller.
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
static int tsb_sdio_op_set_ios(struct device *dev, uint32_t clock,
                               uint32_t vdd, uint8_t bus_mode,
                               uint8_t power_mode, uint8_t bus_width,
                               uint8_t timing, uint8_t signal_voltage,
                               uint8_t drv_type)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_set_widebus(struct device *dev, uint8_t bus_width)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_set_clock(struct device *dev,
                                 enum sdio_clock clock_mode)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_attach(struct device *dev)
{
    /* TODO Implement body of the function */
    return 0;
}

/**
 * @brief Send the SDIO command.
 *
 * This function is called in one of the following conditions:
 * 1. SDIO hardware initialization
 * 2. data_read() and data_write()
 * After calling sendcmd(), call waitresponse() to poll-wait until either
 * the response is available or timeout. There is a loop in waitresponse()
 * to check whether the response is available within the timeout defined
 * based on various SDIO command. Due to various SDIO command, it needs
 * to configure various timeout used by the loop in waitresponse().
 *
 * @param dev - pointer to the SDIO device structure.
 * @param cmd - the command to be sent.
 * @param cmd_flags - the SDIO command flags to be sent.
 * @param cmd_type - the SDIO command type to be sent.
 * @param cmd_arg - 32-bit argument required with commands.
 * @param resp[4] - return the SDIO command response.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_op_sendcmd(struct device *dev, uint8_t cmd,
                               uint8_t cmd_flags, uint8_t cmd_type,
                               uint32_t cmd_arg, uint32_t *resp)
{
    /* TODO Implement body of the function */
    resp[0] = 0x15;
    resp[1] = 0x0;
    resp[2] = 0x0;
    resp[3] = 0x0;
    return 0;
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
static int tsb_sdio_op_recvsetup(struct device *dev, uint8_t *buffer,
                                 size_t nbytes)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_sendsetup(struct device *dev, const uint8_t *buffer,
                                 size_t nbytes)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_cancel(struct device *dev)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_waitresponse(struct device *dev, uint32_t cmd)
{
    /* TODO Implement body of the function */
    return 0;
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
 * @param R1 - buffer to receive the response.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_op_recvR1(struct device *dev, uint32_t cmd, uint32_t *R1)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR2(struct device *dev, uint32_t cmd, uint32_t *R2)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR3(struct device *dev, uint32_t cmd, uint32_t *R3)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR4(struct device *dev, uint32_t cmd, uint32_t *R4)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR5(struct device *dev, uint32_t cmd, uint32_t *R5)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR6(struct device *dev, uint32_t cmd, uint32_t *R6)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_recvR7(struct device *dev, uint32_t cmd, uint32_t *R7)
{
    /* TODO Implement body of the function */
    return 0;
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
 * @param event_set - a bit mask of events to enable or disable. 0=disable;
                   1=enable.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_op_waitenable(struct device *dev, uint8_t event_set)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_eventwait(struct device *dev, uint32_t timeout)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_dmasupported(struct device *dev, uint8_t *dma_status)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_dmapreflight(struct device *dev, const uint8_t *buffer,
                                    size_t buflen)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_dmarecvsetup(struct device *dev, uint8_t *buffer,
                                    size_t buflen)
{
    /* TODO Implement body of the function */
    return 0;
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
static int tsb_sdio_op_dmasendsetup(struct device *dev, const uint8_t *buffer,
                                    size_t buflen)
{
    /* TODO Implement body of the function */
    return 0;
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
 *                 finished. Only for asynchronous read.
 * @param context - the argument to complete() function when it's called.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_op_data_read(struct device *dev, uint16_t *data_blocks,
                                 uint16_t *data_blksz, uint8_t *buffer,
                                 uint8_t data_flags,
                                 void (*complete)(void *context))
{
    /* TODO Implement body of the function */
    *data_blocks = 3;
    *data_blksz = 1024;
    *buffer = 58;
    return 0;
}

/**
 * @brief Write the specified number of sectors to the physical SDIO device.
 *
 * This function supports synchronous and asynchronous write method.
 * For asynchronous write, the caller sets SDIO_FLAG_ASYNC_TRNSFER flag and
 * provides the callback function. This function will start a SDIO write
 * procedure and return immediately without waiting. When SDIO write procedure
 * is completed, this function will invoke the callback function to notify
 * that the caller SDIO write procedure is done. If the caller doesn’t provide
 * the callback function, this function returns -EINVAL error code to notify
 * the problem.
 * For synchronous write, if the caller does not set SDIO_FLAG_ASYNC_TRNSFER
 * flag, it will be blocked on this function until SDIO write procedure is
 * completed or timeout.
 * This function supports DMA transfer. This function will use the DMA mode
 * if the caller sets SDIO_FLAG_DMA_TRNSFER flag.
 * This function can support asynchronous and DMA transfer at the same time.
 * Invalid value being set in the flags field will be ignored.
 *
 * @param dev - pointer to the SDIO device structure.
 * @param data_blocks - return the SDIO numbers of blocks of data to transfer.
 * @param data_blksz - return the SDIO size of the blocks of data to transfer.
 * @param buffer - address of the send buffer.
 * @param data_flags - SDIO transfer mode.
 * @param complete - a callback function called when SDIO read procedure is
 *                 finished. Only for asynchronous write.
 * @param context - the argument to complete() function when it's called.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_op_data_write(struct device *dev, uint16_t *data_blocks,
                                  uint16_t *data_blksz, uint8_t *buffer,
                                  uint8_t data_flags,
                                  void (*complete)(void *context))
{
    /* TODO Implement body of the function */
    *data_blocks = 4;
    *data_blksz = 256;
    return 0;
}

/**
 * @brief Check whether or not the SDIO device is opened.
 *
 * @param info - pointer to the SDIO device information structure.
 * @return Status of the SDIO device.
 */
static bool tsb_sdio_device_is_open(struct tsb_sdio_info *info)
{
    return ((info->flags & TSB_SDIO_FLAG_OPEN) ? true : false);
}

/**
 * @brief Probe SDIO device
 *
 * This function is called by the system to register driver when the system
 * boot up. This function includes the following operations:
 * 1. allocate memory to save driver internal information data
 * 2. attach the SDIO interrupt handler
 * 3. clear static interrupt flags
 * 4. enable SDIO interrupts
 * 5. configure the pin settings and the controller
 *
 * @param dev - pointer to the SDIO device structure.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_dev_probe(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;

    /* check input parameter */
    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    dev->private = info;

    /* TODO Implement the following operations in the function */
    /* 2. attach the SDIO interrupt handler */
    /* 3. clear static interrupt flags */
    /* 4. enable SDIO interrupts */
    /* 5. configure the pin settings and the controller */

    return 0;
}

/**
 * @brief Remove SDIO device
 *
 * This function is called by the system to unregister the driver.
 * It detaches IRQ handlers and frees the internal information memory space.
 *
 * @param dev - pointer to the SDIO device structure.
 */
static void tsb_sdio_dev_remove(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;

    /* check input parameter */
    if (!dev || !dev->private) {
        return;
    }
    info = dev->private;

    dev->private = NULL;
    free(info);

    /* TODO detaches IRQ handlers here */
}

/**
 * @brief Open SDIO device
 *
 * This function is called when the Greybus protocol prepares to use the
 * driver ops in initial stage. It checks whether or not the driver is
 * already opened. If it was opened, it returns with error number.
 * It will keep a flag to identify the driver is opened.
 * Greybus protocol needs to call probe() before use this function.
 *
 * @param dev - pointer to the SDIO device structure.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_dev_open(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;

    /* check input parameter */
    if (!dev || !dev->private) {
        return -EINVAL;
    }
    info = dev->private;

    if (tsb_sdio_device_is_open(info)) {
        return -EBUSY;
    }

    info->flags = TSB_SDIO_FLAG_OPEN;

    return 0;
}

/**
 * @brief Close SDIO device
 *
 * This function is called when protocol releases this driver. It clears
 * the flag for the driver state.
 *
 * @param dev - pointer to the SDIO device structure.
 */
static void tsb_sdio_dev_close(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;

    /* check input parameter */
    if (!dev || !dev->private) {
        return;
    }
    info = dev->private;

    if (!tsb_sdio_device_is_open(info)) {
        return;
    }

    info->flags = 0;
}

static struct device_sdio_type_ops tsb_sdio_type_ops = {
    .lock             = tsb_sdio_op_lock,
    .reset            = tsb_sdio_op_reset,
    .status           = tsb_sdio_op_status,
    .get_capabilities = tsb_sdio_op_get_capabilities,
    .set_ios          = tsb_sdio_op_set_ios,
    .set_widebus      = tsb_sdio_op_set_widebus,
    .set_clock        = tsb_sdio_op_set_clock,
    .attach           = tsb_sdio_op_attach,
    .sendcmd          = tsb_sdio_op_sendcmd,
    .recvsetup        = tsb_sdio_op_recvsetup,
    .sendsetup        = tsb_sdio_op_sendsetup,
    .cancel           = tsb_sdio_op_cancel,
    .waitresponse     = tsb_sdio_op_waitresponse,
    .recvR1           = tsb_sdio_op_recvR1,
    .recvR2           = tsb_sdio_op_recvR2,
    .recvR3           = tsb_sdio_op_recvR3,
    .recvR4           = tsb_sdio_op_recvR4,
    .recvR5           = tsb_sdio_op_recvR5,
    .recvR6           = tsb_sdio_op_recvR6,
    .recvR7           = tsb_sdio_op_recvR7,
    .waitenable       = tsb_sdio_op_waitenable,
    .eventwait        = tsb_sdio_op_eventwait,
    .dmasupported     = tsb_sdio_op_dmasupported,
    .dmapreflight     = tsb_sdio_op_dmapreflight,
    .dmarecvsetup     = tsb_sdio_op_dmarecvsetup,
    .dmasendsetup     = tsb_sdio_op_dmasendsetup,
    .data_read        = tsb_sdio_op_data_read,
    .data_write       = tsb_sdio_op_data_write,
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

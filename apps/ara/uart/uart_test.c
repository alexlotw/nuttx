/*
 * Copyright (c) 2015 Google Inc.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>

/**
 * @brief The modem status change callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ms The present UART modem status.
 * @return None.
 */
static void ms_callback(uint8_t ms)
{
    fprintf(stderr, "ms_callback: (%u)\n", ms);
}

/**
 * @brief The line status change callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void ls_callback(uint8_t ls)
{
    fprintf(stderr, "ms_callback: (%u)\n", ls);
}

/**
 * @brief The receive callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void rx_callback(uint8_t *buffer, int length, int error)
{
    buffer[length] = 0x00; /* string end */
    fprintf(stderr, "rx_callback:\n");
    fprintf(stderr, " data = %s, length = %d, error = %d\n",
            buffer, length, error);
}

/**
 * @brief The transmit callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void tx_callback(uint8_t *buffer, int length, int error)
{
    fprintf(stderr, "tx_callback:\n");
    fprintf(stderr, " data = %s, length = %d, error = %d\n",
            buffer, length, error);
}

/**
 * @brief The set modem control test function
 *
 * Calls the set modem control function, the driver should trun on the debug
 * message to show the register value for verifying.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_set_modem_ctrl(struct device *dev)
{
    int ret;
    uint8_t modem_ctrl;

    modem_ctrl = (MCR_RTS | MCR_LPBK);
    ret = device_uart_set_modem_ctrl(dev, &modem_ctrl);
    if (!ret) {
        fprintf(stderr, "device_uart_set_modem_ctrl failed: %d \n", ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The get modem control test function
 *
 * Calls the get modem control function to get modem control value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_modem_ctrl(struct device *dev)
{
    int ret;
    uint8_t modem_ctrl = 0;

    ret = device_uart_get_modem_ctrl(dev, &modem_ctrl);
    if (!ret) {
        fprintf(stderr, "device_uart_get_modem_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "modem control = %u\n", modem_ctrl);
    return 0;
}

/**
 * @brief The get modem status test function
 *
 * Calls the get modem status function to get modem status value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_modem_status(struct device *dev)
{
    int ret;
    uint8_t modem_status = 0;

    ret = device_uart_get_modem_status(dev, &modem_status);
    if (!ret) {
        fprintf(stderr, "device_uart_get_modem_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "modem status = %u\n", modem_status);
    return 0;
}

/**
 * @brief The get line status test function
 *
 * Calls the get line status function to get line status value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_line_status(struct device *dev)
{
    int ret;
    uint8_t line_status = 0;

    ret = device_uart_get_line_status(dev, &line_status);
    if (!ret) {
        fprintf(stderr, "do_test_get_line_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "line status = %u\n", line_status);
    return 0;
}

/**
 * @brief The set break test function
 *
 * Calls the set break function to set break state.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the setting value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_set_break(struct device *dev)
{
    int ret;

    ret = device_uart_set_break(dev, 1);
    if (!ret) {
        fprintf(stderr, "device_uart_set_break 1 failed: %d\n", ret);
        return ret;
    }

    ret = device_uart_set_break(dev, 0);
    if (!ret) {
        fprintf(stderr, "device_uart_set_break 0 failed: %d\n", ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The attach modem status change callback test function
 *
 * First set the callback NULL, user should observe the driver turn off the
 * modem status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if modem status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_attach_ms_callback(struct device *dev)
{
    int ret;

    ret = device_uart_attach_ms_callback(dev, NULL);
    if (!ret) {
        fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
               ret);
        return ret;
    }

    ret = device_uart_attach_ms_callback(dev, ms_callback);
    if (!ret) {
        fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
               ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The attach line status change callback test function
 *
 * First set the callback NULL, user should observe the driver turn off the
 * line status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if line status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_attach_ls_callback(struct device *dev)
{
    int ret;

    ret = device_uart_attach_ls_callback(dev, NULL);
    if (!ret) {
        fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
               ret);
        return ret;
    }

    ret = device_uart_attach_ls_callback(dev, ls_callback);
    if (!ret) {
        fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
               ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The data transmit test.
 *
 * Send data out in block mode. Check the serial port terminal for the data.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_transmit(struct device *dev)
{
    int ret, xmit = 0;
    uint8_t tx_buf[] = "transmit test";

    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        &xmit, NULL);
    if (!ret) {
        fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
        return ret;
    }

    fprintf(stderr, "device_uart_start_transmitter: sent = %d\n", xmit);

    return 0;
}

/**
 * @brief The data receive test.
 *
 * Receive data from uart port and verify the data.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_receive(struct device *dev)
{
    int ret, recv = 0;
    uint8_t rx_buf[16];

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, &recv, NULL);
    if (!ret) {
        fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        return ret;
    }

    rx_buf[recv] = 0x00; /* string end */
    fprintf(stderr, "device_uart_start_receiver: data = %s, recv = %d",
            rx_buf, recv);

    return 0;
}

/**
 * @brief The data loopback tranfer test.
 *
 * First set the callback NULL, user should observe the driver turn off the
 * line status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if line status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_loopback(struct device *dev)
{
    int ret;
    uint8_t rx_buf[16];
    uint8_t tx_buf[] = "xmit test";

    /*
     * ret = device_uart_set_modem_ctrl(dev, MCR_LPBK);
     * if (!ret) {
     *     fprint(stderr, "device_uart_set_modem_ctrl \n");
     *   return ret;
     * }
     */

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, NULL,
                                     &rx_callback);
    if (!ret) {
        fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        return ret;
    }

    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        NULL, &tx_callback);
    if (!ret) {
        fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Test all
 *
 * To solve un-used functions warnings.
 *
 * @param dev The device driver handler.
 * @return None.
 */
static void do_all_test(struct device *dev)
{
    do_test_set_modem_ctrl(dev);
    do_test_get_modem_ctrl(dev);
    do_test_get_modem_status(dev);
    do_test_get_line_status(dev);
    do_test_set_break(dev);
    do_test_attach_ms_callback(dev);
    do_test_attach_ls_callback(dev);
    do_test_data_transmit(dev);
    do_test_data_receive(dev);
    do_test_data_loopback(dev);
}

/**
 * @brief The main test function.
 *
 * Since there is only one uart port in tsb bridge chip, so we use the jtag for
 * dump debug message without console. Only pick test functions from above
 * do_test_xxx functions.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uart_test_main(int argc, char *argv[])
#endif
{
    struct device *dev;
    int ret;

    fprintf(stderr, "uart_test_main start.\n");

    dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_uart_set_configuration(dev, BAUD_115200, NO_PARITY, 8,
                                        ONE_STOP_BIT, 0);
    if (ret) {
        fprintf(stderr, "device_uart_set_configuration failed: %d\n", ret);
        goto err_device_opened;
    }
    
    ret = do_test_data_loopback(dev);
    if (ret) {
        fprintf(stderr, "uart_test_main failed: %d\n", ret);
    }

err_device_opened:
    device_close(dev);

    return 0;
}

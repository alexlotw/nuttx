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

#include <nuttx/gpio.h>

#include <arch/byteorder.h>

#include "up_arch.h"
#include "tsb_scm.h"

#define DEBUG_ON 1

/* SDIO flags */
#define SDIO_FLAG_OPEN      BIT(0)
#define SDIO_FLAG_WRITE     BIT(1)
#define SDIO_FLAG_READ      BIT(2)

/* System controller registers. The base is SYSCTL_BASE (0x40000000) */
#define UHSSD_DLLCTRL       0x628
#define UHSSD_IO3CTRL       0x64C
#define UHSSD_IO4CTRL       0x650
#define UHSSD_IO5CTRL       0x654
#define UHSSD_IO6CTRL       0x658
#define UHSSD_IO7CTRL       0x65C

/* UHSSD_DLLCTRL bit field details */
#define DLL_ENABLE          BIT(0)
#define SDCMD_PUENABLE      BIT(0) /* UHSSD_IO3CTRL bit field details */
#define SDDATA3_PUENABLE    BIT(0) /* UHSSD_IO4CTRL bit field details */
#define SDDATA2_PUENABLE    BIT(0) /* UHSSD_IO5CTRL bit field details */
#define SDDATA1_PUENABLE    BIT(0) /* UHSSD_IO6CTRL bit field details */
#define SDDATA0_PUENABLE    BIT(0) /* UHSSD_IO7CTRL bit field details */

/*
 * SD card/SDIO registers. The base is 0x40019000
 */
#define SDHC_SDMASYSADDR            0x00
#define SDHC_ARGUMENT2              0x00
#define SDHC_BLOCK_SIZE             0x04
#define SDHC_BLOCK_COUNT            0x06
#define SDHC_ARGUMENT               0x08

#define SDHC_TRANSFER_MODE          0x0C
 #define SDHC_TRNS_BLK_CNT          BIT(1)
 #define SDHC_TRNS_AUTO_CMD12       BIT(2)
 #define SDHC_TRNS_READ             BIT(4)
 #define SDHC_TRNS_MULTI            BIT(5)

#define SDHC_COMMAND                0x0E
 #define SDHC_CMD_RESP_MASK         (BIT(0) | BIT(1))
  #define SDHC_CMD_RESP_NO          0
  #define SDHC_CMD_RESP_136         1
  #define SDHC_CMD_RESP_48          2
  #define SDHC_CMD_RESP_48B         3
 #define SDHC_CMD_CRC               BIT(3)
 #define SDHC_CMD_INDEX             BIT(4)
 #define SDHC_CMD_DATA              BIT(5)

#define SDHC_MAKE_CMD(cmd, flg) (((cmd & 0xff) << 8) | (flg & 0xff))

#define SDHC_RESPONSE0              0x10
#define SDHC_RESPONSE1              0x14
#define SDHC_RESPONSE2              0x18
#define SDHC_RESPONSE3              0x1C
#define SDHC_DATAPORT               0x20

#define SDHC_PRESENTSTATE           0x24
 #define SDHC_CMD_INHIBIT           BIT(0)
 #define SDHC_DATA_INHIBIT          BIT(1)
 #define SDHC_BUF_WRITE_EN          BIT(10)
 #define SDHC_BUF_READ_EN           BIT(11)

#define SDHC_HOST_CTRL              0x28
 #define SDHC_HOST_4BIT             BIT(1)

#define SDHC_POWER_CTRL             0x29
 #define SDHC_PWR_ON                0x01
 #define SDHC_PWR_180               0x0A
 #define SDHC_PWR_300               0x0C
 #define SDHC_PWR_330               0x0E

#define SDHC_BLK_GAP_CTRL           0x2A
#define SDHC_WAKEUP_CTRL            0x2B

#define SDHC_CLOCK_CTRL             0x2C
 #define SDHC_CLK_ENABLE            BIT(0)
 #define SDHC_CLK_STABLE            BIT(1)
 #define SDHC_SD_CLK_EN             BIT(2)
 #define SDHC_DIV_SHIFT             8
 #define SDHC_DIV_MASK              0xFF

#define SDHC_TIMEOUT_CTRL           0x2E
#define SDHC_SOFTWARE_RESET         0x2F

#define SDHC_INT_ERR_STATUS         0x30
#define SDHC_INT_ERR_STATUS_EN      0x34
#define SDHC_INT_ERR_SIGNAL_EN      0x38
 #define SDHC_INT_CMD_CMPLT         BIT(0)
 #define SDHC_INT_TRANS_CMPLT       BIT(1)
 #define SDHC_INT_WRITE_READY       BIT(4)
 #define SDHC_INT_READ_READY        BIT(5)
 #define SDHC_ERR_CMD_TIMEOUT       BIT(16)
 #define SDHC_ERR_CMD_CRC           BIT(17)
 #define SDHC_ERR_CMD_END_BIT       BIT(18)
 #define SDHC_ERR_CMD_INDEX         BIT(19)
 #define SDHC_ERR_DATA_TIMEOUT      BIT(20)
 #define SDHC_ERR_DATA_CRC          BIT(21)
 #define SDHC_ERR_DATA_END_BIT      BIT(22)

#define SDHC_AUTOCMD12_ERR          0x3C

#define SDHC_CAPABILITIES           0x40
 #define SDHC_CAP_TIMEOUT_MASK      0x2F
 #define SDHC_CAP_TO_UINT_MASK      0x80
 #define SDHC_CAP_BASE_CLK_SHIFT    8
 #define SDHC_CAP_BASE_CLK_MASK     0xFF
 #define SDHC_CAP_MAX_BLK_SHIFT     16
 #define SDHC_CAP_MAX_BLK_MASK      0x03
 #define SDHC_CAP_MAX_BLK_512       0
 #define SDHC_CAP_MAX_BLK_1024      1
 #define SDHC_CAP_MAX_BLK_2048      2

#define CMD_INT_MASK  (SDHC_INT_CMD_CMPLT)

#define CMD_ERR_MASK  (SDHC_ERR_CMD_TIMEOUT | SDHC_ERR_CMD_CRC | \
                       SDHC_ERR_CMD_END_BIT | SDHC_ERR_CMD_INDEX)

#define DATA_INT_MASK (SDHC_INT_DATA_END)

#define DATA_ERR_MASK (SDHC_ERR_DATA_TIMEOUT | SDHC_ERR_DATA_CRC | \
                       SDHC_ERR_DATA_END_BIT)

#define DATA_R_INT_MASK (SDHC_INT_READ_READY)
#define DATA_W_INT_MASK (SDHC_INT_WRITE_READY)

/*
 * host ap mmc definition
 */
#define MMC_BUSMODE_OPENDRAIN   1
#define MMC_BUSMODE_PUSHPULL    2

#define MMC_POWER_OFF           0
#define MMC_POWER_UP            1
#define MMC_POWER_ON            2
#define MMC_POWER_UNDEFINED     3

#define MMC_BUS_WIDTH_1         0
#define MMC_BUS_WIDTH_4         2
#define MMC_BUS_WIDTH_8         3

#define MMC_TIMING_LEGACY       0
#define MMC_TIMING_MMC_HS       1
#define MMC_TIMING_SD_HS        2
#define MMC_TIMING_UHS_SDR12    3
#define MMC_TIMING_UHS_SDR25    4
#define MMC_TIMING_UHS_SDR50    5
#define MMC_TIMING_UHS_SDR104   6
#define MMC_TIMING_UHS_DDR50    7
#define MMC_TIMING_MMC_DDR52    8
#define MMC_TIMING_MMC_HS200    9
#define MMC_TIMING_MMC_HS400    10

#define MMC_SIGNAL_VOLTAGE_330  0
#define MMC_SIGNAL_VOLTAGE_180  1
#define MMC_SIGNAL_VOLTAGE_120  2

#define MMC_SET_DRIVER_TYPE_B   0
#define MMC_SET_DRIVER_TYPE_A   1
#define MMC_SET_DRIVER_TYPE_C   2
#define MMC_SET_DRIVER_TYPE_D   3

#define MMC_VDD_165_195     0x00000080  /* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21       0x00000100  /* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22       0x00000200  /* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23       0x00000400  /* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24       0x00000800  /* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25       0x00001000  /* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26       0x00002000  /* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27       0x00004000  /* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28       0x00008000  /* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29       0x00010000  /* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30       0x00020000  /* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31       0x00040000  /* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32       0x00080000  /* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33       0x00100000  /* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34       0x00200000  /* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35       0x00400000  /* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36       0x00800000  /* VDD voltage 3.5 ~ 3.6 */

#define MMC_CAP_4_BIT_DATA      BIT(0)
#define MMC_CAP_MMC_HIGHSPEED   BIT(1)
#define MMC_CAP_SD_HIGHSPEED    BIT(2)
#define MMC_CAP_SDIO_IRQ        BIT(3)
#define MMC_CAP_SPI             BIT(4)
#define MMC_CAP_NEEDS_POLL      BIT(5)
#define MMC_CAP_8_BIT_DATA      BIT(6)
#define MMC_CAP_AGGRESSIVE_PM   BIT(7)
#define MMC_CAP_NONREMOVABLE    BIT(8)
#define MMC_CAP_WAIT_WHILE_BUSY BIT(9)
#define MMC_CAP_ERASE           BIT(10)
#define MMC_CAP_1_8V_DDR        BIT(11)
#define MMC_CAP_1_2V_DDR        BIT(12)
#define MMC_CAP_POWER_OFF_CARD  BIT(13)
#define MMC_CAP_BUS_WIDTH_TEST  BIT(14)
#define MMC_CAP_UHS_SDR12       BIT(15)
#define MMC_CAP_UHS_SDR25       BIT(16)
#define MMC_CAP_UHS_SDR50   (1 << 17)   /* Host supports UHS SDR50 mode */
#define MMC_CAP_UHS_SDR104  (1 << 18)   /* Host supports UHS SDR104 mode */
#define MMC_CAP_UHS_DDR50   (1 << 19)   /* Host supports UHS DDR50 mode */
#define MMC_CAP_RUNTIME_RESUME  (1 << 20)   /* Resume at runtime_resume. */
#define MMC_CAP_DRIVER_TYPE_A   (1 << 23)   /* Host supports Driver Type A */
#define MMC_CAP_DRIVER_TYPE_C   (1 << 24)   /* Host supports Driver Type C */
#define MMC_CAP_DRIVER_TYPE_D   (1 << 25)   /* Host supports Driver Type D */
#define MMC_CAP_CMD23       (1 << 30)   /* CMD23 supported. */
#define MMC_CAP_HW_RESET    (1 << 31)   /* Hardware reset */

/* response type */
#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136 (1 << 1)        /* 136 bit response */
#define MMC_RSP_CRC (1 << 2)        /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3)        /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4)        /* response contains opcode */

/*
 * Command to prepare
 */
#define MMC_SWITCH                  6
#define MMC_SEND_STATUS             13
#define MMC_READ_SINGLE_BLOCK       17
#define MMC_READ_MULTIPLE_BLOCK     18
#define MMC_WRITE_BLOCK             24
#define MMC_WRITE_MULTIPLE_BLOCK    25
#define SD_APP_SEND_SCR             51

/* SDIO GPIO definition */
#define GPB2_SD_POWER_EN 9
#define GPB2_SD_CD       22

/*
 * other definition
 */
#define DELAY_1_MS      1000

#define CARD_NONE       0
#define CARD_INSERTED   1
#define CARD_REMOVED    2

#define DATA_NONE       0
#define DATA_READ       1
#define DATA_WRITE      2

#define SDHC_BASE_BLK   512
#define SDHC_MAX_BLK_COUNT  65535

#define ONE_KHZ         (1000)
#define ONE_MHZ         (1000 * 1000)
/**
 * @brief SDIO device private information
 */
struct tsb_sdio_info {
    /** Device driver handler */
    struct device *dev;
    /** Host controller state */
    uint32_t flags;
    /** SDIO IRQ number */
    int sdhc_irq;
    /** SDIO register base */
    uint32_t reg_base;
    /** command error */
    int cmd_err;
    /** command with data flog */
    uint16_t data_cmd;
    /** data direction */
    uint16_t data_rw;
    /** caller data buffer */
    uint8_t *buffer;
    /** caller data size */
    uint16_t size;
    /** data access error */
    int data_err;
    /** Command semaphore */
    sem_t cmd_sem;
    /** Write semaphore */
    sem_t data_sem;
    /** Data blocking read write */
    sem_t wait_sem;
    /** DMA handler */
    void *dma;
    /** Data thread exit flag */
    bool thread_abort;
    /** Write data thread handle */
    pthread_t data_thread;
    /** Read or write complete callback function*/
    sdio_transfer_callback data_callback;
    /** Event callback function */
    sdio_event_callback evt_callback;
    /** card insert/remove status */
    uint32_t inserted;
    /** sd host controller base clock */
    uint32_t base_clock;
    /** sd host controller timeout clock */
    uint32_t timeout_clock;
    /** current clock */
    uint32_t cur_clock;
    /** current power state */
    uint32_t cur_power;
    /** current bus width */
    uint32_t cur_bus_width;
};

static struct device *sdio_dev = NULL;

static void bit_set(uint32_t reg, uint32_t offset, uint32_t bitmask)
{
    uint32_t reg_val = getreg32(reg + offset);
    reg_val |= bitmask;
    putreg32(reg_val, reg + offset);
}

static void bit_clr(uint32_t reg, uint32_t offset, uint32_t bitmask)
{
    uint32_t reg_val = getreg32(reg + offset);
    reg_val &= ~bitmask;
    putreg32(reg_val, reg + offset);
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void cmd_prepare(struct tsb_sdio_info *info, uint8_t cmd)
{
    switch (cmd) {
    case MMC_SWITCH:
    case MMC_SEND_STATUS:
    case MMC_READ_SINGLE_BLOCK:
    case MMC_READ_MULTIPLE_BLOCK:
    case SD_APP_SEND_SCR:
        info->data_rw = DATA_READ;
        break;
    case MMC_WRITE_BLOCK:
    case MMC_WRITE_MULTIPLE_BLOCK:
        info->data_rw = DATA_WRITE;
        break;
    }
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void convert_rsp_136(uint32_t base, uint32_t *resp)
{
    uint32_t r, i;
    uint8_t b;

    for (i = 0; i < 4; i++) {
        r = getreg32(base + SDHC_RESPONSE0 + (3 - i) * 4);
        b = (i < 3) ? getreg8(base + (2 - i) * 4 + 3) : 0;
        resp[i] = r << 8 | b;
    }
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void set_transfer_mode(struct tsb_sdio_info *info, struct sdio_cmd *cmd)
{
    uint16_t mode = 0;

    if (cmd->cmd == MMC_READ_MULTIPLE_BLOCK ||
        cmd->cmd == MMC_WRITE_MULTIPLE_BLOCK || cmd->data_blocks > 1) {

        mode = SDHC_TRNS_BLK_CNT | SDHC_TRNS_MULTI;

        if (cmd->cmd_flags & BIT(6) && (cmd->cmd != 53)) {
            mode |= SDHC_TRNS_AUTO_CMD12;
        }
        else if (cmd->cmd_flags & BIT(7)) {
            putreg32(cmd->cmd_arg, info->reg_base + SDHC_ARGUMENT2);
        }
    }

    if (info->data_rw == DATA_READ)
        mode |= SDHC_TRNS_READ;

    putreg16(mode, info->reg_base + SDHC_TRANSFER_MODE);
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void sdhc_reset(struct tsb_sdio_info *info, uint8_t reset)
{
    putreg8(reset, info->reg_base + SDHC_SOFTWARE_RESET);

    while (getreg8(info->reg_base + SDHC_SOFTWARE_RESET) & reset) {
        usleep(DELAY_1_MS);
    };
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void sdhc_set_clock(struct tsb_sdio_info *info, uint32_t clock)
{
    uint16_t clk_val = 0;
    uint16_t retry, div;

    putreg16(0, info->reg_base + SDHC_CLOCK_CTRL);

    for (div = 1; div < SDHC_DIV_MASK; div <<= 1) {
        if ((info->base_clock / div) <= clock) {
            info->cur_clock = 0;
            break;
        }
    }

    clk_val |= (div & SDHC_DIV_MASK) << SDHC_DIV_SHIFT;
    clk_val |= SDHC_SD_CLK_EN;
    putreg16(clk_val, info->reg_base + SDHC_CLOCK_CTRL);
    retry = 10;
    while (!(getreg16(info->reg_base + SDHC_CLOCK_CTRL) & SDHC_CLK_STABLE)) {
        usleep(DELAY_1_MS);
        retry--;
        if (!retry) {
            info->cur_clock = 0;
            break;
        }
    }

    clk_val |= SDHC_SD_CLK_EN;
    putreg16(clk_val, info->reg_base + SDHC_CLOCK_CTRL);

    info->cur_clock = clock;
}

/**
 * @brief Enable SD bus power.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Switch the pin share mode and pin high for GPB2_SD_POWER_EN pin
 * 2. Pull up clk and SDIO data interface pins
 * 3. Set SD Bus Voltage Select register with supported maximum voltage
 * 4. Set SD Bus Power register
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdhc_set_power(struct tsb_sdio_info *info,
                           uint8_t mode, uint8_t vdd)
{
    uint8_t pwr = 0;

    if (mode != MMC_POWER_OFF) {
        switch (1 << vdd) {
        case MMC_VDD_165_195:
            pwr = SDHC_PWR_180;
            break;
        case MMC_VDD_29_30:
        case MMC_VDD_30_31:
            pwr = SDHC_PWR_300;
            break;
        case MMC_VDD_32_33:
        case MMC_VDD_33_34:
            pwr = SDHC_PWR_330;
            break;
        default:
            lldbg("unsupport vdd \n");
        }
    }

    if (pwr != 0) {
        pwr |= SDHC_PWR_ON;
    }
    
    putreg8(pwr, info->reg_base + SDHC_POWER_CTRL);
    info->cur_power = mode;
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void sdhc_set_bus_width(struct tsb_sdio_info *info, uint8_t width)
{
    uint8_t ctrl;

    ctrl = getreg8(info->reg_base + SDHC_HOST_CTRL);

    switch (width) {
    case MMC_BUS_WIDTH_4:
        ctrl |= SDHC_HOST_4BIT;
        break;
    case MMC_BUS_WIDTH_1:
        ctrl &= ~SDHC_HOST_4BIT;
        break;
    default:
        lldbg("unsupport bus width");
    }

    putreg8(ctrl, info->reg_base + SDHC_HOST_CTRL);

    info->cur_bus_width = width;
}


/**
 * @brief SDIO read data thread
 *
 * This is the thread for non-blocking data read.
 *
 * @return None.
 */
static void *sdhc_data_thread(void *data)
{
    struct tsb_sdio_info *info = data;
    uint32_t fifo, i;
    uint16_t len;
    uint8_t *datap;

    while (1) {
        sem_wait(&info->data_sem);
        if (info->thread_abort) {
            /* Exit sdhc_data_thread loop */
            break;
        }

        if (info->data_err) {
            if (info->data_callback) {
                info->data_callback(1, info->size, info->buffer, info->data_err);
            } else {
                sem_post(&info->wait_sem);
            }
        }

        datap = info->buffer;
        len = info->size;
        if (info->data_rw == DATA_READ) {
            /* read data from 32 bit fifo to 8 bit buffer */
            while (len) {
                fifo = getreg32(info->reg_base + SDHC_DATAPORT);
                for (i = 0; i < 4; i++, datap++, len--) {
                    *datap = fifo & 0xFF;
                    fifo >>= 8;
                }
            }
        } else if (info->data_rw == DATA_READ) {
            /* write data from 8 bit buffer to 32 bit fifo */
            while (len) {
                for (i = 0; i < 4; i++, datap++, len--) {
                    fifo |= *datap;
                    fifo <<= 8;
                }
                putreg32(fifo, info->reg_base + SDHC_DATAPORT);
            }
        } else {
            /* error */
        }

        if (info->data_callback) {
            info->data_callback(1, info->size, info->buffer, 0);
        }
        else {
            sem_post(&info->wait_sem);
        }
    }

    return NULL;
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void sdhc_init(struct tsb_sdio_info *info)
{
    uint32_t int_err_mask;

    sdhc_reset(info, SDHC_SOFTWARE_RESET );

    int_err_mask = SDHC_INT_CMD_CMPLT | \
                   SDHC_ERR_CMD_TIMEOUT | \
                   SDHC_ERR_CMD_CRC | \
                   SDHC_ERR_CMD_END_BIT | \
                   SDHC_ERR_CMD_INDEX | \
                   SDHC_INT_TRANS_CMPLT | \
                   SDHC_ERR_DATA_TIMEOUT | \
                   SDHC_ERR_DATA_CRC | \
                   SDHC_ERR_DATA_END_BIT | \
                   SDHC_INT_READ_READY | \
                   SDHC_INT_WRITE_READY;

    /* Enable command interrupts */
    putreg32(int_err_mask, info->reg_base + SDHC_INT_ERR_STATUS_EN);
    putreg32(int_err_mask, info->reg_base + SDHC_INT_ERR_SIGNAL_EN);

    info->cur_clock = 0;
    sdhc_set_clock(info, info->cur_clock);

    info->cur_power = 0;
    sdhc_set_power(info, info->cur_power, 18);

    info->cur_bus_width = 0;
    sdhc_set_bus_width(info, info->cur_bus_width);
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void sdhc_deinit(struct tsb_sdio_info *info)
{
    putreg32(0, info->reg_base + SDHC_INT_ERR_STATUS_EN);
    putreg32(0, info->reg_base + SDHC_INT_ERR_SIGNAL_EN);
}


/**
 * @brief SDIO device interrupt routing
 *
 * Set SD host controller configuration for card insert and remove. The
 * configuration includes command interrupts, SD bus power and SD clock. It also
 * inform SDIO protocol driver the card event status.
 *
 * @param irq Number of irq.
 * @param context Pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
int sdhc_card_detect_irq(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint8_t value, card_event;

    gpio_mask_irq(irq);

    value = gpio_get_value(GPB2_SD_CD); // 1: no card, 0: card inserted.

    if (value && info->inserted) {
        // signal card remove
        sdhc_init(info);
        info->inserted = 0;
        card_event = CARD_REMOVED;
    } else if (!value && !info->inserted) {
        sdhc_deinit(info);
        info->inserted = 1;
        card_event = CARD_INSERTED;
    } else {
        card_event = 0;
    }

    if (card_event && info->evt_callback) {
        info->evt_callback(card_event);
    }

    gpio_unmask_irq(irq);

    return 0;
}

/**
 * @brief SDIO interrupt handler.
 *
 * This function is attached as interrupt service when driver init.
 *
 * @param irq The IRQ number from OS.
 * @param context The context for this interrupt.
 * @return None.
 */
static int sdhc_host_irq(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint32_t int_err_status = 0;

    info->data_err = 0;
    info->cmd_err = 0;

    int_err_status = getreg32(info->reg_base + SDHC_INT_ERR_STATUS);

    if (int_err_status & SDHC_INT_CMD_CMPLT) {
        sem_post(&info->cmd_sem);
    }

    if (int_err_status & (SDHC_INT_WRITE_READY | SDHC_INT_READ_READY)) {
        sem_post(&info->data_sem);
    }

    if (int_err_status & SDHC_ERR_CMD_TIMEOUT) {
        lldbg("command err timeout \n");
        info->cmd_err = -1;
    } else if (int_err_status & SDHC_ERR_CMD_CRC) {
        lldbg("command err crc  \n");
        info->cmd_err = -1;
    } else if (int_err_status & SDHC_ERR_CMD_END_BIT) {
        lldbg("command err end bit \n");
        info->cmd_err = -1;
    } else if (int_err_status & SDHC_ERR_CMD_INDEX) {
        lldbg("command err index \n");
        info->cmd_err = -1;
    }

    if (int_err_status & SDHC_ERR_DATA_TIMEOUT) {
        lldbg("data err timeout \n");
        info->data_err = -1;
    } else if (int_err_status & SDHC_ERR_DATA_CRC) {
        lldbg("data err crc  \n");
        info->data_err = -1;
    } else if (int_err_status & SDHC_ERR_DATA_END_BIT) {
        lldbg("cmd err end bit \n");
        info->data_err = -1;
    }

    putreg32(int_err_status, info->reg_base + SDHC_INT_ERR_STATUS);

    return 0;
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void chip_init(void)
{
    uint32_t sysctl;

    /* Enable the 2 clock gating (sdioSysClk/sdioSdClk) */
    tsb_clk_enable(TSB_CLK_SDIOSYS);
    tsb_clk_enable(TSB_CLK_SDIOSD);

    /* Reset the host controller */
    tsb_reset(TSB_RST_SDIOSYS);
    tsb_reset(TSB_RST_SDIOSD);

    /* Assert the DLL enable */
    sysctl = getreg32(SYSCTL_BASE + UHSSD_DLLCTRL);
    sysctl |= DLL_ENABLE;
    putreg32(sysctl, SYSCTL_BASE + UHSSD_DLLCTRL);

    /* Pull up clk and SDIO data interface pins */
    bit_set(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    bit_set(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    bit_set(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    bit_set(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    bit_set(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /* Switch the pin share mode for SD Interfaces */
    tsb_set_pinshare(TSB_PIN_SDIO);

    /* Switch the pin share mode and pin high for GPB2_SD_POWER_EN pin */
    tsb_set_pinshare(TSB_PIN_GPIO9);
    gpio_activate(TSB_PIN_GPIO9);
    gpio_direction_out(GPB2_SD_POWER_EN, 1);

    /* SD CD pin and interrupt handler initialization */
    tsb_set_pinshare(TSB_PIN_GPIO22);
    gpio_activate(GPB2_SD_CD);
    gpio_direction_in(GPB2_SD_CD);
    set_gpio_triggering(GPB2_SD_CD, IRQ_TYPE_EDGE_BOTH);
    gpio_irqattach(GPB2_SD_CD, sdhc_card_detect_irq);
}

/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @return
 */
static void chip_deinit(void)
{
    uint32_t sysctl;

    /* sdcard detect pin configure */
    gpio_mask_irq(GPB2_SD_CD);
    gpio_deactivate(GPB2_SD_CD);

    /* Switch the pin share mode and pin low for GPB2_SD_POWER_EN pin */
    tsb_set_pinshare(TSB_PIN_GPIO9);
    gpio_activate(TSB_PIN_GPIO9);
    gpio_direction_out(GPB2_SD_POWER_EN, 0);

    /* Pull down clk and SDIO data interface pins */
    bit_clr(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    bit_clr(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    bit_clr(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    bit_clr(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    bit_clr(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /* turn off sd host controoler dll */
    sysctl = getreg32(SYSCTL_BASE + UHSSD_DLLCTRL);
    sysctl &= DLL_ENABLE;
    putreg32(sysctl, SYSCTL_BASE + UHSSD_DLLCTRL);

    /* stop sd host controller clock */
    tsb_clk_disable(TSB_CLK_SDIOSYS);
    tsb_clk_disable(TSB_CLK_SDIOSD);
}

/**
 * @brief Retrieve SDIO resource from driver core.
 *
 * This function get the SDIO register base and irq number form driver core
 * infrastructure.
 *
 * @param dev The pointer to device structure.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdhc_extract_resources(struct device *dev,
                                  struct tsb_sdio_info *info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS,
                                    "reg_base");
    if (!r) {
        return -EINVAL;
    }
    info->reg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "irq_sdio");
    if (!r) {
        return -EINVAL;
    }
    info->sdhc_irq = (int)r->start;

    return 0;
}

/**
 * @brief Get capabilities of SD host controller
 *
 * This function is to get capabilities of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @param cap Pointer to structure of capabilities.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_get_capability(struct device *dev, struct sdio_cap *cap)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint32_t cap_reg;
    uint16_t clk_reg;
    //lldbg("tsb_sdio_get_capability() \n");

    cap->caps = 0;

    /* timing */
    cap->caps |= MMC_CAP_MMC_HIGHSPEED |
                 MMC_CAP_SD_HIGHSPEED;

    /* bus width */
    cap->caps |= MMC_CAP_4_BIT_DATA;

    /* driver type */
    cap->caps |= MMC_CAP_DRIVER_TYPE_A |
                 MMC_CAP_DRIVER_TYPE_C |
                 MMC_CAP_DRIVER_TYPE_D;

    /* ocr for vdd */
    cap->ocr = MMC_VDD_32_33 |
               MMC_VDD_33_34;

    /* max block count */
    cap_reg = getreg32(info->reg_base + SDHC_CAPABILITIES);
    cap_reg = (cap_reg >> SDHC_CAP_MAX_BLK_SHIFT) & SDHC_CAP_MAX_BLK_MASK;
    cap->max_blk_size = SDHC_BASE_BLK * (1 << cap_reg);

    cap->max_blk_count = SDHC_MAX_BLK_COUNT;

    /* base clock */
    clk_reg = getreg16(info->reg_base + SDHC_CAPABILITIES);
    info->base_clock = ((clk_reg >> SDHC_CAP_BASE_CLK_SHIFT) &
                        SDHC_CAP_BASE_CLK_MASK);
    info->base_clock *= ONE_MHZ;

    /* timeout clock */
    clk_reg = getreg16(info->reg_base + SDHC_CAPABILITIES);
    info->timeout_clock = (clk_reg & SDHC_CAP_TIMEOUT_MASK) *
                          (clk_reg & SDHC_CAP_TO_UINT_MASK) ? ONE_MHZ : ONE_KHZ;

    return 0;
}

/**
 * @brief Set ios of SD host controller
 *
 * This function is to set the parameters to configure the SD controller.
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_set_ios(struct device *dev, struct sdio_ios *ios)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    if (info->cur_clock != ios->clock) {
        sdhc_set_clock(info, ios->clock);
    }

    if (info->cur_power != ios->power_mode) {
        sdhc_set_power(info, ios->power_mode, ios->vdd);
    }

    if (info->cur_bus_width != ios->bus_width) {
        sdhc_set_bus_width(info, ios->bus_width);
    }

    return 0;
}

/**
 * @brief Send SDIO command through SDIO host controller
 *
 * This function follows the sequence to send command to card through SD host
 * controller.
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */

static int tsb_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    int ret = 0;
    uint32_t mask;
    uint8_t cmd_flags = 0;
    uint8_t retry = 0;

    lldbg("*** CMD(%d), Flags(0x%02x), Type(0x%02x), Arg(0x%08x) *** \n",
          cmd->cmd, cmd->cmd_flags, cmd->cmd_type, cmd->cmd_arg);

    /*
     * prepare data information
     */
    cmd_prepare(info, cmd->cmd);

    /* Wait for CMD & DAT Line unused */
    mask = SDHC_CMD_INHIBIT;
    if (cmd->data_cmd) {
        mask |= SDHC_DATA_INHIBIT;
    }

    retry = 100;
    while (getreg32(info->reg_base + SDHC_PRESENTSTATE) & mask) {
        usleep(DELAY_1_MS);
        retry--;
        if (retry) {
            return -1;
        }
    }

    /*
     * prepare data information
     */
    if (cmd->data_cmd) {
        putreg16(cmd->data_blksz, info->reg_base + SDHC_BLOCK_SIZE);
        putreg16(cmd->data_blocks, info->reg_base + SDHC_BLOCK_COUNT);

        putreg8(0x03, info->reg_base + SDHC_TIMEOUT_CTRL);

        lldbg(">> data: size(%d), blocks(%d) \n", cmd->data_blksz, cmd->data_blocks);
    }

    /* Set Argument 1 Reg register */
    putreg32(cmd->cmd_arg, info->reg_base + SDHC_ARGUMENT);

    /* Set transfer mode */
    set_transfer_mode(info, cmd);

    /* prepare command reg */
    if (!(cmd->cmd_flags & MMC_RSP_PRESENT))
        cmd_flags = SDHC_CMD_RESP_NO;
    else if (cmd->cmd_flags & MMC_RSP_136)
        cmd_flags = SDHC_CMD_RESP_136;
    else if (cmd->cmd_flags & MMC_RSP_BUSY)
        cmd_flags = SDHC_CMD_RESP_48B;
    else
        cmd_flags = SDHC_CMD_RESP_48;

    if (cmd->cmd_flags & MMC_RSP_CRC)
        cmd_flags |= SDHC_CMD_CRC;
    if (cmd->cmd_flags & MMC_RSP_OPCODE)
        cmd_flags |= SDHC_CMD_INDEX;

    if (cmd->data_cmd) {
        cmd_flags |= SDHC_CMD_DATA;
    }

    lldbg("cmd = 0x%04x \n", SDHC_MAKE_CMD(cmd->cmd, cmd_flags));

    putreg16(SDHC_MAKE_CMD(cmd->cmd, cmd_flags), info->reg_base + SDHC_COMMAND);

    //lldbg("SDHCI_BLOCK_SIZE = 0x%04x \n", sdhci_readw(info->reg_base, SDHCI_BLOCK_SIZE));
    //lldbg("SDHCI_BLOCK_COUNT = 0x%04x \n", sdhci_readw(info->reg_base, SDHCI_BLOCK_COUNT));
    //lldbg("SDHCI_ARGUMENT = 0x%04x \n", sdhci_readw(info->reg_base, SDHCI_ARGUMENT));
    //lldbg("SDHCI_TRANSFER_MODE = 0x%04x \n", sdhci_readw(info->reg_base, SDHCI_TRANSFER_MODE));
    //lldbg("SDHCI_COMMAND = 0x%04x \n", sdhci_readw(info->reg_base, SDHCI_COMMAND));
    //lldbg("SDHCI_INT_ENABLE = 0x%08x \n", sdhci_readl(info->reg_base, SDHCI_INT_ENABLE));

    //lldbg("before wait...\n");
    //sdio_dump_registers(info);

    /* Wait for Command Complete Int register */
    sem_wait(&info->cmd_sem);

    if (info->cmd_err) {
        ret = info->cmd_err;
    } else {
        if (cmd_flags & SDHC_CMD_RESP_136) {
            convert_rsp_136(info->reg_base, cmd->resp);
        } else {
            *cmd->resp = getreg32(info->reg_base + SDHC_RESPONSE0);
        }
    }

    return ret;
}

/**
 * @brief Write data to SD card
 *
 * This function is to write data to SD card through the SD host controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_write(struct device *dev, struct sdio_transfer *transfer)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    int ret = 0;
    //lldbg("tsb_sdio_write() \n");

    if (info->data_rw != DATA_WRITE) {
        return -EBUSY;
    }

    if (transfer->callback) {
        info->buffer = transfer->data;
        info->size = transfer->blksz;
        info->data_callback = transfer->callback;
    } else {
        sem_wait(&info->wait_sem);
        ret = info->data_err;
    }

    return ret;
}

/**
 * @brief Read data from SD card
 *
 * This function is to read data from SD card through the SD host controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_read(struct device *dev, struct sdio_transfer *transfer)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint32_t ret = 0;

    lldbg("tsb_sdio_read()+ block = %d, blksz = %d \n", transfer->blocks, transfer->blksz);

    if (info->data_rw != DATA_READ) {
        return -EBUSY;
    }

    if (transfer->callback) { /* non-blocking returns */
        info->buffer = transfer->data;
        info->size = transfer->blksz;
        info->data_callback = transfer->callback;
    } else {
        sem_wait(&info->wait_sem);
        ret = info->data_err;
    }

    return ret;
}

/**
 * @brief Attach callback function to SD host controller driver
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_attach_callback(struct device *dev,
                                    sdio_event_callback callback)
{
    struct tsb_sdio_info *info = NULL;
    uint32_t removed;
    uint8_t card_event;
    //lldbg("tsb_sdio_attach_callback() \n");

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->evt_callback = callback;
    if (info->evt_callback) {
        /* Turn on card detect interrupt */
        gpio_unmask_irq(GPB2_SD_CD);

        /* Get card insert or remove status */
        removed = gpio_get_value(GPB2_SD_CD);

        if (!info->inserted && !removed ) {
            card_event = CARD_INSERTED;
            info->inserted = 1;
        } else if (info->inserted && removed) {
            card_event = CARD_REMOVED;
            info->inserted = 0;
        } else {
            card_event = CARD_NONE;
        }

        if (!card_event) {
            info->evt_callback(card_event);
        }

    } else {
        info->inserted = CARD_NONE;
        /* Turn off card detect interrupt */
        gpio_mask_irq(GPB2_SD_CD);
    }

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
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    int ret = 0;

    //lldbg("tsb_sdio_dev_open() \n");
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    flags = irqsave();

    if (info->flags & SDIO_FLAG_OPEN) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    info->flags |= SDIO_FLAG_OPEN;

    info->thread_abort = false;
    ret = pthread_create(&info->data_thread, NULL, sdhc_data_thread, info);
    if (ret) {
        goto err_irqrestore;
    }

    sdhc_init(info);
    /* turn on sd host controller interrupt */
    up_enable_irq(info->sdhc_irq);

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
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;

    //lldbg("tsb_sdio_dev_close() \n");
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    flags = irqsave();

    if (!(info->flags & SDIO_FLAG_OPEN)) {
        goto err_irqrestore;
    }

    up_disable_irq(info->sdhc_irq);

    info->flags &= ~SDIO_FLAG_OPEN;

    sdhc_deinit(info);

    info->thread_abort = true;
    sem_post(&info->data_sem);
    if (info->data_thread != (pthread_t)0) {
        /* Wait for read_data_thread completed */
        pthread_join(info->data_thread, NULL);
    }

err_irqrestore:
    irqrestore(flags);
}

/**
* @brief The device probe function.
*
* This function is called by the system to register this driver when the
* system boots up. This function allocates memory for saving driver internal
* information data, attaches the interrupt handlers to IRQs of the controller
* and configures the pin settings of the SDIO controller.
*
* @param dev Pointer to the SDIO device structure.
* @return 0 for success, negative errno on error.
*/
static int tsb_sdio_dev_probe(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    int ret = 0;
    //lldbg("tsb_sdio_dev_probe() \n");

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    chip_init();

    ret = sdhc_extract_resources(dev, info);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->cmd_sem, 0, 0);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->data_sem, 0, 0);
    if (ret) {
        goto err_destroy_cmd_sem;
    }

    ret = sem_init(&info->wait_sem, 0, 0);
    if (ret) {
        goto err_destroy_data_sem;
    }

    flags = irqsave();

    ret = irq_attach(info->sdhc_irq, sdhc_host_irq);
    if (ret) {
        goto err_destroy_wait_sem;
    }

    sdio_dev = dev;
    info->dev = dev;
    device_set_private(dev, info);

    irqrestore(flags);

    return 0;

err_destroy_wait_sem:
    irqrestore(flags);
    sem_destroy(&info->wait_sem);
err_destroy_data_sem:
    sem_destroy(&info->data_sem);
err_destroy_cmd_sem:
    sem_destroy(&info->cmd_sem);
err_free_info:
    free(info);

    return ret;
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
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    //lldbg("tsb_sdio_dev_remove() \n");

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & SDIO_FLAG_OPEN) {
        tsb_sdio_dev_close(dev);
    }
    info->flags = 0;

    flags = irqsave();

    irq_detach(info->sdhc_irq);

    sem_destroy(&info->wait_sem);
    sem_destroy(&info->data_sem);
    sem_destroy(&info->cmd_sem);

    irqrestore(flags);

    chip_deinit();

    free(info);
    sdio_dev = NULL;
    device_set_private(dev, NULL);
}

static struct device_sdio_type_ops tsb_sdio_type_ops = {
    .get_capabilities = tsb_sdio_get_capability,
    .set_ios          = tsb_sdio_set_ios,
    .send_cmd         = tsb_sdio_send_cmd,
    .write            = tsb_sdio_write,
    .read             = tsb_sdio_read,
    .attach_callback  = tsb_sdio_attach_callback,
};

static struct device_driver_ops tsb_sdio_driver_ops = {
    .probe    = tsb_sdio_dev_probe,
    .remove   = tsb_sdio_dev_remove,
    .open     = tsb_sdio_dev_open,
    .close    = tsb_sdio_dev_close,
    .type_ops = &tsb_sdio_type_ops,
};

struct device_driver tsb_sdio_driver = {
    .type = DEVICE_TYPE_SDIO_HW,
    .name = "tsb_sdio",
    .desc = "TSB SDIO Driver",
    .ops  = &tsb_sdio_driver_ops,
};

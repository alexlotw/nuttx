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

/* SD card/SDIO registers. The base is 0x40019000 */
#define SDHC_SDMASYSADDR            0x00
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
#define SDHC_POWER_CTRL             0x29
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
 #define SDHC_CAP_BASE_CLK_SHIFT    8
 #define SDHC_CAP_BASE_CLK_MASK     0x3F00;
 
#define CMD_INT_MASK  (SDHC_INT_CMD_CMPLT)

#define CMD_ERR_MASK  (SDHC_ERR_CMD_TIMEOUT | SDHC_ERR_CMD_CRC | \
                       SDHC_ERR_CMD_END_BIT | SDHC_ERR_CMD_INDEX)

#define DATA_INT_MASK (SDHCI_INT_DATA_END)

#define DATA_ERR_MASK (SDHC_ERR_DATA_TIMEOUT | SDHC_ERR_DATA_CRC | \
                       SDHC_ERR_DATA_END_BIT)

#define DATA_R_INT_MASK (SDHC_INT_READ_READY)
#define DATA_W_INT_MASK (SDHC_INT_WRITE_READY)

/* other definition */
#define DELAY_1_MS  1000

/* SDIO buffer structure */
struct sdio_buffer
{
  int16_t head; /* Index to the head [IN] index in the buffer */
  int16_t tail; /* Index to the tail [OUT] index in the buffer */
  uint8_t *buffer; /* Pointer to the allocated buffer memory */
};

/**
 * @brief SDIO device private information
 */
struct tsb_sdio_info {
    /** Device driver handler */
    struct device *dev;
    /** SDIO write buffer structure */
    struct sdio_buffer write_buf;
    /** SDIO read buffer structure */
    struct sdio_buffer read_buf;
    /** Host controller state */
    uint32_t flags;
    /** SDIO register base */
    uint32_t reg_base;
    /** Command response */
    uint32_t resp[4];
    /** SDIO number of blocks of data to transfer */
    uint16_t blocks;
    /** SDIO size of the blocks of data to transfer */
    uint16_t blksz;
    /** Card event type */
    uint8_t card_event;
    /** Previous card event type */
    uint8_t pre_card_event;
    /** Command semaphore */
    sem_t cmd_sem;
    /** Write semaphore */
    sem_t write_sem;
    /** Read semaphore */
    sem_t read_sem;
    /** DMA handler */
    void *dma;
    /** SDIO IRQ number */
    int sdio_irq;
    /** SDIO interrupt error status for the return of command */
    int sdio_int_err_status;
    /** Error status for SD host controller */
    int err_status;
    /** Data thread exit flag */
    bool thread_abort;
    /** Write data thread handle */
    pthread_t write_data_thread;
    /** Read data thread handle */
    pthread_t read_data_thread;
    /** Write complete callback function*/
    sdio_transfer_callback write_callback;
    /** Read complete callback function*/
    sdio_transfer_callback read_callback;
    /** Event callback function */
    sdio_event_callback callback;

    /** SDIO command flags */
    uint8_t cmd_flags;


    /*
     *  for command passer
     */
    uint16_t app_cmd;
    uint16_t data_cmd;
    uint16_t data_flags;

    uint32_t ier;
    uint32_t data_int;

    uint32_t data_err;
    uint32_t inserted;
};

static struct device *sdio_dev = NULL;

 
/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static uint8_t read_byte(uint32_t base, uint32_t offset)
{
    uint8_t *datap = (uint8_t*) (base + offset);
    return *datap;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static void write_byte(uint32_t base, uint8_t value, uint32_t offset)
{
    uint8_t *datap = (uint8_t*) (base + offset);

    *datap = value;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static uint16_t read_word(uint32_t base, uint32_t offset)
{
    uint16_t *datap = (uint16_t*) (base + offset);
    return *datap;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static void write_word(uint32_t base, uint16_t value, uint32_t offset)
{
    uint16_t *datap = (uint16_t*) (base + offset);

    *datap = value;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static uint32_t read_long(uint32_t base, uint32_t offset)
{
    uint32_t *datap = (uint32_t*) (base + offset);
    return *datap;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static void write_long(uint32_t base, uint32_t value, uint32_t offset)
{
    uint32_t *datap = (uint32_t*) (base + offset);

    *datap = value;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static uint32_t calc_divisor(uint32_t base_div, uint32_t target_clock)
{
    uint32_t divisor;
    uint32_t base_clock = base_div * MHZ_DEFINE; /* To MHz */

    /* Target Clock Frequency = (Base Clock) / divisor */
    divisor = base_clock / target_clock;
    if ((base_clock / divisor) > target_clock) {
        divisor++;
    }

    return divisor;
}



/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static void sdhc_reset(uint32_t base, uint8_t reset)
{
    write_byte(base, reset, SDHC_SOFTWARE_RESET);

    while (read_byte(base, SDHC_SOFTWARE_RESET) & reset) {
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
static void convert_rsp_136(uint32_t base, uint32_t *resp)
{
    uint32_t r, i;
    uint8_t b;

    for (i = 0; i < 4; i++) {
        r = read_long(base, SDHC_RESPONSE0 + (3 - i) * 4);
        b = (i < 3) ? read_byte(base, (2 - i) * 4 + 3) : 0;
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
static void sdhc_init(uint32_t base)
{
    sdhc_reset(base, SDHCI_RESET_ALL);

    
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static uint32_t sdhc_read(struct tsb_sdio_info *info, uint8_t *buf,
                          uint16_t size)
{
    uint32_t ret = 0;
    uint32_t state, fifo, i;
    uint16_t len = size;
    uint8_t *datap = buf;

    /*
     * Wait for the Buffer Read Ready interrupt or errors
     */
    while (1) {
        state = read_long(info->reg_base, SDHC_INT_ERR_STATUS);
        if (state & SDHC_INT_READ_READY) {
            write_long(info->reg_base, SDHC_INT_READ_READY,
                       SDHC_INT_ERR_STATUS);
            break;
        }
        if (state & DATA_ERR_MASK) {
            lldbg("data read error ... \n");
            ret = -1;
            goto err_data_read;
        }
    }

    /*
     * Read block data
     */
    while (len) {
        fifo = read_long(info->reg_base, SDHC_DATAPORT);
        for (i = 0; i < 4; i++, datap++, len--) {
            *datap = fifo & 0xFF;
            fifo >>= 8;
        }
    }

    /*
     * Wait for Transfer Complete interrupt.
     */
     
    // todo

err_data_read:

    return ret;
}

/**
 * @brief 
 *
 * @param 
 * @param 
 * @param 
 * @return 
 */
static void sdhc_set_clock(uint32_t base, uint32_t clock)
{
    uint16_t clk_val;
    uint32_t tsb_base_div, clk;
    
    write_word(base, 0, SDHC_CLOCK_CTRL);

    clk = read_word(base, SDHC_CAPABILITIES);
    tsb_base_div = (clk & SDHC_CAP_BASE_CLK_MASK) >> SDHC_CAP_BASE_CLK_SHIFT;
    
    clk_val = calc_divisor(tsb_base_div, clock);
    
    clk_val <<= SDHC_DIV_SHIFT;
    clk_val |= SDHC_SD_CLK_EN;
    
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
static int sdio_extract_resources(struct device *dev,
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
    info->sdio_irq = (int)r->start;

    return 0;
}


/**
 * @brief SD clock supply.
 *
 * This function is for supplying SD clock to a SD card. The clock shall be
 * supplied to the card before either of the following actions is taken.
 * 1. Issuing a SD command
 * 2. Detect an interrupt from a SD card in 4-bit mode.
 *
 * @param ios Pointer to structure of ios.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_clock_supply(struct sdio_ios *ios, struct tsb_sdio_info *info)
{
    uint32_t caps_low = 0, tsb_base_clock = 0, divisor = 0;
    uint8_t retry = 0;

    sdio_reg_bit_clr(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    /* Calculate a divisor for SD clock frequency. Base clock frequency of
     * Toshiba host controller is 192MHz (0xC0) */
    caps_low = sdio_getreg(info->reg_base, CAPABILITIES_LOW);
    tsb_base_clock = (caps_low & BASE_CLOCK_FREQ_FOR_SD_CLOCK_MASK) >>
                      BASE_CLOCK_FREQ_FOR_SD_CLOCK_SHIFT;
    divisor = sdio_calc_divisor(ios, tsb_base_clock);

    if (divisor == 0) {
        /* Use Toshiba 192MHz base clock */
        sdio_reg_bit_clr(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SDCLK_FREQ_SELECT_MASK);
    } else {
        /* SDCLK_FREQ_SELECT register is set by (divisor / 2) */
        sdio_reg_field_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                           SDCLK_FREQ_SELECT_MASK,
                           (divisor / 2) << SDCLK_FREQ_SELECT_SHIFT);
    }
    sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     INTERNAL_CLOCK_EN);

    /* Waiting for INTERNAL_CLOCK_STABLE register stable */
    while (!(sdio_getreg(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
             INTERNAL_CLOCK_STABLE) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == (REGISTER_MAX_RETRY - 1)) { /* Detect timeout */
        return -EINVAL;
    }

    sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    return 0;
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
static void sdio_enable_bus_power(struct tsb_sdio_info *info)
{
    uint32_t caps_low = sdio_getreg(info->reg_base, CAPABILITIES_LOW);
    uint32_t voltage_support = 0;

    /* Switch the pin share mode and pin high for GPB2_SD_POWER_EN pin */
    tsb_set_pinshare(TSB_PIN_GPIO9);
    gpio_activate(TSB_PIN_GPIO9);
    gpio_direction_out(GPB2_SD_POWER_EN, 1);

    /* Pull up clk and SDIO data interface pins */
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /* Get the support voltage of the Host Controller and set SD Bus Voltage
     * Select with supported maximum voltage */
    if (caps_low & VOLTAGE_SUPPORT_3_3V) {
        voltage_support = SUPPORT_3_3V;
    } else if (caps_low & VOLTAGE_SUPPORT_3_0V) {
        voltage_support = SUPPORT_3_0V;
    } else {
        voltage_support = SUPPORT_1_8V;
    }
    sdio_reg_field_set(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                       SD_BUS_VOLTAGE_SELECT_MASK,
                       voltage_support << SD_BUS_VOLTAGE_SELECT_SHIFT);

    sdio_reg_bit_set(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                     SD_BUS_POWER);
}

/**
 * @brief Disable SD bus power.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Switch the pin share mode and pin low for GPB2_SD_POWER_EN pin
 * 2. Pull down clk and SDIO data interface pins
 * 3. Set SD Bus Voltage Select register with supported maximum voltage
 * 4. Clean SD Bus Power register
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_disable_bus_power(struct tsb_sdio_info *info)
{
    uint32_t caps_low = sdio_getreg(info->reg_base, CAPABILITIES_LOW);
    uint32_t voltage_support = 0;

    /* Switch the pin share mode and pin low for GPB2_SD_POWER_EN pin */
    tsb_set_pinshare(TSB_PIN_GPIO9);
    gpio_activate(TSB_PIN_GPIO9);
    gpio_direction_out(GPB2_SD_POWER_EN, 0);

    /* Pull down clk and SDIO data interface pins */
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /* Get the support voltage of the Host Controller and set SD Bus Voltage
     * Select with supported maximum voltage */
    if (caps_low & VOLTAGE_SUPPORT_3_3V) {
        voltage_support = SUPPORT_3_3V;
    } else if (caps_low & VOLTAGE_SUPPORT_3_0V) {
        voltage_support = SUPPORT_3_0V;
    } else {
        voltage_support = SUPPORT_1_8V;
    }
    sdio_reg_field_set(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                       SD_BUS_VOLTAGE_SELECT_MASK,
                       voltage_support << SD_BUS_VOLTAGE_SELECT_SHIFT);

    sdio_reg_bit_clr(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                     SD_BUS_POWER);
}

/**
 * @brief Change SD bus width.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Disable Card Interrupt in host
 * 2. Change bit mode for host
 * 3. Enable Card Interrupt in host
 *
 * @param ios Pointer to structure of ios.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_change_bus_width(struct sdio_ios *ios,
                                 struct tsb_sdio_info *info)
{
    /* Disable Card Interrupt in host */
    //sdio_reg_bit_clr(info->reg_base, INT_ERR_STATUS_EN,
    //                 CARD_INTERRUPT_STAT_EN);
    //sdio_reg_bit_clr(info->reg_base, INT_ERR_SIGNAL_EN,
    //                 CARD_INTERRUPT_EN);

    /* Change bit mode for host */
    switch (ios->bus_width) {
    case HC_SDIO_BUS_WIDTH_1:
        sdio_reg_bit_clr(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                         DATA_TRASFER_WIDTH);
        break;
    case HC_SDIO_BUS_WIDTH_4:
    case HC_SDIO_BUS_WIDTH_8:
        /*sdio_reg_bit_set(info->reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                         DATA_TRASFER_WIDTH);
        */
        {
            uint8_t ctrl;
            ctrl = sdhci_readb(info->reg_base, SDHCI_HOST_CONTROL);
            ctrl |= SDHCI_CTRL_4BITBUS;
            sdhci_writeb(info->reg_base, ctrl, SDHCI_HOST_CONTROL);
        }

        //lldbg(".......change to 4 bits\n");
        break;
    default:
        return -EINVAL;
    }

    /* Enable Card Interrupt in host */
    //sdio_reg_bit_set(info->reg_base, INT_ERR_STATUS_EN,
    //                 CARD_INTERRUPT_STAT_EN);
    //sdio_reg_bit_set(info->reg_base, INT_ERR_SIGNAL_EN,
    //                 CARD_INTERRUPT_EN);

    return 0;
}

/**
 * @brief Recover error interrupt.
 *
 * This function follows the sequence below to recover error interrupt.
 * 1. Disable Error Interrupt Signal
 * 2. Check Error Interrupt Status for command
 * 3. Set Software Reset For CMD line (CR) if CMD Line Error occurs
 * 4. Check Error Interrupt Status for data
 * 5. Set Software Reset For DAT line (DR) if DAT Line Error occurs
 * 6. Clean previous error status
 *
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_error_interrupt_recovery(struct tsb_sdio_info *info)
{
    uint32_t cmd_line_err = CMD_INDEX_ERROR | CMD_END_BIT_ERROR |
                            CMD_CRC_ERROR | CMD_TIMEOUT_ERROR;
    uint32_t dat_line_err = DATA_END_BIT_ERROR | DATA_CRC_ERROR |
                            DATA_TIMEOUT_ERROR;
    uint8_t retry = 0;

    lldbg("sdio_error_interrupt_recovery \n");
    //sdio_dump_int_status_registers(info);

    /* Disable Error Interrupt Signal */
    sdio_reg_bit_clr(info->reg_base, INT_ERR_STATUS_EN,
                     DATA_END_BIT_ERR_STAT_EN | DATA_CRC_ERR_STAT_EN |
                     DATA_TIMEOUT_ERR_STAT_EN | CMD_INDEX_ERR_STAT_EN |
                     CMD_END_BIT_ERR_STAT_EN | CMD_CRC_ERR_STAT_EN |
                     CMD_TIMEOUT_ERR_STAT_EN);
    sdio_reg_bit_clr(info->reg_base, INT_ERR_SIGNAL_EN,
                     DATA_END_BIT_ERR_EN | DATA_CRC_ERR_EN |
                     DATA_TIMEOUT_ERR_EN | CMD_INDEX_ERR_EN |
                     CMD_END_BIT_ERR_EN | CMD_CRC_ERR_EN |
                     CMD_TIMEOUT_ERR_EN);

    /* Check Error Interrupt Status for command */
    if (info->err_status & cmd_line_err) { /* CMD Line Error occurs */
        /* Set Software Reset For CMD line (CR) */
        sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SW_RESET_CMD_LINE);

        /* Check CR */
        while ((sdio_getreg(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
                 SW_RESET_CMD_LINE) && (retry < REGISTER_MAX_RETRY)) {
            retry++;
        }

        if (retry == (REGISTER_MAX_RETRY - 1)) { /* Detect timeout */
            return -EINVAL;
        }
    }

    /* Check Error Interrupt Status for data */
    if (info->err_status & dat_line_err) { /* DAT Line Error occurs */
        /* Set Software Reset For DAT line (DR) */
        sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SW_RESET_DAT_LINE);

        /* Check DR */
        while ((sdio_getreg(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
                 SW_RESET_DAT_LINE) && (retry < REGISTER_MAX_RETRY)) {
            retry++;
        }

        if (retry == (REGISTER_MAX_RETRY - 1)) { /* Detect timeout */
            return -EINVAL;
        }
    }

    /* Clean previous error status */
    sdio_reg_bit_clr(info->reg_base, INT_ERR_STATUS,
                     DATA_END_BIT_ERROR | DATA_CRC_ERROR |
                     DATA_TIMEOUT_ERROR | CMD_INDEX_ERROR |
                     CMD_END_BIT_ERROR | CMD_CRC_ERROR |
                     CMD_TIMEOUT_ERROR);

    return 0;
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
int sdio_irq_event(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint8_t value = 0;

    gpio_mask_irq(irq);

    value = gpio_get_value(GPB2_SD_CD); // 1: no card, 0: card inserted.

    if (value && info->inserted) {
        // signal card remove
        sdhci_writel(info->reg_base, 0, SDHCI_INT_ENABLE);
        sdhci_writel(info->reg_base, 0, SDHCI_SIGNAL_ENABLE);
        sdio_disable_bus_power(info);
        sdio_reg_bit_clr(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);
        info->inserted = 0;                 
        info->card_event = HC_SDIO_CARD_REMOVED;
    } else if (!value && !info->inserted) {
        // signal card insert
        sdhci_writel(info->reg_base, CMD_INT_MASK | DATA_INT_MASK,
                     SDHCI_INT_ENABLE);
        sdhci_writel(info->reg_base, CMD_INT_MASK | DATA_INT_MASK,
                     SDHCI_SIGNAL_ENABLE);
        sdio_enable_bus_power(info);
        sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);
        info->inserted = 1;                 
        info->card_event = HC_SDIO_CARD_INSERTED;
    } else {
        info->card_event = 0;
    }

    if (info->card_event && info->callback) {
        info->callback(info->card_event);
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
static int sdio_irq_handler(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint32_t int_err_status = 0;

    info->data_err = 0;
    
    int_err_status = sdhci_readl(info->reg_base, SDHCI_INT_STATUS);

    if (int_err_status & SDHCI_INT_RESPONSE) {
        if (info->cmd_flags == HC_SDIO_RSP_R2) {
            convert_rsp_136(info->reg_base, info->resp);
        } else {
            info->resp[0] = sdhci_readl(info->reg_base, SDHCI_RESPONSE);
        }
    } else if (int_err_status & CMD_ERR_MASK) {
        lldbg("command error... \n");
    } else if (int_err_status & DATA_INT_MASK) {
        lldbg("data ready... \n");
    } else if (int_err_status & DATA_ERR_MASK) {
        info->data_err = 1;
        lldbg("data error... \n");
    } else {
        lldbg("unexpected error... \n");
    }
    
    sdhci_writel(info->reg_base, int_err_status, SDHCI_INT_STATUS);

    return 0;
}

/**
 * @brief Write data from buffer to FIFO.
 *
 * This function put the data from buffer to FIFO until the buffer is
 * empty. If buffer is empty, it calls the up layer callback
 * function in case of non-blocking mode.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_write_fifo_data(struct tsb_sdio_info *info)
{
    uint32_t presentstate = 0, buf_port = 0;
    uint32_t data_mask = BUFFER_READ_ENABLE | BUFFER_WRITE_ENABLE |
                         READ_TRANSFER_ACTIVE | WRITE_TRANSFER_ACTIVE |
                         DAT_LINE_ACTIVE | COMMAND_INHIBIT_DAT;
    int16_t remaining = 0, i = 0;
    uint8_t *wbuf = info->write_buf.buffer;

    presentstate = sdio_getreg(info->reg_base, PRESENTSTATE);
    while (presentstate & data_mask) {
        if (presentstate & BUFFER_WRITE_ENABLE) {
            sdio_reg_bit_set(info->reg_base, INT_ERR_STATUS,
                             BUFFER_WRITE_RDY);

            /* Is there at least a full uint32_t data remaining in the user
             * buffer? */
            remaining = info->write_buf.tail - info->write_buf.head;
            if (remaining >= sizeof(uint32_t)) {
                /* Yes. Write uint32_t data to the FIFO */
                buf_port = wbuf[info->write_buf.head] |
                           wbuf[info->write_buf.head + 1] << BYTE_SHIFT |
                           wbuf[info->write_buf.head + 2] << (BYTE_SHIFT * 2) |
                           wbuf[info->write_buf.head + 3] << (BYTE_SHIFT * 3);
                info->write_buf.head += sizeof(uint32_t);
            } else {
                /* No. Write the bytes remaining in the user buffer to the
                 * FIFO */
                for (i = 0; i < remaining; i++) {
                    buf_port |= wbuf[info->write_buf.head + i] <<
                                (BYTE_SHIFT * i);
                }
                info->write_buf.head = info->write_buf.tail;
            }
            sdio_putreg(info->reg_base, DATAPORTREG, buf_port);

            /* More blocks? */
            if (info->write_buf.head == info->write_buf.tail) { /* No */
                if (info->write_callback) { /* Non-blocking */
                    info->flags &= ~SDIO_FLAG_WRITE;
                    info->write_callback(info->write_buf.head, info->blksz,
                                         info->write_buf.buffer, 0);
                }
            }
        }
        presentstate = sdio_getreg(info->reg_base, PRESENTSTATE);
    }
}

/**
 * @brief Read data from FIFO to buffer.
 *
 * This function put the data from buffer to FIFO until the buffer is
 * empty. If buffer is empty, it calls the up layer callback
 * function in case of non-blocking mode.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_read_fifo_data(struct tsb_sdio_info *info)
{
    uint32_t presentstate = 0, buf_port = 0;
    uint32_t data_mask = BUFFER_READ_ENABLE | BUFFER_WRITE_ENABLE |
                         READ_TRANSFER_ACTIVE | WRITE_TRANSFER_ACTIVE |
                         DAT_LINE_ACTIVE | COMMAND_INHIBIT_DAT;
    int16_t remaining = 0, i = 0;
    uint8_t *rbuf = info->read_buf.buffer;
    uint32_t j = 2;

    presentstate = sdio_getreg(info->reg_base, PRESENTSTATE);

    //lldbg("+presentstate = 0x%08p \n", presentstate);

    while (presentstate & data_mask) {
        //lldbg("BUFFER_READ_ENABLE + \n");
        if (presentstate & BUFFER_READ_ENABLE) {
            sdio_reg_bit_set(info->reg_base, INT_ERR_STATUS,
                             BUFFER_READ_RDY);

            /* Is there at least a full uint32_t data remaining in the user
             * buffer? */
            remaining = info->read_buf.tail - info->read_buf.head;
            buf_port = sdio_getreg(info->reg_base, DATAPORTREG);

            if (j) {
                lldbg("data = 0x%08p \n", buf_port);
                j--;
            }

            if (remaining >= sizeof(uint32_t)) {
                /* Yes. Transfer uint32_t data in FIFO to the user buffer */
                rbuf[info->read_buf.head] = (uint8_t)buf_port;
                rbuf[info->read_buf.head + 1] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 1));
                rbuf[info->read_buf.head + 2] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 2));
                rbuf[info->read_buf.head + 3] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 3));
                info->read_buf.head += sizeof(uint32_t);
            } else {
                /* No. Transfer the bytes remaining in FIFO to the user
                 * buffer */
                for (i = 0; i < remaining; i++) {
                    rbuf[info->read_buf.head + i] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * i));
                }
                info->read_buf.head = info->read_buf.tail;
            }

            /* More blocks? */
            if (info->read_buf.head == info->read_buf.tail) { /* No */
                lldbg("buffer full ...\n");
                if (info->read_callback) { /* Non-blocking */
                    info->flags &= ~SDIO_FLAG_READ;
                    info->read_callback(info->read_buf.head, info->blksz,
                                        info->read_buf.buffer, 0);
                } else {
                    lldbg("stop read ....\n");
                    break;
                }
            }

            lldbg("remaining = %d \n", remaining);
        }
        presentstate = sdio_getreg(info->reg_base, PRESENTSTATE);
        //lldbg("-presentstate = 0x%08p \n", presentstate);
    }
}

/**
 * @brief SDIO write data thread
 *
 * This is the thread for non-blocking data write.
 *
 * @return None.
 */
static void *sdio_write_data_thread(void *data)
{
    struct tsb_sdio_info *info = data;

    while (1) {
        sem_wait(&info->write_sem);
        if (info->thread_abort) {
            /* Exit sdio_write_data_thread loop */
            break;
        }
        sdio_write_fifo_data(info);
    }

    return NULL;
}

/**
 * @brief SDIO read data thread
 *
 * This is the thread for non-blocking data read.
 *
 * @return None.
 */
static void *sdio_read_data_thread(void *data)
{
    struct tsb_sdio_info *info = data;

    while (1) {
        //lldbg("sem_wait+ \n");
        sem_wait(&info->read_sem);
        //lldbg("sem_wait- \n");
        if (info->thread_abort) {
            /* Exit sdio_read_data_thread loop */
            break;
        }
        sdio_read_fifo_data(info);
    }

    return NULL;
}

/**
 * @brief Get capabilities of SD host controller
 *
 * This function is to get capabilities of SD host controller. The
 * capabilities include:
 * 1. Unremovable support
 * 2. 4 bit transfers support
 * 3. 8 bit transfers support
 * 4. MMC high-speed timings support
 * 5. SD high-speed timings support
 * 6. Erase and trim commands support
 * 7. DDR mode at 1.2V support
 * 8. DDR mode at 1.8V support
 * 9. Power off card support
 * 10. UHS SDR12 mode support
 * 11. UHS SDR25 mode support
 * 12. UHS SDR50 mode support
 * 13. UHS SDR104 mode support
 * 14. UHS DDR50 mode support
 * 15. Driver Type A support
 * 16. Driver Type C support
 * 17. Driver Type D support
 * 18. HS200 mode at 1.2V support
 * 19. HS200 mode at 1.8V support
 * 20. HS400 mode at 1.2V support
 * 21. HS400 mode at 1.8V support
 * 22. SD VDD voltage range
 * 23. Maximum number of blocks per data command transfer
 * 24. Maximum size of each block to transfer
 *
 * @param dev Pointer to structure of device.
 * @param cap Pointer to structure of capabilities.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_get_capability(struct device *dev, struct sdio_cap *cap)
{
    /* Not support for HC_SDIO_CAP_NONREMOVABLE, HC_SDIO_CAP_8_BIT_DATA,
     * HC_SDIO_CAP_1_2V_DDR, HC_SDIO_CAP_1_8V_DDR, HC_SDIO_CAP_UHS_SDR50,
     * HC_SDIO_CAP_UHS_SDR104, HC_SDIO_CAP_UHS_DDR50, HC_SDIO_CAP_HS200_1_2V,
     * HC_SDIO_CAP_HS200_1_8V, HC_SDIO_CAP_HS400_1_2V, HC_SDIO_CAP_HS400_1_8V,
     * HC_SDIO_CAP_ERASE */
    //lldbg("tsb_sdio_get_capability() \n");
    cap->caps = HC_SDIO_CAP_4_BIT_DATA | HC_SDIO_CAP_MMC_HS |
                HC_SDIO_CAP_SD_HS | HC_SDIO_CAP_POWER_OFF_CARD |
                HC_SDIO_CAP_UHS_SDR12 | HC_SDIO_CAP_UHS_SDR25 |
                HC_SDIO_CAP_DRIVER_TYPE_A | HC_SDIO_CAP_DRIVER_TYPE_C |
                HC_SDIO_CAP_DRIVER_TYPE_D;
    cap->ocr = HC_SDIO_VDD_33_34;
    cap->max_blk_count = MAX_BLK_COUNT;
    cap->max_blk_size = MAX_BLK_SIZE;

    return 0;
}

/**
 * @brief Set ios of SD host controller
 *
 * This function is to set the parameters to configure the SD controller. The
 * parameters include:
 * 1. Clock rate in Hz
 * 2. Voltage range
 * 3. Bus mode
 * 4. Power_mode
 * 5. Bus width
 * 6. Timing
 * 7. Signal voltage
 * 8. Driver type
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd);

static int tsb_sdio_set_ios(struct device *dev, struct sdio_ios *ios)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    uint32_t uhs_mode_sel = 0;

    //lldbg("ios->power_mode =  \n");

    /* Set clock */
    if (sdio_clock_supply(ios, info)) {
        return -EINVAL;
    }

    /* Set vdd */
    /* Not support to change vdd since board uses jumper to change power */

    /* set bus mode */
    /* Not support */

    /* Set power mode */
    switch (ios->power_mode) {
    case HC_SDIO_POWER_OFF:
        sdio_disable_bus_power(info);
        break;
    case HC_SDIO_POWER_UP:
    case HC_SDIO_POWER_ON:
        sdio_enable_bus_power(info);
        break;
    case HC_SDIO_POWER_UNDEFINED:
    default:
        //lldbg("ios->power_mode ERR \n");
        return -EINVAL;
    }

    /* Set bus width */
    if (sdio_change_bus_width(ios, info)) {
        return -EINVAL;
    } /*else { // workaround for first CMD17 error

        if (ios->bus_width == HC_SDIO_BUS_WIDTH_4) {
            struct sdio_cmd cmd;
            uint32_t resp[4];

            cmd.cmd = 100;
            cmd.cmd_flags = 0x15;
            cmd.cmd_type = 0x01;
            cmd.cmd_arg = 0;
            cmd.resp = resp;
            cmd.data_cmd = 1;
            cmd.data_blocks = 0;
            cmd.data_blksz = 0;
            cmd.data_blksz = 0;

            tsb_sdio_send_cmd(dev, &cmd);
        }
    }*/

    /* Set timing */
    switch (ios->timing) {
    case HC_SDIO_TIMING_LEGACY:
    case HC_SDIO_TIMING_MMC_HS:
    case HC_SDIO_TIMING_SD_HS:
    case HC_SDIO_TIMING_UHS_SDR12:
        uhs_mode_sel = SDR12;
        break;
    case HC_SDIO_TIMING_UHS_SDR25:
        uhs_mode_sel = SDR25;
        break;
    case HC_SDIO_TIMING_UHS_SDR50:
        uhs_mode_sel = SDR50;
        break;
    case HC_SDIO_TIMING_UHS_SDR104:
        uhs_mode_sel = SDR104;
        break;
    case HC_SDIO_TIMING_UHS_DDR50:
        uhs_mode_sel = DDR50;
        break;
    case HC_SDIO_TIMING_MMC_DDR52:
    case HC_SDIO_TIMING_MMC_HS200:
    case HC_SDIO_TIMING_MMC_HS400:
    default:
        //lldbg("ios->timing ERR \n");
        return -EINVAL;
    }
    sdio_reg_field_set(info->reg_base, AUTOCMD12ERST_HOST_CTRL2,
                       UHS_MODE_SEL_MASK,
                       uhs_mode_sel << UHS_MODE_SEL_SHIFT);

    /* Set signal voltage */
    /* Not support to change signal voltage since board uses jumper to change
     * signal_voltage */

    /* Set driver type */
    switch (ios->drv_type) {
    case HC_SDIO_SET_DRIVER_TYPE_A:
        sdio_reg_bit_set(info->reg_base, CAPABILITIES_HI,
                         DRIVER_TYPEA_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_C:
        sdio_reg_bit_set(info->reg_base, CAPABILITIES_HI,
                         DRIVER_TYPEC_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_D:
        sdio_reg_bit_set(info->reg_base, CAPABILITIES_HI,
                         DRIVER_TYPED_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_B:
        break; // alex
    default:
        //lldbg("ios->drv_type ERR \n");
        return -EINVAL;
    }

    //lldbg("tsb_sdio_set_ios() OK \n");
    return 0;
}

/**
 * @brief Send SDIO command through SDIO host controller
 *
 * This function follows the sequence to send command to card through SD host
 * controller.
 * 1. Check Command Inhibit (CMD) register
 * 2. Check that does issue the command with the busy
 * 3. Check that does issue Abort Command
 * 4. Check Command Inhibit (DAT) register
 * 5. Set Argument 1 Reg register
 * 6. Set Command Reg register
 * 7. Wait for Command Complete Int register
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static void set_transfer_mode(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint16_t mode = 0;

    if (cmd->cmd == HC_MMC_READ_MULTIPLE_BLOCK ||
        cmd->cmd == HC_MMC_WRITE_MULTIPLE_BLOCK || cmd->data_blocks > 1) {

        mode = SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_MULTI;

        if (cmd->cmd_flags & SDHCI_AUTO_CMD12 && (cmd->cmd != 53)) {
            mode |= SDHCI_TRNS_AUTO_CMD12;
        }
        else if (cmd->cmd_flags & SDHCI_AUTO_CMD23) {
            sdhci_writel(info->reg_base, cmd->cmd_arg, SDHCI_ARGUMENT2);
        }
    }

    if (info->data_flags & MMC_DATA_READ)
        mode |= SDHCI_TRNS_READ;

    sdhci_writew(info->reg_base, mode, SDHCI_TRANSFER_MODE);
}

static void cmd_parser(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint16_t app_cmd = info->app_cmd;


    if (app_cmd && (cmd->cmd == HC_SD_APP_SEND_SCR)) { // CMD51
        //info->data_cmd = 1;
        //info->blocks = 1;
        //info->blksz = 8;
        info->data_flags = MMC_DATA_READ;
    } else if (app_cmd && (cmd->cmd == HC_MMC_SEND_STATUS)) { // CMD13
        //info->data_cmd = 1;
        //info->blocks = 1;
        //info->blksz = 64;
        info->data_flags = MMC_DATA_READ;
    } else if (!app_cmd && (cmd->cmd == HC_MMC_SWITCH)) { // CMD6
        //info->data_cmd = 1;
        //info->blocks = 1;
        //info->blksz = 64;
        info->data_flags = MMC_DATA_READ;
    /*} else if (cmd->cmd == HC_MMC_SET_BLOCKLEN) { // CMD16
        info->blksz = (uint16_t)cmd->cmd_arg;
    } else if (cmd->cmd == HC_MMC_SET_BLOCK_COUNT) { // CMD23
        info->blocks = (uint16_t)cmd->cmd_arg;*/
    } else if (cmd->cmd == HC_MMC_READ_SINGLE_BLOCK) { // CMD17
        //info->data_cmd = 1;
        //info->blocks = 1;
        //info->blksz = 512;
        info->data_flags = MMC_DATA_READ;
    } else if (cmd->cmd == HC_MMC_READ_MULTIPLE_BLOCK) { // CMD18
        //info->data_cmd = 1;
        info->data_flags = MMC_DATA_READ;
    } else if (cmd->cmd == HC_MMC_WRITE_BLOCK) { // CMD24
        //info->data_cmd = 1;
        //info->blocks = 1;
        //info->blksz = 512;
        info->data_flags = MMC_DATA_WRITE;
    } else if (cmd->cmd == HC_MMC_WRITE_MULTIPLE_BLOCK) { // CMD25
        //info->data_cmd = 1;
        info->data_flags = MMC_DATA_WRITE;
    } else if (cmd->cmd == 100) { // CMD17 for workaround
        cmd->cmd = 17;
        info->data_cmd = 1;
        info->blocks = 0;
        info->blksz = 0;
    } else {
        info->data_cmd = 0;
        info->blocks = 0;
        info->blksz = 0;
        info->data_flags = 0;
    }

    info->app_cmd = (cmd->cmd == 55)? 1: 0;
}

static int tsb_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    //uint32_t cmd_reg = 0;
    uint8_t cmd_flags = 0;
    uint8_t retry = 0;

    //lldbg("tsb_sdio_send_cmd() \n");

    lldbg("*** CMD(%d), Flags(0x%02x), Type(0x%02x), Arg(0x%08x) *** \n",
          cmd->cmd, cmd->cmd_flags, cmd->cmd_type, cmd->cmd_arg);

    info->cmd_flags = cmd->cmd_flags;

    cmd_parser(dev, cmd);

    /* Enable command interrupts */

    sdio_reg_bit_set(info->reg_base, INT_ERR_STATUS_EN,
                     CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
    sdio_reg_bit_set(info->reg_base, INT_ERR_SIGNAL_EN,
                     CMD_TIMEOUT_ERR_EN |CMD_COMPLETE_EN);


    /*{
        uint32_t mask = SDHCI_INT_DATA_END_BIT |
                SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT |
                SDHCI_INT_END_BIT | SDHCI_INT_CRC |
                SDHCI_INT_TIMEOUT | SDHCI_INT_DATA_END |
                SDHCI_INT_RESPONSE;

        sdhci_writel(info->reg_base, mask, SDHCI_INT_ENABLE);
        sdhci_writel(info->reg_base, mask, SDHCI_SIGNAL_ENABLE);
    }*/

    /* Wait for CMD Line unused */
    while ((sdio_getreg(info->reg_base, PRESENTSTATE) &
           COMMAND_INHIBIT_CMD) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == (REGISTER_MAX_RETRY - 1)) { /* Detect timeout */
        return -EINVAL;
    }

    /*
     * prepare data information
     */
    if (cmd->data_cmd) {
        uint32_t int_reg;
        //lldbg("cmd with data\n");

        int_reg = sdhci_readl(info->reg_base, SDHCI_INT_ENABLE);

        int_reg |= (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL);

        int_reg |= SDHCI_INT_DATA_END_BIT | SDHCI_INT_DATA_CRC |
                   SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_END_BIT |
                   SDHCI_INT_CRC | SDHCI_INT_TIMEOUT | SDHCI_INT_DATA_END |
                   SDHCI_INT_RESPONSE;

        sdhci_writel(info->reg_base, int_reg, SDHCI_INT_ENABLE);
        sdhci_writel(info->reg_base, int_reg, SDHCI_SIGNAL_ENABLE);

        sdhci_writew(info->reg_base, cmd->data_blksz, SDHCI_BLOCK_SIZE);
        sdhci_writew(info->reg_base, cmd->data_blocks, SDHCI_BLOCK_COUNT);

        sdhci_writeb(info->reg_base, 0x03, SDHCI_TIMEOUT_CONTROL);

        lldbg(">> data: size(%d), blocks(%d) \n", cmd->data_blksz, cmd->data_blocks);
    }

    //lldbg("test 1\n");

    /* Issue the command with the busy? */
    if (cmd->cmd_flags == HC_SDIO_RSP_R1B) {
        /* Issue Abort Command? */
        if (cmd->cmd_type != HC_SDIO_CMD_BCR) {
            /* Wait for DAT Line unused */
            while ((sdio_getreg(info->reg_base, PRESENTSTATE) &
                   COMMAND_INHIBIT_DAT) && (retry < REGISTER_MAX_RETRY)) {
                usleep(REGISTER_INTERVAL);
                retry++;
            }

            if (retry == (REGISTER_MAX_RETRY - 1)) { /* Detect timeout */
                return -EINVAL;
            }
        }
    }

    //lldbg("test 2\n");



    /* Set Argument 1 Reg register */
    sdio_putreg(info->reg_base, ARGUMENT1, cmd->cmd_arg);

    set_transfer_mode(dev, cmd);

    if (!(cmd->cmd_flags & MMC_RSP_PRESENT))
        cmd_flags = SDHCI_CMD_RESP_NONE;
    else if (cmd->cmd_flags & MMC_RSP_136)
        cmd_flags = SDHCI_CMD_RESP_LONG;
    else if (cmd->cmd_flags & MMC_RSP_BUSY)
        cmd_flags = SDHCI_CMD_RESP_SHORT_BUSY;
    else
        cmd_flags = SDHCI_CMD_RESP_SHORT;

    if (cmd->cmd_flags & MMC_RSP_CRC)
        cmd_flags |= SDHCI_CMD_CRC;
    if (cmd->cmd_flags & MMC_RSP_OPCODE)
        cmd_flags |= SDHCI_CMD_INDEX;

    if (cmd->data_cmd) {
        cmd_flags |= SDHCI_CMD_DATA;
    }

    lldbg("cmd = 0x%04x \n", SDHCI_MAKE_CMD(cmd->cmd, cmd_flags));

    if (cmd->cmd == 17) {
        //sdhci_writel(info->reg_base, 0, SDHCI_DMA_ADDRESS);
            /*{
                uint8_t mask = SDHCI_RESET_DATA;
                sdhci_writeb(info->reg_base, mask, SDHCI_SOFTWARE_RESET);

                while (sdhci_readb(info->reg_base, SDHCI_SOFTWARE_RESET) & mask) {
                    lldbg("data software reset..\n");
                };
            }*/

        //sdio_dump_registers(info);
    }

    sdhci_writew(info->reg_base, SDHCI_MAKE_CMD(cmd->cmd, cmd_flags),
                 SDHCI_COMMAND);

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
    usleep(COMMAND_INTERVAL);

    //lldbg("after wait... \n");
    //sdio_dump_registers(info);

/*    if (cmd->cmd == 9) {
        cmd->resp[0] = cpu_to_be32(info->resp[0]);
        cmd->resp[1] = cpu_to_be32(info->resp[1]);
        cmd->resp[2] = cpu_to_be32(info->resp[2]);
        cmd->resp[3] = cpu_to_be32(info->resp[3]);
    } else {*/
        cmd->resp[0] = info->resp[0];
        cmd->resp[1] = info->resp[1];
        cmd->resp[2] = info->resp[2];
        cmd->resp[3] = info->resp[3];
//    }

    return info->sdio_int_err_status;
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
    //lldbg("tsb_sdio_write() \n");

    if (info->flags & SDIO_FLAG_WRITE) {
        return -EBUSY;
    }
    info->flags |= SDIO_FLAG_WRITE;

    info->write_buf.buffer = transfer->data;
    info->write_buf.head = 0;
    info->write_buf.tail = (uint16_t)(transfer->blocks * transfer->blksz);
    info->blocks = transfer->blocks;
    info->blksz = transfer->blksz;
    info->write_callback = transfer->callback;

    if (!info->write_callback) { /* Blocking */
        sdio_write_fifo_data(info);
        info->flags &= ~SDIO_FLAG_WRITE;
    } else { /* Non-blocking */
        sem_post(&info->write_sem);
    }

    return 0;
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

    lldbg("tsb_sdio_read()+ block = %d, blksz = %d \n", transfer->blocks, transfer->blksz);

    //sdio_dump_registers(info);

    if (info->flags & SDIO_FLAG_READ) {
        //lldbg("tsb_sdio_read() flag err \n");
        return -EBUSY;
    }
    info->flags |= SDIO_FLAG_READ;

    info->read_buf.buffer = transfer->data;
    info->read_buf.head = 0;
    info->read_buf.tail = (uint16_t)(transfer->blocks * transfer->blksz);
    info->blocks = transfer->blocks;
    info->blksz = transfer->blksz;
    info->read_callback = transfer->callback;

    if (!info->read_callback) { /* Blocking */
        //lldbg("tsb_sdio_read() blocking + \n");
        sdio_read_fifo_data(info);
        info->flags &= ~SDIO_FLAG_READ;
        //lldbg("tsb_sdio_read() blocking - \n");
    } else { /* Non-blocking */
        sem_post(&info->read_sem);
    }

    lldbg("tsb_sdio_read()- \n");

    return 0;
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
    //lldbg("tsb_sdio_attach_callback() \n");

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

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
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    int ret = 0;
    uint8_t value = 0;

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
    info->card_event = HC_SDIO_CARD_REMOVED;
    info->pre_card_event = HC_SDIO_CARD_REMOVED;

    info->thread_abort = false;
    ret = pthread_create(&info->write_data_thread, NULL, sdio_write_data_thread,
                         info);
    if (ret) {
        goto err_irqrestore;
    }

    ret = pthread_create(&info->read_data_thread, NULL, sdio_read_data_thread,
                         info);
    if (ret) {
        goto err_irqrestore;
    }

    sdhc_reset(info->reg_base, SDHCI_RESET_ALL )

    value = gpio_get_value(GPB2_SD_CD);

    /* Card is inserted before SDIO device open. */
    if (!value) {
        /* Enable command interrupts */
        sdhci_writel(info->reg_base, CMD_INT_MASK | CMD_ERR_MASK,
                     SDHCI_INT_ENABLE);
        sdhci_writel(info->reg_base, CMD_INT_MASK | CMD_ERR_MASK,
                     SDHCI_SIGNAL_ENABLE);

        sdio_enable_bus_power(info);
        
        sdio_reg_bit_set(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);

        info->card_event = HC_SDIO_CARD_INSERTED;
        if (info->pre_card_event != info->card_event) {
            if (info->callback) {
                info->callback(info->card_event);
            }
            info->pre_card_event = info->card_event;
        }
    }

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

    info->flags &= ~SDIO_FLAG_OPEN;
    info->card_event = HC_SDIO_CARD_REMOVED;
    info->pre_card_event = HC_SDIO_CARD_REMOVED;

    /* Disable command interrupts */
    /*sdio_reg_bit_clr(info->reg_base, INT_ERR_STATUS_EN,
                     CARD_INTERRUPT_STAT_EN);
    sdio_reg_bit_clr(info->reg_base, INT_ERR_SIGNAL_EN,
                     CARD_INTERRUPT_EN);
    */
    sdhci_writel(info->reg_base, 0, SDHCI_INT_ENABLE);
    sdhci_writel(info->reg_base, 0, SDHCI_SIGNAL_ENABLE);

    sdhci_writel(info->reg_base, 0, SDHCI_DMA_ADDRESS);

    sdio_reg_bit_clr(info->reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    sdio_disable_bus_power(info);

    info->thread_abort = true;
    sem_post(&info->read_sem);
    sem_post(&info->write_sem);
    if (info->read_data_thread != (pthread_t)0) {
        /* Wait for read_data_thread completed */
        pthread_join(info->read_data_thread, NULL);
    }
    if (info->write_data_thread != (pthread_t)0) {
        /* Wait for write_data_thread completed */
        pthread_join(info->write_data_thread, NULL);
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

    /* Enable the 2 clock gating (sdioSysClk/sdioSdClk) */
    tsb_clk_enable(TSB_CLK_SDIOSYS);
    tsb_clk_enable(TSB_CLK_SDIOSD);

    /* Reset the host controller */
    tsb_reset(TSB_RST_SDIOSYS);
    tsb_reset(TSB_RST_SDIOSD);

    /* Assert the DLL enable */
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_DLLCTRL, DLL_ENABLE);

    /* Switch the pin share mode for SD Interfaces */
    tsb_set_pinshare(TSB_PIN_SDIO);

    /* SD CD pin and interrupt handler initialization */
    tsb_set_pinshare(TSB_PIN_GPIO22);
    gpio_activate(GPB2_SD_CD);
    gpio_direction_in(GPB2_SD_CD);
    set_gpio_triggering(GPB2_SD_CD, IRQ_TYPE_EDGE_BOTH);
    gpio_irqattach(GPB2_SD_CD, sdio_irq_event);
    gpio_unmask_irq(GPB2_SD_CD);

    ret = sdio_extract_resources(dev, info);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->cmd_sem, 0, 0);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->write_sem, 0, 0);
    if (ret) {
        goto err_destroy_cmd_sem;
    }

    ret = sem_init(&info->read_sem, 0, 0);
    if (ret) {
        goto err_destroy_write_sem;
    }

    flags = irqsave();

    ret = irq_attach(info->sdio_irq, sdio_irq_handler);
    if (ret) {
        goto err_destroy_read_sem;
    }

    up_enable_irq(info->sdio_irq);

    sdio_dev = dev;
    info->dev = dev;
    device_set_private(dev, info);

    irqrestore(flags);

    return 0;

err_destroy_read_sem:
    irqrestore(flags);
    sem_destroy(&info->read_sem);
err_destroy_write_sem:
    sem_destroy(&info->write_sem);
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
    up_disable_irq(info->sdio_irq);
    irq_detach(info->sdio_irq);

    sem_destroy(&info->read_sem);
    sem_destroy(&info->write_sem);
    sem_destroy(&info->cmd_sem);

    gpio_unmask_irq(GPB2_SD_CD);
    gpio_deactivate(GPB2_SD_CD);
    irqrestore(flags);

    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_DLLCTRL, DLL_ENABLE);

    tsb_clk_disable(TSB_CLK_SDIOSYS);
    tsb_clk_disable(TSB_CLK_SDIOSD);

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

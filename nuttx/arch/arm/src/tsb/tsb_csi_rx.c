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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <debug.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <arch/tsb/cdsi.h>
#include <arch/tsb/cdsi0_offs_def.h>
#include <arch/tsb/cdsi0_reg_def.h>
#include <arch/tsb/csi.h>

    /*
     * BIT4: Clock Lane
     * 0: Lane operation disabled
     * 1: Lane operation enabled
     *
     * BIT0-BIT2: Data Lane
     * In order to receive data, at least Data Lane 0 shall be enabled.
     * 000: All data lane is disabled.
     * 001: Data Lane 0 is enabled.
     * 010: Data Lane 0 and 1 are enabled.
     * 011: Data Lane 0, 1 and 2 are enabled.
     * 100: Data Lane 0, 1, 2 and 3 are enabled
     */

#define AL_RX_BRG_MODE_VAL                              0x00000003
#define AL_RX_BRG_CSI_INFO_VAL                          0x00000000
#define AL_RX_BRG_CSI_DT0_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT1_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT2_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT3_VAL                           0x00000000

#define CDSIRX_CLKEN_VAL                                0x00000001
#define CDSIRX_CLKSEL_VAL                               0x00000101
#define CDSIRX_MODE_CONFIG_VAL                          0x00000001
#define CDSIRX_LANE_ENABLE_VAL                          0x00000012
#define CDSIRX_VC_ENABLE_VAL                            0x0000000F
#define CDSIRX_LINE_INIT_COUNT_VAL                      0x000012C0
#define CDSIRX_HSRXTO_COUNT_VAL                         0xFFFFFFFF
#define CDSIRX_FUNC_ENABLE_VAL                          0x00070701
#define CDSIRX_DSI_LPTX_MODE_VAL                        0x00000001
#define CDSIRX_DSI_TATO_COUNT_VAL                       0xFFFFFFFF
#define CDSIRX_DSI_LPTXTO_COUNT_VAL                     0xFFFFFFFF
#define CDSIRX_FUNC_MODE_VAL                            0x00000000
#define CDSIRX_PPI_HSRX_CNTRL_VAL                       0x40000000
#define CDSIRX_PPI_HSRX_COUNT_VAL                       0x0400000A
#define CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL            0x00000003
#define CDSIRX_PPI_DPHY_DLYCNTRL_VAL                    0x00000000
#define CDSIRX_PPI_DPHY_LPRX_THSLD_VAL                  0x000002AA
#define CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL         0x00000001
#define CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL                 0x00000FFF
#define CDSIRX_PPI_DSI_BTA_COUNT_VAL                    0x000407FF
#define CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL                0x00000002
#define CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL                 0x000002AA
#define CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL                0x00190040
#define CDSIRX_LPRX_STATE_INT_MASK_VAL                  0x1F1F1F1D

#define CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_RXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_TXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL          0x00000000

#define CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL              0x10000010
#define CDSI0_CDSIRX_START_VAL                          0x00000001
#define CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL            0x00000001
#define CDSI0_CDSIRX_ADDRESS_CONFIG_VAL                 0x00000000

#define CDSI0_CDSIRX_STOP                               0x00000000
#define CDSI0_CDSIRX_SYSTEM_CLEAR_VAL                   0x00000001
#define CDSI0_CDSIRX_CLKDISABLE_VAL                     0x00000000
#define CDSI0_AL_RX_BRG_DISABLE_VAL                     0x00000000

/**
 * @brief Initialize the CSI receiver
 * @param dev dev pointer to structure of cdsi_dev device data
 * @return 0 on success of a negative error value on error
 *
 * Initialization of the CSI receiver configures and enables the CSIRX block
 * and performs LPRX calibration. This results in the CSI receiver being clocked
 * and ready to be started.
 *
 * The receiver shouldn't be left initialized when unused as it consumes power
 * in that state, even if not started yet. To uninitialize the CSI receiver and
 * put it back to sleep call csi_rx_uninit().
 *
 * This function is stateless and must only be called when the CSI receiver is
 * unintialized.
 */
int csi_rx_init(struct cdsi_dev *dev, const struct csi_rx_config *cfg)
{
    unsigned int timeout;

    /* Enable the Rx bridge and set to CSI mode */
    cdsi_write(dev, CDSI0_AL_RX_BRG_MODE_OFFS, AL_RX_BRG_MODE_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_INFO_OFFS, AL_RX_BRG_CSI_INFO_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT0_OFFS, AL_RX_BRG_CSI_DT0_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT1_OFFS, AL_RX_BRG_CSI_DT1_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT2_OFFS, AL_RX_BRG_CSI_DT2_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT3_OFFS, AL_RX_BRG_CSI_DT3_VAL);

    /* Enable CDSIRX */
    cdsi_write(dev, CDSI0_CDSIRX_CLKEN_OFFS, CDSIRX_CLKEN_VAL);

    /* Set CDSIRX functions enable */
    cdsi_write(dev, CDSI0_CDSIRX_FUNC_ENABLE_OFFS, CDSIRX_FUNC_ENABLE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_ADDRESS_CONFIG_OFFS,
               CDSI0_CDSIRX_ADDRESS_CONFIG_VAL);

    /* Set LPRX calibration */
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRXCALCNTRL_OFFS,
               CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRX_THSLD_OFFS,
               CDSIRX_PPI_DPHY_LPRX_THSLD_VAL);

    /* Clear the LPRX calibration status and start LPRX calibration. */
    cdsi_write(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS,
               CDSI0_CDSIRX_LPRX_STATE_INT_STAT_AUTOCALDONE_MASK);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL);

    /* CDSIRX configuration */
    cdsi_write(dev, CDSI0_CDSIRX_CLKSEL_OFFS, CDSIRX_CLKSEL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_MODE_CONFIG_OFFS, CDSIRX_MODE_CONFIG_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_LANE_ENABLE_OFFS, CDSIRX_LANE_ENABLE_VAL); /* Update with lane count */
    cdsi_write(dev, CDSI0_CDSIRX_VC_ENABLE_OFFS, CDSIRX_VC_ENABLE_VAL); /* Update with VC mask */
    cdsi_write(dev, CDSI0_CDSIRX_LINE_INIT_COUNT_OFFS,
               CDSIRX_LINE_INIT_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_HSRXTO_COUNT_OFFS,
               CDSIRX_HSRXTO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_FUNC_MODE_OFFS, CDSIRX_FUNC_MODE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPTXTIMECNT_OFFS,
               CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS,
               CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DSI_BTA_COUNT_OFFS,
               CDSIRX_PPI_DSI_BTA_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_HSRX_CNTRL_OFFS,
               CDSIRX_PPI_HSRX_CNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_HSRX_COUNT_OFFS,
               CDSIRX_PPI_HSRX_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DSI_DPHYTX_ADJUST_OFFS,
               CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_HSRX_ADJUST_OFFS,
               CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_DLYCNTRL_OFFS,
               CDSIRX_PPI_DPHY_DLYCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_LPRX_STATE_INT_MASK_OFFS,
               CDSIRX_LPRX_STATE_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_RXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_RXERR_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_TXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_TXERR_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_LPTXTO_COUNT_OFFS,
               CDSIRX_DSI_LPTXTO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_TATO_COUNT_OFFS,
               CDSIRX_DSI_TATO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_WAITBTA_COUNT_OFFS,
               CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL);

    /* Wait for LPRX calibration to complete. */
    for (timeout = 50; timeout != 0; --timeout) {
        uint32_t val;

        val = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
        if (val & CDSI0_CDSIRX_LPRX_STATE_INT_STAT_AUTOCALDONE_MASK)
            break;

        usleep(10);
    }

    if (timeout == 0) {
        printf("cdsi: LPRX calibration timeout\n");
        return -ETIMEDOUT;
    }

    printf("cdsi: LPRX calibration complete (%u us)\n", (50 - timeout) * 10);
    return 0;
}

/**
 * @brief Uninitialize the CSI receiver
 * @param dev dev pointer to structure of cdsi_dev device data
 * @return 0 on success of a negative error value on error
 *
 * Uninitializing the CSI receiver disables the clock to the CSIRX block,
 * lowering its power consumption. The receiver should be kept uninitialized for
 * as long as possible.
 *
 * This function is stateless and must only be called when the CSI receiver is
 * stopped and initialized.
 */
int csi_rx_uninit(struct cdsi_dev *dev)
{
    uint32_t val;

    /*
     * Disable CDSIRX and the RX bridge and verify that the bridge became
     * idle.
     */
    cdsi_write(dev, CDSI0_CDSIRX_CLKEN_OFFS, CDSI0_CDSIRX_CLKDISABLE_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_MODE_OFFS, CDSI0_AL_RX_BRG_DISABLE_VAL);

    usleep(10);

    val = cdsi_read(dev, CDSI0_AL_RX_BRG_MODE_OFFS);
    if (val & CDSI0_AL_RX_BRG_MODE_BUSY_MASK)
        printf("cdsi: RX bridge failed to become idle (0x%08x)\n", val);

    return 0;
}

/**
 * @brief Start the CSI receiver
 * @param dev dev pointer to structure of cdsi_dev device data
 * @return 0 on success of a negative error value on error
 *
 * Starting the CSI receiver enables sending of UniPro packets encapsulating
 * CSI-2 data. Before starting the CSI receiver it must be initialized with a
 * call to csi_rx_init().
 *
 * The CSI receiver can only be started when all lanes of the CSI input are in
 * the stopped state (LP-11). The caller must thus start the CSI input after the
 * CSI receiver.
 *
 * This function is stateless and must only be called when the CSI receiver is
 * initialized and not started.
 *
 * To stop the CSI receiver call csi_rx_stop().
 */
int csi_rx_start(struct cdsi_dev *dev)
{
    unsigned int timeout;

    /* Clear the line initialization done status. */
    cdsi_write(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS,
               CDSI0_CDSIRX_LPRX_STATE_INT_STAT_LINEINITDONE_MASK);

    /* Start CDSIRX */
    cdsi_write(dev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_START_VAL);

    /* Wait for line initialization to complete. */
    for (timeout = 50; timeout != 0; --timeout) {
        uint32_t val;

        val = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
        if (val & CDSI0_CDSIRX_LPRX_STATE_INT_STAT_LINEINITDONE_MASK)
            break;

        usleep(10);
    }

    if (timeout == 0) {
        printf("cdsi: line initialization timeout\n");
        return -ETIMEDOUT;
    }

    printf("cdsi: line initialization complete (%u us)\n", (50 - timeout) * 10);
    return 0;
}

#define CSI_RX_STOP_TIMEOUT     5 /* 50 us */

/**
 * @brief Stop the CSI receiver
 * @param dev dev pointer to structure of cdsi_dev device data
 * @return 0 on success of a negative error value on error
 *
 * Stopping the CSI receiver disables sending of UniPro packets. Once stopped
 * the CSI receiver can be put to power down mode with a call to
 * csi_rx_uninit().
 *
 * The CSI receiver can only be stopped when all lanes of the CSI input are in
 * the stopped state (LP-11). The caller must thus stop the CSI input before the
 * CSI receiver.
 *
 * This function is stateless and must only be called when the CSI receiver is
 * started.
 */
int csi_rx_stop(struct cdsi_dev *dev)
{
    const uint32_t hs_mask = CDSI0_CDSIRX_LANE_STATUS_HS_CLRXACTIVEHS_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_HS_D3RXACTIVEHS_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_HS_D2RXACTIVEHS_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_HS_D1RXACTIVEHS_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_HS_D0RXACTIVEHS_MASK;
    const uint32_t lp_mask = CDSI0_CDSIRX_LANE_STATUS_LP_CLSTOPSTATE_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_LP_L3STOPSTATE_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_LP_L2STOPSTATE_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_LP_L1STOPSTATE_MASK
                           | CDSI0_CDSIRX_LANE_STATUS_LP_L0STOPSTATE_MASK;
    unsigned int timeout;
    uint32_t val;

    /*
     * Wait for the CSI receiver to become idle. Lanes should first switch to
     * Low Power mode, then to the stop state (LP-11) and finally the CSI
     * receiver should become idle.
     */
    for (timeout = 0; timeout < CSI_RX_STOP_TIMEOUT; ++timeout) {
        val = cdsi_read(dev, CDSI0_CDSIRX_LANE_STATUS_HS_OFFS);
        if (!(val & hs_mask))
            break;

        usleep(10);
    }

    if (val & hs_mask)
        printf("cdsi: lanes failed to switch to LP (0x%08x)\n", val);
    else
        printf("cdsi: lanes switched to LP (%u us)\n", timeout * 10);

    for (timeout = 0; timeout < CSI_RX_STOP_TIMEOUT; ++timeout) {
        val = cdsi_read(dev, CDSI0_CDSIRX_LANE_STATUS_LP_OFFS);
        if ((val & lp_mask) == lp_mask)
            break;

        usleep(10);
    }

    if ((val & lp_mask) != lp_mask)
        printf("cdsi: lanes failed to switch to LP-11 (0x%08x)\n", val);
    else
        printf("cdsi: lanes switched to LP-11 (%u us)\n", timeout * 10);

    for (timeout = 0; timeout < CSI_RX_STOP_TIMEOUT; ++timeout) {
        val = cdsi_read(dev, CDSI0_CDSIRX_INTERNAL_STAT_OFFS);
        if (!val)
            break;

        usleep(10);
    }

    if (val)
        printf("cdsi: CDSIRX failed to become idle (0x%08x)\n", val);
    else
        printf("cdsi: CDSIRX became idle (%u us)\n", timeout * 10);

    /* Stop CDSIRX and clear its internal state. */
    cdsi_write(dev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_STOP);
    cdsi_write(dev, CDSI0_CDSIRX_SYSTEM_INIT_OFFS,
               CDSI0_CDSIRX_SYSTEM_CLEAR_VAL);

    return 0;
}

uint32_t csi_rx_get_error(struct cdsi_dev *dev)
{
    uint32_t val;

    val = cdsi_read(dev, CDSI0_CDSIRX_ERR_STATUS_OFFS);
    printf("cdsi: RX_ERR_STATUS 0x%08x\n", val);

    return val;
}

struct cdsi_dev *csi_rx_open(unsigned int cdsi)
{
    return csi_initialize(cdsi, TSB_CDSI_RX);
}

void csi_rx_close(struct cdsi_dev *dev)
{
    csi_uninitialize(dev);
}

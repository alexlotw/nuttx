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
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <nuttx/gpio.h>
#include <nuttx/wqueue.h>
#include <arch/tsb/cdsi.h>
#include <arch/tsb/gpio.h>
#include <arch/board/csi.h>
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>

#define AL_TX_BRG_MODE_VAL                              0x00000092
#define AL_TX_BRG_WAIT_CYCLE_SET_VAL                    0x00100010
#define AL_TX_BRG_VSYNC_LINE_SET_VAL                    0x00000001
#define AL_TX_BRG_TYPE_SEL1_SET_VAL                     0x08FBFFFC
#define AL_TX_BRG_TYPE_SEL2_SET_VAL                     0xFF00C0E0
#define AL_TX_BRG_TYPE_MASK1_SET_VAL                    0x08FB00FC
#define AL_TX_BRG_TYPE_MASK2_SET_VAL                    0xFF00C0E0
#define AL_TX_BRG_PIC_COM_SET_VAL                       0x0000000F
#define AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_VAL            0x00000423
#define AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_VAL            0x00000423
#define AL_TX_BRG_PIC_COM_MAXFCNT_VAL                   0x00000000
#define AL_TX_BRG_PIC_COM_3DCM_PLDT_VAL                 0x00000000
#define AL_TX_BRG_PIC_COM_3DCM_LINE_VAL                 0x00000000
#define AL_TX_BRG_PIC_SYN_SET_VAL                       0x00000001
#define AL_TX_BRG_PIC_SYN_LINE_VAL                      0x00000000
#define AL_TX_BRG_STATUS_ENABLE_VAL                     0x000000FF
#define AL_TX_BRG_CDSITX_MODE_VAL                       0x00000001
#define AL_TX_BRG_UNIPRO_BYTESWAP_VAL                   0x76543210

#define CDSITX_INTERRUPT_FUNC_ENABLE_00_VAL             0x0000000F
#define CDSITX_INTERRUPT_FUNC_ENABLE_01_VAL             0x0000003F
#define CDSITX_INTERRUPT_FUNC_ENABLE_02_VAL             0x007F1F1F
#define CDSITX_INTERRUPT_FUNC_ENABLE_03_VAL             0x00000000

#define CDSITX_POWER_SW_VAL                             0x00000007

#define CDSITX_DPHY_RESET_VAL                           0x00000001

#define CDSITX_PLL_CONFIG_00_VAL                        0x00211033
#define CDSITX_PLL_CONFIG_02_VAL                        0x00001680
#define CDSITX_PLL_CONTROL_00_VAL                       0x00000001
#define CDSITX_PLL_CONTROL_01_VAL                       0x00000001

#define CDSITX_LANE_ENABLE_00_VAL                       0x00000013
#define CDSITX_LANE_ENABLE_01_VAL                       0x00000013
#define CDSITX_LANE_ENABLE_02_VAL                       0x00000013

#define CDSITX_VREG_CONFIG_VAL                          0x00000013
#define CDSITX_VREG_CONTROL_VAL                         0x00000013

#define CDSI0_CDSITX_LPRX_CALIB_CONFIG_VAL              0x0000000A
#define CDSI0_CDSITX_LPRX_CALIB_CONTROL_VAL             0x00000001

#define CDSITX_CSI2DSI_SELECT_VAL                       0x00000003

#define CDSITX_GLOBAL_TIMING_PARAM_00_VAL               0x00000000
#define CDSITX_GLOBAL_TIMING_PARAM_01_VAL               0x00022222
#define CDSITX_GLOBAL_TIMING_PARAM_02_VAL               0x00000000
#define CDSITX_GLOBAL_TIMING_PARAM_03_VAL               0x00000003
#define CDSITX_GLOBAL_TIMING_PARAM_04_VAL               0x00120004
#define CDSITX_GLOBAL_TIMING_PARAM_05_VAL               0x00080002
#define CDSITX_GLOBAL_TIMING_PARAM_06_VAL               0x00040005
#define CDSITX_GLOBAL_TIMING_PARAM_07_VAL               0x00090005
#define CDSITX_GLOBAL_TIMING_PARAM_08_VAL               0x00001F13
#define CDSITX_GLOBAL_TIMING_PARAM_09_VAL               0x00000002
#define CDSITX_GLOBAL_TIMING_PARAM_10_VAL               0x00030009

#define CDSITX_SIDEBAND_COUNT_CONFIG_00_VAL             0x00000794
#define CDSITX_SIDEBAND_COUNT_CONFIG_01_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_02_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_03_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_04_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_05_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_06_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_07_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_08_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_09_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_10_VAL             0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_11_VAL             0xFFFFFFFF

/* D-PHY Trimming */
#define CDSITX_SIDEBAND_CONFIG_00_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_01_VAL                   0x0000001F
#define CDSITX_SIDEBAND_CONFIG_02_VAL                   0x00022222
#define CDSITX_SIDEBAND_CONFIG_03_VAL                   0x15555555
#define CDSITX_SIDEBAND_CONFIG_04_VAL                   0x00022222

/* Sideband Configulation */
#define CDSITX_SIDEBAND_CONFIG_05_VAL                   0x0000000F
#define CDSITX_SIDEBAND_CONFIG_06_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_07_VAL                   0x00000080
#define CDSITX_SIDEBAND_CONFIG_08_VAL                   0x00000004
#define CDSITX_SIDEBAND_CONFIG_09_VAL                   0x00002001
#define CDSITX_SIDEBAND_CONFIG_10_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_11_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_12_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_13_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_14_VAL                   0x00000000
#define CDSITX_SIDEBAND_CONFIG_15_VAL                   0x00001800

/* PROVIDING CLOCKS */
#define CDSITX_PISO_ENABLE_VAL                          0x00000013
#define CDSITX_SIDEBAND_INIT_CONTROL_00_VAL             0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_01_VAL             0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_02_VAL             0x00000001

/* LINE INITIALIZATION */
#define CDSITX_SIDEBAND_INIT_CONTROL_03_VAL             0x00000001

/* RELEASE RESET TO LINK/APF */
#define CDSITX_LINK_RESET_VAL                           0x00000001
#define CDSITX_APF_RESET_VAL                            0x00000001

/* PROVIDE CLOCK FOR APF */
#define CDSITX_SIDEBAND_INIT_CONTROL_04_VAL             0x00000001

#define CDSI0_AL_TX_BRG_VPARAM_UPDATE_VAL               0X00000003
#define CDSI0_AL_TX_BRG_PIC_COM_START_VAL               0X00000001
#if defined(CONFIG_TSB_CHIP_REV_ES2)
#define CDSI0_AL_TX_BRG_ENABLE_VAL                      0x00000001
#endif

#define CDSI0_AL_RX_BRG_CSI_INFO_VAL                    0X00000003
#define CDSI0_AL_RX_BRG_CSI_DT0_VAL                     0X002A0001

#define CSI_TX_PRIORITY      (60)
#define CSI_TX_STACK_SIZE    (2048)

/* State of CSI */
#define CSI_STATE_STOP  0
#define CSI_STATE_START 1
#define CSI_STATE_BUSY  2

/* Command of CSI TX */
#define CSI_CMD_STOP    0
#define CSI_CMD_START   1

/**
 * @brief csi tx information
 */
struct csi_tx_info {
    /** data from AP */
    struct csi_control csi_ctrl;
    /** csi commmand */
    uint32_t csi_cmd;
    /** csi state */
    uint32_t csi_state;
    /** low level driver */
    struct cdsi_dev *csi_dev;
    /** semapher for csi tx operation */
    sem_t csi_sem;
    /** csi tx control thread */
    pthread_t csi_thread;
};

/**
 * @brief csi tx information
 */
struct csi_tx_info *info;

/**
 * @brief Set the registers in CSI controller to start transfer.
 *
 * @param dev Pointer to the structure of CSI device driver
 * @return None.
 */
static void csi_reg_start(struct cdsi_dev *dev)
{
    uint32_t rdata;

    printf("csi_reg_start + \n");

    /* for csi_stop use */
    info->csi_dev = dev;
    info->csi_state = CSI_STATE_BUSY;
    
    /* Set to Tx mode for CDSI */
    cdsi_write(dev, CDSI0_AL_TX_BRG_CDSITX_MODE_OFFS,
               AL_TX_BRG_CDSITX_MODE_VAL);

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    cdsi_write(dev, CDSI0_AL_TX_BRG_SOFT_RESET_OFFS,
               CDSI0_AL_TX_BRG_ENABLE_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_SYSCLK_ENABLE_OFFS,
               CDSI0_AL_TX_BRG_ENABLE_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_ENABLE_OFFS, CDSI0_AL_TX_BRG_ENABLE_VAL);
#endif

    /* Tx bridge setting */
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS, AL_TX_BRG_MODE_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_SET_OFFS,
               AL_TX_BRG_PIC_COM_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_OFFS,
               AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_OFFS,
               AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_MAXFCNT_OFFS,
               AL_TX_BRG_PIC_COM_MAXFCNT_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS,
               AL_TX_BRG_PIC_SYN_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_LINE_OFFS,
               AL_TX_BRG_PIC_SYN_LINE_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_PLDT_OFFS,
               AL_TX_BRG_PIC_COM_3DCM_PLDT_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_LINE_OFFS,
               AL_TX_BRG_PIC_COM_3DCM_LINE_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_OFFS,
               AL_TX_BRG_WAIT_CYCLE_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VSYNC_LINE_SET_OFFS,
               AL_TX_BRG_VSYNC_LINE_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL1_SET_OFFS,
               AL_TX_BRG_TYPE_SEL1_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL2_SET_OFFS,
               AL_TX_BRG_TYPE_SEL2_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK1_SET_OFFS,
               AL_TX_BRG_TYPE_MASK1_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK2_SET_OFFS,
               AL_TX_BRG_TYPE_MASK2_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_STATUS_ENABLE_OFFS,
               AL_TX_BRG_STATUS_ENABLE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_OFFS,
               AL_TX_BRG_UNIPRO_BYTESWAP_VAL);

    /* Tx bridge enable */
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS, 0x00000093);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_03_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_03_VAL);

    cdsi_write(dev, CDSI0_CDSITX_POWER_SW_OFFS, CDSITX_POWER_SW_VAL);
    /* D-PHY reset deassertion */
    cdsi_write(dev, CDSI0_CDSITX_DPHY_RESET_OFFS, CDSITX_DPHY_RESET_VAL);

    /* D-PHY PLL setting */
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_00_OFFS, CDSITX_PLL_CONFIG_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_02_OFFS, CDSITX_PLL_CONFIG_02_VAL);
    /* D-PHY PLL enable */
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_00_OFFS,
               CDSITX_PLL_CONTROL_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_01_OFFS,
               CDSITX_PLL_CONTROL_01_VAL);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_00_OFFS,
               CDSITX_LANE_ENABLE_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_01_OFFS,
               CDSITX_LANE_ENABLE_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_02_OFFS,
               CDSITX_LANE_ENABLE_02_VAL);

    /* D-PHY Vreg setting */
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONFIG_OFFS, CDSITX_VREG_CONFIG_VAL);
    /* D-PHY Vreg enable */
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONTROL_OFFS, CDSITX_VREG_CONTROL_VAL);

    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONFIG_OFFS,
               CDSI0_CDSITX_LPRX_CALIB_CONFIG_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONTROL_OFFS,
               CDSI0_CDSITX_LPRX_CALIB_CONTROL_VAL);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_CSI2DSI_SELECT_OFFS,
               CDSITX_CSI2DSI_SELECT_VAL);

    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_00_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_03_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_04_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_05_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_06_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_07_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_08_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_08_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_09_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_10_VAL);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_00_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_01_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_02_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_03_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_04_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_04_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_05_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_05_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_06_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_06_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_07_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_07_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_08_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_08_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_09_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_09_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_10_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_10_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_11_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_11_VAL);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_00_OFFS,
               CDSITX_SIDEBAND_CONFIG_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_01_OFFS,
               CDSITX_SIDEBAND_CONFIG_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_02_OFFS,
               CDSITX_SIDEBAND_CONFIG_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_03_OFFS,
               CDSITX_SIDEBAND_CONFIG_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_04_OFFS,
               CDSITX_SIDEBAND_CONFIG_04_VAL);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_05_OFFS,
               CDSITX_SIDEBAND_CONFIG_05_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_06_OFFS,
               CDSITX_SIDEBAND_CONFIG_06_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_07_OFFS,
               CDSITX_SIDEBAND_CONFIG_07_VAL);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_08_OFFS,
               CDSITX_SIDEBAND_CONFIG_08_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_09_OFFS,
               CDSITX_SIDEBAND_CONFIG_09_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_10_OFFS,
               CDSITX_SIDEBAND_CONFIG_10_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_11_OFFS,
               CDSITX_SIDEBAND_CONFIG_11_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_12_OFFS,
               CDSITX_SIDEBAND_CONFIG_12_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_13_OFFS,
               CDSITX_SIDEBAND_CONFIG_13_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_14_OFFS,
               CDSITX_SIDEBAND_CONFIG_14_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_15_OFFS,
               CDSITX_SIDEBAND_CONFIG_15_VAL);

    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS, rdata & 0xFFFFFFF7);

    /* Wait PLL lockup time */
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    while((rdata &
           CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_LOCKUPDONE_MASK) == 0x0) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }

    /* Wait D-PHY Vreg startup */
    while((rdata &
           CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_HSTXVREGRDY_MASK) == 0x0) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }

    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0x0000000A);

    cdsi_write(dev, CDSI0_CDSITX_PISO_ENABLE_OFFS, CDSITX_PISO_ENABLE_VAL);
    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_01_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_02_VAL);

    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0x00000001);
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS, rdata & 0xFFFFFFFE);
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_03_VAL);

    /* Wait D-PHY line initialization */
    while ((rdata &
            CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_LINEINITDONE_MASK) ==
           0x0) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_RX_BRG_MODE_OFFS, 0x00000003);

    /* Clear all interrupt statuses */
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_01_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_02_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_03_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_04_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_05_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS, 0xFFFFFFFF);

    /* Reset deassertion for LINK */
    cdsi_write(dev, CDSI0_CDSITX_LINK_RESET_OFFS, CDSITX_LINK_RESET_VAL);
    /* Reset deassertion for APF */
    cdsi_write(dev, CDSI0_CDSITX_APF_RESET_OFFS, CDSITX_APF_RESET_VAL);

    /* APF start */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_04_VAL);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_SET_OFFS, 0X0000000B);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS, 0X00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VPARAM_UPDATE_OFFS,
               CDSI0_AL_TX_BRG_VPARAM_UPDATE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_START_OFFS,
               CDSI0_AL_TX_BRG_PIC_COM_START_VAL);

    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_INFO_OFFS,
               CDSI0_AL_RX_BRG_CSI_INFO_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT0_OFFS, CDSI0_AL_RX_BRG_CSI_DT0_VAL);

    info->csi_state = CSI_STATE_START;
}

/**
 * @brief Set the registers in CSI controller to stop transfer.
 *
 * @param dev Pointer to the structure of CSI device driver
 * @return None.
 */
static void csi_reg_stop(struct cdsi_dev *dev)
{
    printf("csi_reg_stop + \n");

    info->csi_state = CSI_STATE_BUSY;

    /* Stop the Pixel I/F of CDSITX */

    /* Wait the stop of Pixel I/F */

    /* Wait until the transmission from UniPro to CDSITX is stopped */

    /* Disable the Tx bridge */

    /* Assert the Tx bridge reset */

    /* Stop the APF function */

    /* Disable the PPI function */

    /* Disable the clock distribution for LINK and APF */

    /* Assert the APF reset */

    /* Assert the LINK reset */

    /* Initialize the PPI */

    /* Enable the clock distribution to stop the clock lane of D-PHY */

    /* Wait for stop the clock lane of D-PHY */

    /* Disable the clock destribuiton to stop the clock lane of D-PHY */

    /* Disable the voltage requlator for D-PHY */

    /* Disable D-PHY functions */

    /* Disable PLL for CDSI */

    /* Clear the interrupt statuses */

    /* Release the Initialization signal for the PPI */

    info->csi_state = CSI_STATE_STOP;
}

/**
 * @brief Structure of CSI device driver ops
 */
struct camera_sensor cam_sensor = {
    .cdsi_sensor_init = csi_reg_start,
};

/**
 * @brief The CSI starting task for camera stream
 *
 * @param p_data Pointer to data for task process.
 * @return None.
 */
static void *csi_tx_thread(void *data)
{
    while (1) {
        sem_wait(&info->csi_sem);
        if (info->csi_cmd == CSI_CMD_START) {
            csi_initialize(&cam_sensor, TSB_CDSI1, TSB_CDSI_TX);
        } else {
            csi_reg_stop(info->csi_dev);
            csi_uninitialize(info->csi_dev);
        }
    }

    return NULL;
}

/**
 * @brief The CSI stopping task for camera stream
 *
 * @param p_data Pointer to data for task process.
 * @return None.
 */
int csi_tx_stop(void)
{
    if (info->csi_state == CSI_STATE_START) {
        info->csi_cmd = CSI_CMD_STOP;
        sem_post(&info->csi_sem);
    } else {
        return -EINVAL;
    }
    return 0;
}

/**
 * @brief Start the CSI Tx for camera stream
 *
 * @param csi_ctrl Pointer to structure of CSI control settings.
 * @return 0 on success, negative errno on error.
 */
int csi_tx_start(struct csi_control *csi_ctrl)
{
    if (info->csi_state == CSI_STATE_STOP) {
        info->csi_cmd = CSI_CMD_START;
        sem_post(&info->csi_sem);
    } else {
        return -EINVAL;
    }
    return 0;
}

/**
 * @brief Stop the CSI TX for camera stream
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_init(void)
{
    int ret;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    ret = sem_init(&info->csi_sem, 0, 0);
    if (ret) {
        lldbg("csi_tx create semapher failed \n");
        ret = -ret;
        goto err_free_mem;
    }
    
    ret = pthread_create(&info->csi_thread, NULL, csi_tx_thread, info);
    if  (ret) {
        lldbg("csi_tx create thread failed \n");
        ret = -ret;
        goto err_destroy_sem;
    }

    info->csi_state = CSI_STATE_STOP;

    return 0;

err_destroy_sem:
    sem_destroy(&info->csi_sem);
err_free_mem:
    free(info);
    
    return ret;
}

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

#ifndef __ARCH_ARM_TSB_TSB_UART_H
#define __ARCH_ARM_TSB_TSB_UART_H

/* Registers and address offset */
#define UA_RBR_THR_DLL      0x00
#define UA_IER_DLH          0x04
#define UA_FCR_IIR          0x08
#define UA_LCR              0x0C
#define UA_MCR              0x10
#define UA_LSR              0x14
#define UA_MSR              0x18
#define UA_SCR              0x1C
#define UA_FAR              0x70
#define UA_TFR              0x74
#define UA_RFW              0x78
#define UA_USR              0x7C
#define UA_TFL              0x80
#define UA_RFL              0x84
#define UA_HTX              0xA4
#define UA_DMASA            0xA8
#define UA_CPR              0xF4
#define UA_UCV              0xF8
#define UA_CTR              0xFC

/*
 * UA_IER_DLH
 */
/* DLAB = 0 */
#define UA_IER_ERBFI        BIT(0)  /* Enable Received Data Available */
#define UA_IER_ETBEI        BIT(1)  /* Enable Transmit Holding Register Empty */
#define UA_IER_ELSI         BIT(2)  /* Enable Receiver Line Status Interrupt */
#define UA_IER_EDSSI        BIT(3)  /* Enable Modem Status Interrupt */
#define UA_IER_PTIME        BIT(7)  /* Programmable THRE Interrupt Mode */

/* DLAB = 1 */
#define UA_DLH_MASK         0x7F    /* Divisor Latch register */

/*
 * UA_FCR_IIR
 */
#define UA_RX_FIF0_TRIGGER_SHIFT    6
#define UA_RX_FIFO_TRIGGER_1_CHAR   0x00
#define UA_RX_FIFO_TRIGGER_1_4      0x01
#define UA_RX_FIFO_TRIGGER_1_2      0x02
#define UA_RX_FIFO_TRIGGER_2        0x03

#define UA_TX_FIF0_TRIGGER_SHIFT    4
#define UA_TX_FIFO_TRIGGER_EMPTY    0x00
#define UA_TX_FIFO_TRIGGER_2_CHAR   0x01
#define UA_TX_FIFO_TRIGGER_1_4      0x02
#define UA_TX_FIFO_TRIGGER_1_2      0x03

/*
 * UA_FCR_IIR
 */
#define UA_INTERRUPT_ID_MASK        0x0f
#define UA_INTERRUPT_ID_MS          0x00    /* modem status */
#define UA_INTERRUPT_ID_NO          0x01    /* no interrupt pending */
#define UA_INTERRUPT_ID_TX          0x02    /* THR empty */
#define UA_INTERRUPT_ID_RX          0x04    /* received data available */
#define UA_INTERRUPT_ID_LS          0x06    /* receiver line status */
#define UA_INTERRUPT_ID_BUSY        0x07    /* busy detect */
#define UA_INTERRUPT_ID_TO          0x0C    /* character timeout */

#define UA_FIFO_ENABLE              BIT(0)
#define UA_RX_FIFO_RESET            BIT(1)
#define UA_TX_FIFO_RESET            BIT(2)

/*
 * UA_LCR
 */
#define UA_LCR_MASK                 0x7C
#define UA_LCR_STOP                 BIT(2)
#define UA_LCR_PEN                  BIT(3)
#define UA_LCR_EPS                  BIT(4)
#define UA_LCR_SP                   BIT(5)
#define UA_LCR_BREAK                BIT(6)

#define UA_DLAB                     BIT(7)

#define UA_DLS_MASK                 0x03
#define UA_DLS_5_BITS               0x00
#define UA_DLS_6_BITS               0x01
#define UA_DLS_7_BITS               0x02
#define UA_DLS_8_BITS               0x03

/*
 * UA_MCR
 */
#define UA_CTS                      BIT(0)
#define UA_RTS                      BIT(1)
#define UA_OUT1                     BIT(2)
#define UA_OUT2                     BIT(3)
#define UA_LOOP_BACK                BIT(4)
#define UA_AUTO_FLOW_ENABLE         BIT(5)

/*
 * UA_LSR
 */
#define UA_LSR_DR                   BIT(0)
#define UA_LSR_OE                   BIT(1)
#define UA_LSR_PE                   BIT(2)
#define UA_LSR_FE                   BIT(3)
#define UA_LSR_BI                   BIT(4)
#define UA_LSR_MASK                 0x1f

#define UA_LSR_THRE                 BIT(5)  /* Xmit Holding Register Empty */
#define UA_LSR_TEMT                 BIT(6)  /* Transmitter Empty bit */
#define UA_LSR_REF                  BIT(7)  /* Receive FIFO Error */

/*
 * UA_MSR
 */
#define UA_MSR_DCTS                 BIT(0)
#define UA_MSR_DDSR                 BIT(1)
#define UA_MSR_TERI                 BIT(2)
#define UA_MSR_DDCD                 BIT(3)
#define UA_MSR_CTS                  BIT(4)
#define UA_MSR_DSR                  BIT(5)
#define UA_MSR_RI                   BIT(6)
#define UA_MSR_DCD                  BIT(7)

/*
 * UA_USR
 */
#define UA_USR_BUSY                 BIT(0)  /* UART Busy */
#define UA_USR_TFNF                 BIT(1)  /* Transmit FIFO Not Full */
#define UA_USR_TFE                  BIT(2)  /* Transmit FIFO Empty */
#define UA_USR_RFNE                 BIT(3)  /* Receive FIFO Not Empty */
#define UA_USR_REF                  BIT(4)  /* Receive FIFO Full */

/* uart driver status flag */
#define TSB_UART_FLAG_OPEN          BIT(0)
#define TSB_UART_FLAG_XMIT          BIT(1)
#define TSB_UART_FLAG_RECV          BIT(2)

/* buffer structure */
struct uart_buffer
{
  volatile int16_t head;    /* Index to the head [IN] index in the buffer */
  volatile int16_t tail;    /* Index to the tail [OUT] index in the buffer */
  uint8_t          *buffer; /* Pointer to the allocated buffer memory */
  int16_t          size;    /* The allocated size of the buffer */
};


#endif /* __ARCH_ARM_TSB_TSB_UART_H */


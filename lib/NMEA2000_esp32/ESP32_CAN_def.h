/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __DRIVERS_CAN_REGDEF_H_
#define __DRIVERS_CAN_REGDEF_H_

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
		CAN_SPEED_100KBPS = 100,  /**< \brief CAN Node runs at 100kBit/s. */
		CAN_SPEED_125KBPS = 125,  /**< \brief CAN Node runs at 125kBit/s. */
		CAN_SPEED_250KBPS = 250,  /**< \brief CAN Node runs at 250kBit/s. */
		CAN_SPEED_500KBPS = 500,  /**< \brief CAN Node runs at 500kBit/s. */
		CAN_SPEED_800KBPS = 800,  /**< \brief CAN Node runs at 800kBit/s. */
		CAN_SPEED_1000KBPS = 1000 /**< \brief CAN Node runs at 1000kBit/s. */
	} CAN_speed_t;

	/**
	 * \brief CAN frame type (standard/extended)
	 */
	typedef enum
	{
		CAN_frame_std = 0, /**< Standard frame, using 11 bit identifer. */
		CAN_frame_ext = 1  /**< Extended frame, using 29 bit identifer. */
	} CAN_frame_format_t;

	/**
	 * \brief CAN RTR
	 */
	typedef enum
	{
		CAN_no_RTR = 0, /**< No RTR frame. */
		CAN_RTR = 1		/**< RTR frame. */
	} CAN_RTR_t;

	/** \brief Frame information record type */
	typedef union
	{
		uint32_t U; /**< \brief Unsigned access */
		struct
		{
			uint8_t DLC : 4;			   /**< \brief [3:0] DLC, Data length container */
			unsigned int unknown_2 : 2;	   /**< \brief \internal unknown */
			CAN_RTR_t RTR : 1;			   /**< \brief [6:6] RTR, Remote Transmission Request */
			CAN_frame_format_t FF : 1;	   /**< \brief [7:7] Frame Format, see# CAN_frame_format_t*/
			unsigned int reserved_24 : 24; /**< \brief \internal Reserved */
		} B;
	} CAN_FIR_t;

/** \brief Start address of CAN registers */
#define MODULE_CAN ((volatile CAN_Module_t *)DR_REG_TWAI_BASE)

/** \brief Get standard message ID */
#define _CAN_GET_STD_ID (((uint32_t)MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[0] << 3) | \
						 (MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[1] >> 5))

/** \brief Get extended message ID */
#define _CAN_GET_EXT_ID (((uint32_t)MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[0] << 21) | \
						 (MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[1] << 13) |           \
						 (MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[2] << 5) |            \
						 (MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[3] >> 3))

/** \brief Set standard message ID */
#define _CAN_SET_STD_ID(x)                                   \
	MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[0] = ((x) >> 3); \
	MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[1] = ((x) << 5);

/** \brief Set extended message ID */
#define _CAN_SET_EXT_ID(x) \
	//MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[0] = ((x) >> 21); \
	//MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[1] = ((x) >> 13); \
	//MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[2] = ((x) >> 5);  \
	//MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[3] = ((x) << 3);

	/** \brief Interrupt status register */
	typedef enum
	{
		__CAN_IRQ_RX = BIT(0),			 /**< \brief RX Interrupt */
		__CAN_IRQ_TX = BIT(1),			 /**< \brief TX Interrupt */
		__CAN_IRQ_ERR = BIT(2),			 /**< \brief Error Interrupt */
		__CAN_IRQ_DATA_OVERRUN = BIT(3), /**< \brief Date Overrun Interrupt */
		__CAN_IRQ_WAKEUP = BIT(4),		 /**< \brief Wakeup Interrupt */
		__CAN_IRQ_ERR_PASSIVE = BIT(5),	 /**< \brief Passive Error Interrupt */
		__CAN_IRQ_ARB_LOST = BIT(6),	 /**< \brief Arbitration lost interrupt */
		__CAN_IRQ_BUS_ERR = BIT(7),		 /**< \brief Bus error Interrupt */
	} __CAN_IRQ_t;

		/**
	 * CAN controller (SJA1000).
	 */
	typedef struct
	{
		union
		{
			struct
			{
				uint32_t CODE[4]; /**< \brief Acceptance Message ID */
				uint32_t MASK[4]; /**< \brief Acceptance Mask */
				uint32_t RESERVED2[5];
			} ACC; /**< \brief Acceptance filtering */
			struct
			{
				CAN_FIR_t FIR; /**< \brief Frame information record */
				union
				{
					struct
					{
						uint32_t ID[2];	  /**< \brief Standard frame message-ID*/
						uint32_t data[8]; /**< \brief Standard frame payload */
						uint32_t reserved[2];
					} STD; /**< \brief Standard frame format */
					struct
					{
						uint32_t ID[4];	  /**< \brief Extended frame message-ID*/
						uint32_t data[8]; /**< \brief Extended frame payload */
					} EXT;				  /**< \brief Extended frame format */
				} TX_RX;				  /**< \brief RX/TX interface */
			} FCTRL;					  /**< \brief Function control regs */
		} MBX_CTRL;						  /**< \brief Mailbox control */

		// Configuration and Control Registers
		//@from https://github.com/espressif/esp-idf/blob/master/components/soc/esp32c3/include/soc/twai_struct.h
		union
		{
			struct
			{
				uint32_t rm : 1;		 /* MOD.0 Reset Mode */
				uint32_t lom : 1;		 /* MOD.1 Listen Only Mode */
				uint32_t stm : 1;		 /* MOD.2 Self Test Mode */
				uint32_t afm : 1;		 /* MOD.3 Acceptance Filter Mode */
				uint32_t reserved4 : 28; /* Internal Reserved. MOD.4 Sleep Mode not supported */
			};
			uint32_t val;
		} mode_reg; /* Address 0x0000 */
		union
		{
			struct
			{
				uint32_t tr : 1;		 /* CMR.0 Transmission Request */
				uint32_t at : 1;		 /* CMR.1 Abort Transmission */
				uint32_t rrb : 1;		 /* CMR.2 Release Receive Buffer */
				uint32_t cdo : 1;		 /* CMR.3 Clear Data Overrun */
				uint32_t srr : 1;		 /* CMR.4 Self Reception Request */
				uint32_t reserved5 : 27; /* Internal Reserved */
			};
			uint32_t val;
		} command_reg; /* Address 0x0004 */
		union
		{
			struct
			{
				uint32_t rbs : 1;		 /* SR.0 Receive Buffer Status */
				uint32_t dos : 1;		 /* SR.1 Data Overrun Status */
				uint32_t tbs : 1;		 /* SR.2 Transmit Buffer Status */
				uint32_t tcs : 1;		 /* SR.3 Transmission Complete Status */
				uint32_t rs : 1;		 /* SR.4 Receive Status */
				uint32_t ts : 1;		 /* SR.5 Transmit Status */
				uint32_t es : 1;		 /* SR.6 Error Status */
				uint32_t bs : 1;		 /* SR.7 Bus Status */
				uint32_t ms : 1;		 /* SR.8 Miss Status */
				uint32_t reserved9 : 23; /* Internal Reserved */
			};
			uint32_t val;
		} status_reg; /* Address 0x0008 */
		union
		{
			struct
			{
				uint32_t ri : 1;		 /* IR.0 Receive Interrupt */
				uint32_t ti : 1;		 /* IR.1 Transmit Interrupt */
				uint32_t ei : 1;		 /* IR.2 Error Interrupt */
				uint32_t doi : 1;		 /* IR.3 Data Overrun Interrupt */
				uint32_t reserved4 : 1;	 /* Internal Reserved (Wake-up not supported) */
				uint32_t epi : 1;		 /* IR.5 Error Passive Interrupt */
				uint32_t ali : 1;		 /* IR.6 Arbitration Lost Interrupt */
				uint32_t bei : 1;		 /* IR.7 Bus Error Interrupt */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} interrupt_reg; /* Address 0x000C */
		union
		{
			struct
			{
				uint32_t rie : 1;		 /* IER.0 Receive Interrupt Enable */
				uint32_t tie : 1;		 /* IER.1 Transmit Interrupt Enable */
				uint32_t eie : 1;		 /* IER.2 Error Interrupt Enable */
				uint32_t doie : 1;		 /* IER.3 Data Overrun Interrupt Enable */
				uint32_t reserved4 : 1;	 /* Internal Reserved (Wake-up not supported) */
				uint32_t epie : 1;		 /* IER.5 Error Passive Interrupt Enable */
				uint32_t alie : 1;		 /* IER.6 Arbitration Lost Interrupt Enable */
				uint32_t beie : 1;		 /* IER.7 Bus Error Interrupt Enable */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} interrupt_enable_reg; /* Address 0x0010 */
		uint32_t reserved_14;
		union
		{
			struct
			{
				uint32_t brp : 13;		  /* BTR0[12:0] Baud Rate Prescaler */
				uint32_t reserved13 : 1;  /* Internal Reserved */
				uint32_t sjw : 2;		  /* BTR0[15:14] Synchronization Jump Width*/
				uint32_t reserved16 : 16; /* Internal Reserved */
			};
			uint32_t val;
		} bus_timing_0_reg; /* Address 0x0018 */
		union
		{
			struct
			{
				uint32_t tseg1 : 4;		 /* BTR1[3:0] Timing Segment 1 */
				uint32_t tseg2 : 3;		 /* BTR1[6:4] Timing Segment 2 */
				uint32_t sam : 1;		 /* BTR1.7 Sampling*/
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} bus_timing_1_reg;	  /* Address 0x001C */
		uint32_t reserved_20; /* Address 0x0020 (Output control not supported) */
		uint32_t reserved_24; /* Address 0x0024 (Test Register not supported) */
		uint32_t reserved_28; /* Address 0x0028 */

		// Capture and Counter Registers
		union
		{
			struct
			{
				uint32_t alc : 5;		 /* ALC[4:0] Arbitration lost capture */
				uint32_t reserved5 : 27; /* Internal Reserved */
			};
			uint32_t val;
		} arbitration_lost_captue_reg; /* Address 0x002C */
		union
		{
			struct
			{
				uint32_t seg : 5;		 /* ECC[4:0] Error Code Segment 0 to 5 */
				uint32_t dir : 1;		 /* ECC.5 Error Direction (TX/RX) */
				uint32_t errc : 2;		 /* ECC[7:6] Error Code */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} error_code_capture_reg; /* Address 0x0030 */
		union
		{
			struct
			{
				uint32_t ewl : 8;		 /* EWL[7:0] Error Warning Limit */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} error_warning_limit_reg; /* Address 0x0034 */
		union
		{
			struct
			{
				uint32_t rxerr : 8;		 /* RXERR[7:0] Receive Error Counter */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} rx_error_counter_reg; /* Address 0x0038 */
		union
		{
			struct
			{
				uint32_t txerr : 8;		 /* TXERR[7:0] Receive Error Counter */
				uint32_t reserved8 : 24; /* Internal Reserved */
			};
			uint32_t val;
		} tx_error_counter_reg; /* Address 0x003C */

		// Shared Registers (TX Buff/RX Buff/Acc Filter)
		union
		{
			struct
			{
				union
				{
					struct
					{
						uint32_t byte : 8;		 /* ACRx[7:0] Acceptance Code */
						uint32_t reserved8 : 24; /* Internal Reserved */
					};
					uint32_t val;
				} acr[4];
				union
				{
					struct
					{
						uint32_t byte : 8;		 /* AMRx[7:0] Acceptance Mask */
						uint32_t reserved8 : 24; /* Internal Reserved */
					};
					uint32_t val;
				} amr[4];
				uint32_t reserved_60;
				uint32_t reserved_64;
				uint32_t reserved_68;
				uint32_t reserved_6c;
				uint32_t reserved_70;
			} acceptance_filter;
			union
			{
				struct
				{
					uint32_t byte : 8;		  /* TX/RX Byte X [7:0] */
					uint32_t reserved24 : 24; /* Internal Reserved */
				};
				uint32_t val;
			} tx_rx_buffer[13];
		}; /* Address 0x0040 - 0x0070 */

		// Misc Registers
		union
		{
			struct
			{
				uint32_t rmc : 7;		 /* RMC[6:0] RX Message Counter */
				uint32_t reserved7 : 25; /* Internal Reserved */
			};
			uint32_t val;
		} rx_message_counter_reg; /* Address 0x0074 */
		uint32_t reserved_78;	  /* Address 0x0078 (RX Buffer Start Address not supported) */
		union
		{
			struct
			{
				uint32_t cd : 8;		 /* CDR[7:0] CLKOUT frequency selector based of fOSC */
				uint32_t co : 1;		 /* CDR.8 CLKOUT enable/disable */
				uint32_t reserved9 : 23; /* Internal Reserved  */
			};
			uint32_t val;
		} clock_divider_reg; /* Address 0x007C */
	} CAN_Module_t;

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_CAN_REGDEF_H_ */

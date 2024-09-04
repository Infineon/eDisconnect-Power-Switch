/*
@file    EVE_target.h
@brief   target specific includes, definitions and functions
@version 0.1
@date    2022-02-04
@author  Rene Santeler @ MCI 2022

@section LICENSE

MIT License
Copyright (c) 2022 Rene Santeler
 */


#ifndef EVE_TARGET_H_
#define EVE_TARGET_H_

/* While the following lines make things a lot easier like automatically compiling the code for the target you are compiling for, */
/* a few things are expected to be taken care of beforehand. */
/* - setting up the Pins and their functions */
/* - setting up Current measurement via ADC */
/* - setting up the Interrupt routine in main code */
/* - setting up target specific components (see target definition below) */
/* - setting up the SPI */

#pragma once


	#if defined (__GNUC__)

		//#if defined (XMC1302_T028x0200)
		#if defined (XMC1302_T028x0200)
			// Target definition for XMC4500 on BMS_SystemBoard
			// 		APP				Name					Comment
			//		SPI_MASTER		SPI_DRIVER				Main SPI_MASTER APP. Config: Full_Duplex, bus speed = 4MHz, TX/RX mode = direct, number of SS lines = 1, word = 8, Frame = 16, leading/trailing delay = 1, enable frame end mode, Mode: MSB first, [low if inactive, TX on rising, RX on falling], FIFO enabled 16bit
			// 		DIGITAL_IO  	IO_DRIVER_ENABLE		Enable pin APP
			// 		DIGITAL_IO 		IO_DRIVER_SAVESTATEEN	Save state enable pin APP
			// 		PIN_INTERRUPT 	PIN_INT_DRIVER			Interrupt pin APP (connected to 2ED INT pin)
			// Note: Use Dave Apps for IO/SPI and name accordingly!
			#include <DAVE.h>


			// A write debug function must be implemented externally!
			extern void debugPrintf(const char *p_frm, ...);
			#define DEBUGPRINTF(p_frm, ...) debugPrintf(p_frm, ##__VA_ARGS__) // redefinition needed to allow for multiple targets

			// A delay function must be implemented externally!
			extern void delay_ms(uint32_t ms);
			#define DELAY_MS(ms) delay_ms(ms) // redefinition needed to allow for multiple targets



			static inline void enable_setHIGH(void) {
				DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
			}

			static inline void enable_setLOW(void) {
				DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
			}

			static inline uint8_t isEnabled(void) {
				return (uint8_t)DIGITAL_IO_GetInput(&IO_DRIVER_ENABLE);
			}


			static inline void saveStateEn_setHIGH(void) {
				// Save state is disabled!
				DIGITAL_IO_SetOutputHigh(&IO_DRIVER_SAVESTATEEN);
			}

			static inline void saveStateEn_setLOW(void) {
				// Save state is enabled!
				DIGITAL_IO_SetOutputLow(&IO_DRIVER_SAVESTATEEN);
			}

			static inline uint8_t getSaveState(void) {
				return (uint8_t)DIGITAL_IO_GetInput(&IO_DRIVER_SAVESTATEEN);
			}



			static inline void enable_interruptPin(void) {
				PIN_INTERRUPT_SetEdgeSensitivity(&PIN_INT_DRIVER, PIN_INTERRUPT_EDGE_RISING);
				// PIN_INTERRUPT_Enable(&PIN_INT);
			}

			static inline void disable_interruptPin(void) {
				PIN_INTERRUPT_SetEdgeSensitivity(&PIN_INT_DRIVER, PIN_INTERRUPT_EDGE_NONE);
				// PIN_INTERRUPT_Disable(&PIN_INT);
			}


			//****************************************************************************
			// spiRead
			//
			// sends read command for register address
			// gets data of previous SPI access
			//****************************************************************************
			static inline void spiRead(uint8_t adr, uint8_t *data)
			{
			    uint8_t tx_buf[2]; // transmit buffer
			    uint8_t rx_buf[2]; // receive buffer

			    *data = 0;
			   // uint8_t *data2 = 0;

			    // read only if driver is enabled (ENABLE pin high)
			    if(isEnabled())
			    {
			        if(adr >=0 && adr <= 10)
			        {
			            tx_buf[0] = adr; // R/W bit is 0 anyway with address <= 10
			            tx_buf[1] = 0;   // data is "don't care"


			            SPI_MASTER_EnableSlaveSelectSignal(&SPI_DRIVER, SPI_MASTER_SS_SIGNAL_0);

			            SPI_MASTER_Transfer(&SPI_DRIVER, tx_buf, rx_buf, 2);

			            SPI_MASTER_DisableSlaveSelectSignal(&SPI_DRIVER);




			            *data = rx_buf[1];
			           // *data = rx_buf[1];
			        }
			        else
			        {
			        	DEBUGPRINTF("[ERROR]: Can't read register - driver is disabled");
			        	//XMC_DEBUG("[ERROR]: Can't read register - driver is disabled");
			        }
			    }
			}

			//****************************************************************************
			// spiWrite
			//
			// writes 8 bit data to register address
			//****************************************************************************
			static inline void spiWrite(uint8_t adr, uint8_t data)
			{
			    uint8_t tx_buf[2]; // transmit buffer
			    uint8_t rx_buf[2]; // receive buffer

			    // write only if driver is enabled (ENABLE pin high)
			    if(isEnabled())
			    {

			        if(adr >=0 && adr <= 10)
			        {
			            tx_buf[0] = adr;
			            tx_buf[0] |= (1<<7); // set R/W bit to "1"
			            tx_buf[1] = data;

			            //XMC_SPI_CH_EnableSOF(&SPI_DRIVER);
			            SPI_MASTER_EnableSlaveSelectSignal(&SPI_DRIVER, SPI_MASTER_SS_SIGNAL_0);

			            //SPI_MASTER_Transmit(&SPI_DRIVER, tx_buf, 2);
			            SPI_MASTER_Transfer(&SPI_DRIVER, tx_buf, rx_buf, 2);

			            SPI_MASTER_DisableSlaveSelectSignal(&SPI_DRIVER);

			           // XMC_SPI_CH_DisableSOF(&SPI_DRIVER);




			        }
			    }
			    else
			    {
			    //	DEBUGPRINTF("\r\n"); // new line   //***SOE
			     //   DEBUGPRINTF("[ERROR]: Can't write register - driver is disabled");   **SOE
			    	//XMC_DEBUG("\r\n");
			    	//XMC_DEBUG("[ERROR]: Can't write register - driver is disabled");
			    }
			}


			//static inline void spi_transmit(uint8_t data) {
			//	// Transmit a byte and wait till its finished
			//	SPI_MASTER_Transmit(&SPI_MASTER_0, &data, sizeof(data));
			//	while (XMC_SPI_CH_GetStatusFlag(SPI_MASTER_0.channel) & XMC_SPI_CH_STATUS_FLAG_MSLS) __NOP();
			//}
			//
			//static inline uint8_t spi_receive(uint8_t data) {
			//	// Receive a byte and wait till its finished
			//	ReadData = 42;
			//	SPI_MASTER_Receive(&SPI_MASTER_0, &ReadData, sizeof(ReadData)); //SPI_ReceiveByte(data);
			//	while(SPI_MASTER_0.runtime->rx_busy){}
			//
			//	// Return byte
			//	return (uint8_t) ReadData;
			//}

		#endif /* XMC1302_T038x0200 */

	#endif /* __GNUC__ */

#endif /* TARGET_H_ */

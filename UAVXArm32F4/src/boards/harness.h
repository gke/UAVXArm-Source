// DO NOT FORMAT  DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT
// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/


#ifndef _harness_h
#define _harness_h

// Ugly but OK
#define PA0 {GPIOA,GPIO_Pin_0,GPIO_PinSource0}
#define PA1 {GPIOA,GPIO_Pin_1,GPIO_PinSource1}
#define PA2 {GPIOA,GPIO_Pin_2,GPIO_PinSource2}
#define PA3 {GPIOA,GPIO_Pin_3,GPIO_PinSource3}
#define PA4 {GPIOA,GPIO_Pin_4,GPIO_PinSource4}
#define PA5 {GPIOA,GPIO_Pin_5,GPIO_PinSource5}
#define PA6 {GPIOA,GPIO_Pin_6,GPIO_PinSource6}
#define PA7 {GPIOA,GPIO_Pin_7,GPIO_PinSource7}
#define PA8 {GPIOA,GPIO_Pin_8,GPIO_PinSource8}
#define PA9 {GPIOA,GPIO_Pin_9,GPIO_PinSource9}
#define PA10 {GPIOA,GPIO_Pin_10,GPIO_PinSource10}
#define PA11 {GPIOA,GPIO_Pin_11,GPIO_PinSource11}
#define PA12 {GPIOA,GPIO_Pin_12,GPIO_PinSource12}
#define PA13 {GPIOA,GPIO_Pin_13,GPIO_PinSource13}
#define PA14 {GPIOA,GPIO_Pin_14,GPIO_PinSource14}
#define PA15 {GPIOA,GPIO_Pin_15,GPIO_PinSource15}

#define PB0 {GPIOB,GPIO_Pin_0,GPIO_PinSource0}
#define PB1 {GPIOB,GPIO_Pin_1,GPIO_PinSource1}
#define PB2 {GPIOB,GPIO_Pin_2,GPIO_PinSource2}
#define PB3 {GPIOB,GPIO_Pin_3,GPIO_PinSource3}
#define PB4 {GPIOB,GPIO_Pin_4,GPIO_PinSource4}
#define PB5 {GPIOB,GPIO_Pin_5,GPIO_PinSource5}
#define PB6 {GPIOB,GPIO_Pin_6,GPIO_PinSource6}
#define PB7 {GPIOB,GPIO_Pin_7,GPIO_PinSource7}
#define PB8 {GPIOB,GPIO_Pin_8,GPIO_PinSource8}
#define PB9 {GPIOB,GPIO_Pin_9,GPIO_PinSource9}
#define PB10 {GPIOB,GPIO_Pin_10,GPIO_PinSource10}
#define PB11 {GPIOB,GPIO_Pin_11,GPIO_PinSource11}
#define PB12 {GPIOB,GPIO_Pin_12,GPIO_PinSource12}
#define PB13 {GPIOB,GPIO_Pin_13,GPIO_PinSource13}
#define PB14 {GPIOB,GPIO_Pin_14,GPIO_PinSource14}
#define PB15 {GPIOB,GPIO_Pin_15,GPIO_PinSource15}

#define PC0 {GPIOC,GPIO_Pin_0,GPIO_PinSource0}
#define PC1 {GPIOC,GPIO_Pin_1,GPIO_PinSource1}
#define PC2 {GPIOC,GPIO_Pin_2,GPIO_PinSource2}
#define PC3 {GPIOC,GPIO_Pin_3,GPIO_PinSource3}
#define PC4 {GPIOC,GPIO_Pin_4,GPIO_PinSource4}
#define PC5 {GPIOC,GPIO_Pin_5,GPIO_PinSource5}
#define PC6 {GPIOC,GPIO_Pin_6,GPIO_PinSource6}
#define PC7 {GPIOC,GPIO_Pin_7,GPIO_PinSource7}
#define PC8 {GPIOC,GPIO_Pin_8,GPIO_PinSource8}
#define PC9 {GPIOC,GPIO_Pin_9,GPIO_PinSource9}
#define PC10 {GPIOC,GPIO_Pin_10,GPIO_PinSource10}
#define PC11 {GPIOC,GPIO_Pin_11,GPIO_PinSource11}
#define PC12 {GPIOC,GPIO_Pin_12,GPIO_PinSource12}
#define PC13 {GPIOC,GPIO_Pin_13,GPIO_PinSource13}
#define PC14 {GPIOC,GPIO_Pin_14,GPIO_PinSource14}
#define PC15 {GPIOC,GPIO_Pin_15,GPIO_PinSource15}

#define PD0 {GPIOD,GPIO_Pin_0,GPIO_PinSource0}
#define PD1 {GPIOD,GPIO_Pin_1,GPIO_PinSource1}
#define PD2 {GPIOD,GPIO_Pin_2,GPIO_PinSource2}
#define PD3 {GPIOD,GPIO_Pin_3,GPIO_PinSource3}
#define PD4 {GPIOD,GPIO_Pin_4,GPIO_PinSource4}
#define PD5 {GPIOD,GPIO_Pin_5,GPIO_PinSource5}
#define PD6 {GPIOD,GPIO_Pin_6,GPIO_PinSource6}
#define PD7 {GPIOD,GPIO_Pin_7,GPIO_PinSource7}
#define PD8 {GPIOD,GPIO_Pin_8,GPIO_PinSource8}
#define PD9 {GPIOD,GPIO_Pin_9,GPIO_PinSource9}
#define PD10 {GPIOD,GPIO_Pin_10,GPIO_PinSource10}
#define PD11 {GPIOD,GPIO_Pin_11,GPIO_PinSource11}
#define PD12 {GPIOD,GPIO_Pin_12,GPIO_PinSource12}
#define PD13 {GPIOD,GPIO_Pin_13,GPIO_PinSource13}
#define PD14 {GPIOD,GPIO_Pin_14,GPIO_PinSource14}
#define PD15 {GPIOD,GPIO_Pin_15,GPIO_PinSource15}

#define PE0 {GPIOE,GPIO_Pin_0,GPIO_PinSource0}
#define PE1 {GPIOE,GPIO_Pin_1,GPIO_PinSource1}
#define PE2 {GPIOE,GPIO_Pin_2,GPIO_PinSource2}
#define PE3 {GPIOE,GPIO_Pin_3,GPIO_PinSource3}
#define PE4 {GPIOE,GPIO_Pin_4,GPIO_PinSource4}
#define PE5 {GPIOE,GPIO_Pin_5,GPIO_PinSource5}
#define PE6 {GPIOE,GPIO_Pin_6,GPIO_PinSource6}
#define PE7 {GPIOE,GPIO_Pin_7,GPIO_PinSource7}
#define PE8 {GPIOE,GPIO_Pin_8,GPIO_PinSource8}
#define PE9 {GPIOE,GPIO_Pin_9,GPIO_PinSource9}
#define PE10 {GPIOE,GPIO_Pin_10,GPIO_PinSource10}
#define PE11 {GPIOE,GPIO_Pin_11,GPIO_PinSource11}
#define PE12 {GPIOE,GPIO_Pin_12,GPIO_PinSource12}
#define PE13 {GPIOE,GPIO_Pin_13,GPIO_PinSource13}
#define PE14 {GPIOE,GPIO_Pin_14,GPIO_PinSource14}
#define PE15 {GPIOE,GPIO_Pin_15,GPIO_PinSource15}

#define PWMIn_2_1 {true,TIM2,TIM_Channel_1,TIM_IT_CC1,0,GPIO_AF_TIM2},{false,},TIM2_IRQn
#define PWMIn_2_2 {true,TIM2,TIM_Channel_2,TIM_IT_CC2,0,GPIO_AF_TIM2},{false,},TIM2_IRQn
#define PWMIn_2_3 {true,TIM2,TIM_Channel_3,TIM_IT_CC3,0,GPIO_AF_TIM2},{false,},TIM2_IRQn
#define PWMIn_2_4 {true,TIM2,TIM_Channel_4,TIM_IT_CC4,0,GPIO_AF_TIM2},{false,},TIM2_IRQn

#define PWMIn_3_1 {true,TIM3,TIM_Channel_1,TIM_IT_CC1,0,GPIO_AF_TIM3},{false,},TIM3_IRQn
#define PWMIn_3_2 {true,TIM3,TIM_Channel_2,TIM_IT_CC2,0,GPIO_AF_TIM3},{false,},TIM3_IRQn
#define PWMIn_3_3 {true,TIM3,TIM_Channel_3,TIM_IT_CC3,0,GPIO_AF_TIM3},{false,},TIM3_IRQn
#define PWMIn_3_4 {true,TIM3,TIM_Channel_4,TIM_IT_CC4,0,GPIO_AF_TIM3},{false,},TIM3_IRQn

#define PWMIn_4_1 {true,TIM4,TIM_Channel_1,TIM_IT_CC1,0,GPIO_AF_TIM4},{false,},TIM4_IRQn
#define PWMIn_4_2 {true,TIM4,TIM_Channel_2,TIM_IT_CC2,0,GPIO_AF_TIM4},{false,},TIM4_IRQn
#define PWMIn_4_3 {true,TIM4,TIM_Channel_3,TIM_IT_CC3,0,GPIO_AF_TIM4},{false,},TIM4_IRQn
#define PWMIn_4_4 {true,TIM4,TIM_Channel_4,TIM_IT_CC4,0,GPIO_AF_TIM4},{false,},TIM4_IRQn

#define PWMOut_1_1 {true,TIM1,TIM_Channel_1,0,&(TIM1->CCR1),GPIO_AF_TIM1}
#define PWMOut_1_2 {true,TIM1,TIM_Channel_2,0,&(TIM1->CCR2),GPIO_AF_TIM1}
#define PWMOut_1_3 {true,TIM1,TIM_Channel_3,0,&(TIM1->CCR3),GPIO_AF_TIM1}
#define PWMOut_1_4 {true,TIM1,TIM_Channel_4,0,&(TIM1->CCR4),GPIO_AF_TIM1}

#define PWMOut_2_1 {true,TIM2,TIM_Channel_1,0,&(TIM2->CCR1),GPIO_AF_TIM2}
#define PWMOut_2_2 {true,TIM2,TIM_Channel_2,0,&(TIM2->CCR2),GPIO_AF_TIM2}
#define PWMOut_2_3 {true,TIM2,TIM_Channel_2,0,&(TIM2->CCR3),GPIO_AF_TIM2}
#define PWMOut_2_4 {true,TIM2,TIM_Channel_4,0,&(TIM2->CCR4),GPIO_AF_TIM2}

#define PWMOut_3_1 {true,TIM3,TIM_Channel_1,0,&(TIM3->CCR1),GPIO_AF_TIM3}
#define PWMOut_3_2 {true,TIM3,TIM_Channel_2,0,&(TIM3->CCR2),GPIO_AF_TIM3}
#define PWMOut_3_3 {true,TIM3,TIM_Channel_3,0,&(TIM3->CCR3),GPIO_AF_TIM3}
#define PWMOut_3_4 {true,TIM3,TIM_Channel_4,0,&(TIM3->CCR4),GPIO_AF_TIM3}

#define PWMOut_4_1 {true,TIM4,TIM_Channel_1,0,&(TIM4->CCR1),GPIO_AF_TIM4}
#define PWMOut_4_2 {true,TIM4,TIM_Channel_2,0,&(TIM4->CCR2),GPIO_AF_TIM4}
#define PWMOut_4_3 {true,TIM4,TIM_Channel_3,0,&(TIM4->CCR3),GPIO_AF_TIM4}
#define PWMOut_4_4 {true,TIM4,TIM_Channel_4,0,&(TIM4->CCR4),GPIO_AF_TIM4}

#define PWMOut_5_1 {true,TIM5,TIM_Channel_1,0,&(TIM5->CCR1),GPIO_AF_TIM5}
#define PWMOut_5_2 {true,TIM5,TIM_Channel_2,0,&(TIM5->CCR2),GPIO_AF_TIM5}
#define PWMOut_5_3 {true,TIM5,TIM_Channel_3,0,&(TIM5->CCR3),GPIO_AF_TIM5}
#define PWMOut_5_5 {true,TIM5,TIM_Channel_5,0,&(TIM5->CCR4),GPIO_AF_TIM5}

#define PWMOut_8_1 {true,TIM8,TIM_Channel_1,0,&(TIM8->CCR1),GPIO_AF_TIM8}
#define PWMOut_8_2 {true,TIM8,TIM_Channel_2,0,&(TIM8->CCR2),GPIO_AF_TIM8}
#define PWMOut_8_3 {true,TIM8,TIM_Channel_3,0,&(TIM8->CCR3),GPIO_AF_TIM8}
#define PWMOut_8_4 {true,TIM8,TIM_Channel_4,0,&(TIM8->CCR4),GPIO_AF_TIM8}

#define PWMOut_9_1 {true,TIM9,TIM_Channel_1,0,&(TIM9->CCR1),GPIO_AF_TIM9}
#define PWMOut_9_2 {true,TIM9,TIM_Channel_2,0,&(TIM9->CCR2),GPIO_AF_TIM9}
#define PWMOut_9_3 {true,TIM9,TIM_Channel_3,0,&(TIM9->CCR3),GPIO_AF_TIM9}
#define PWMOut_9_4 {true,TIM9,TIM_Channel_4,0,&(TIM9->CCR4),GPIO_AF_TIM9}

#define PWMOut_8_1CC {true,TIM8,TIM_Channel_1,TIM_DMA_CC1,&(TIM8->CCR1),GPIO_AF_TIM8}

#define PWMPinConfig GPIO_Mode_AF,GPIO_OType_PP,GPIO_PuPd_UP
#define LEDPinConfig GPIO_Mode_OUT,GPIO_OType_PP,GPIO_PuPd_NOPULL
#define OutPinConfig GPIO_Mode_OUT,GPIO_OType_PP,GPIO_PuPd_UP // ???
#define InpPinConfig GPIO_Mode_IN,GPIO_OType_PP,GPIO_PuPd_UP

#define USART1Config GPIO_AF_USART1,USART1_IRQn,DMA_Channel_4,DMA2_Stream7,DMA2_Stream7_IRQn,DMA2_Stream5
#define USART2Config GPIO_AF_USART2,USART2_IRQn,DMA_Channel_4,DMA1_Stream6,DMA1_Stream6_IRQn,DMA1_Stream5

#define USART3Config GPIO_AF_USART3,USART3_IRQn,DMA_Channel_4,DMA1_Stream3,DMA1_Stream3_IRQn,DMA1_Stream1
#define UART4Config  GPIO_AF_UART4, UART4_IRQn, DMA_Channel_4,DMA1_Stream4,DMA1_Stream4_IRQn,DMA1_Stream2

// More thought needed!
#define DMA2_0_0_noIRQn {true,DMA_Channel_0,DMA2_Stream0}
//#define DMA1_4_3_IRQn DMA_Channel_4,DMA1_Stream3,DMA1_Stream3_IRQn
//#define DMA2_0_2_IRQn {true,DMA_Channel_0,DMA2_Stream2},DMA2_Stream2_IRQn

#define MAX_RC_INPUTS 1 // 8
#define MAX_PWM_OUTPUTS 10

// Drives
#define PWM_PS 					TIMER_PS	// 1MHz
#define PWM_WIDTH				1000 // 1ms pulse width
#define PWM_MIN					PWM_WIDTH
#define PWM_MAX					(2000) // must preserve a synchronisation gap for ESCs
#define PWM_PERIOD    			(1000000L/450) // could go to 490Hz?
#define PWM_PS_SYNC 			TIMER_PS
#define PWM_WIDTH_SYNC			PWM_WIDTH // 1ms pulse width
#define PWM_MIN_SYNC			PWM_WIDTH_SYNC
#define PWM_MAX_SYNC			PWM_MAX // must preserve a synchronisation gap for ESCs
#define PWM_PERIOD_SYNC    		(PWM_MAX_SYNC*2)

#define PWM_PS_SYNC_DIV8		(TIMER_PS/12)	// 12MHz
#define PWM_WIDTH_SYNC_DIV8		((PWM_MIN*3)>>1)
#define PWM_MIN_SYNC_DIV8		PWM_WIDTH_SYNC_DIV8
#define PWM_MAX_SYNC_DIV8		((PWM_MAX*3)>>1)
#define PWM_PERIOD_SYNC_DIV8	(PWM_MAX_SYNC_DIV8*2)

#define DC_DRIVE_FREQ_HZ 		12000 // KHz 1000, 2000, 4000, 8000, 12000, 24000, 42000, 84000, 168000
#define DC_DRIVE_PS				(DC_DRIVE_FREQ_HZ/1000) // increase resolution
#define PWM_PS_DC				(TIMER_PS/DC_DRIVE_PS)
#define PWM_WIDTH_DC			1 // so we get a marker for logic analysers - can be zero
#define PWM_MIN_DC				PWM_WIDTH_DC
#define PWM_MAX_DC				(1000-PWM_MIN_DC)
#define PWM_PERIOD_DC    		((1000000*DC_DRIVE_PS)/DC_DRIVE_FREQ_HZ)

// Servos

#define PWM_PERIOD_DIGITAL  (1000000/200) // pulse period for digital servo
#define PWM_PERIOD_ANALOG  (22500) //1000000/50) // pulse period for analog servo
#ifdef USE_DIGITAL_SERVOS
#define PWM_PERIOD_SERVO PWM_PERIOD_DIGITAL
#else
#define PWM_PERIOD_SERVO PWM_PERIOD_ANALOG
#endif

#define PWM_MIN_SERVO	(800)
#define PWM_WIDTH_SERVO	(1000) // 1ms pulse width
#define PWM_NEUTRAL		(1500)
#define PWM_MAX_SERVO	(2200)

enum GPIOSelectors {
	BeeperSel,
	ArmedSel,
	LandingSel,
	Aux1Sel,
	Aux2Sel,
	WPMissionOrProbeSel,
	InverterSel,
	MPU6XXXIntSel,
	HMC5XXXRdySel,
	MAX_GPIO_PINS
};

enum ADCSelectors {
	RangefinderAnalogSel, BattCurrentAnalogSel, BattVoltsAnalogSel,
	// Unused
	RollAnalogSel,
	PitchAnalogSel,
	YawAnalogSel,
	MAX_ANALOG_CHANNELS
};

enum AlternateADCSelectors { ExternalVoltsAnalogSel = RangefinderAnalogSel };

enum BusDevSelectors {
	imuSel,
	baroSel,
	magSel,
	memSel,
	gpsSel,
	rfSel,
	asSel,
	flowSel,
	oledSel,
//	escSPISel,
//	escI2CSel,
	mag2Sel,
	maxDevSel
};

enum {
	mpu6000IMU, mpu6050IMU, icm20689IMU, noIMU
};
enum {
	ms5611Baro, ms5607Baro, bmp085Baro, noBaro
};
enum {
	hmc5xxxMag, ist8310Mag, noMag
};
enum {
	servoGimbal, noGimbal
};

enum {
	i2cEEPROMMem, spiFlashMem, ArmFlashMem, noMem
};

enum {
	oledDisplay, noDisplay
};

enum GPIOSelectorsBF {
	BaroXCLRSel = 2, BaroEOCSel = 3
};

enum BusSelectors {
	useI2C, useSPI
};

enum LEDSelectors {
	ledYellowSel, ledRedSel, ledBlueSel, ledGreenSel, MAX_LED_PINS
};

enum SerialPortSelectors {
	USBSerial, Usart1, Usart2, Usart3, Uart4, SoftSerial, MAX_SERIAL_PORTS
};

typedef const struct {
	GPIO_TypeDef* Port;
	uint16 Pin;
	uint16 PinSource;
} ConnectDef;

typedef const struct {
	boolean Used;
	TIM_TypeDef *Tim;
	uint16 Channel;
	uint16 CC;
	volatile uint32 * CCR;
	uint8 TimAF;
} TIMChannelDef;

typedef const struct {
	boolean Used;
	uint32 Channel;
	DMA_Stream_TypeDef * Stream;
} DMAChannelDef;

extern TIM_ICInitTypeDef TIM_ICInitStructure;

typedef const struct {
	boolean Used;
	ConnectDef P;
	GPIOMode_TypeDef Mode;
	GPIOOType_TypeDef OType;
	GPIOPuPd_TypeDef PuPd;
	TIMChannelDef Timer;
	DMAChannelDef DMA;
	IRQn_Type PinISR;
} PinDef;

typedef const struct {
	boolean Used;
	ADC_TypeDef* ADCx;
	ConnectDef A;
	uint32 ADCChannel;
	DMAChannelDef DMA;
	uint8 Rank;
} AnalogPinDef;

typedef const struct {
	boolean Used;
	USART_TypeDef* USART;
	ConnectDef Tx, Rx;
	uint32 Baud;
	uint16 Parity;
	uint16 StopBits;
	boolean DMAUsed;
	uint8 USART_AF;
	IRQn_Type ISR;
	uint32 DMAChannel;
	DMA_Stream_TypeDef * TxStream;
	IRQn_Type TxDMAISR;
	DMA_Stream_TypeDef * RxStream;
} SerialPortDef;

typedef const struct {
	boolean Used;
	I2C_TypeDef* I2C;
	ConnectDef SCL, SDA;
	uint8 I2C_AF;
} I2CPortDef;

typedef const struct {
	boolean Used;
	SPI_TypeDef* SPIx;
	ConnectDef P[3];
} SPIPortDef;

typedef const struct { // C2 Port GUI changeable UAVX
	boolean Used;
	uint8 tag;
	uint8 type;
	uint8 useSPI;
	uint8 busNo;
	uint8 i2cId;
	ConnectDef P;
} DevDef;

//________________________________________________________________________________________________

#define spi_21 			SPI_BaudRatePrescaler_2
#define spi_10_5 		SPI_BaudRatePrescaler_4
#define spi_5_250 		SPI_BaudRatePrescaler_8
#define spi_2_625 		SPI_BaudRatePrescaler_16
#define spi_1_3125 		SPI_BaudRatePrescaler_32
#define spi_0_65625 	SPI_BaudRatePrescaler_64
#define spi_0_3128125 	SPI_BaudRatePrescaler_128
#define spi_0_15640625 	SPI_BaudRatePrescaler_256

#define MAX_SPI_PORTS 4
#define MAX_I2C_PORTS 4

//______________________________________________________________________

extern const PinDef GPIOPins[];
extern const PinDef LEDPins[];
extern const AnalogPinDef AnalogPins[];
extern const I2CPortDef I2CPorts[];
extern const SPIPortDef SPIPorts[];
extern const SerialPortDef SerialPorts[];
extern const PinDef SoftSerialTxPin;
extern PinDef WSPin;

extern const PinDef PWMPins[];

extern idx CurrMaxPWMOutputs;
extern idx GPSSerial, RCSerial, TelemetrySerial, FrSkySerial, OpenLogSerial;

extern const uint8 IMUQuadrant;
extern const uint8 MagQuadrant;

extern const uint8 currGimbalType;
extern const DevDef busDev[];
extern const boolean ledsLowOn;
extern const boolean beeperLowOn;
extern boolean UsingOpenLog;
const uint8 CurrBattVoltsAnalogSel;
extern uint8 CurrMagSel;


//_______________________________________________________________

void systemReset(boolean toBootloader);
void InitClocks(void);

void InitHarness(void);
void InitTarget(void);

void InitPin(const PinDef * d);
void InitOutputPin(const PinDef * d);
void InitPinMode(const PinDef * d, boolean IsInput);

void SetBaudRate(uint8 s, uint32 BaudRate);
void InitSerialPort(uint8 s, boolean Enable);

void InitPWMPin(const PinDef * u, uint16 pwmprescaler, uint32 pwmperiod,
		uint32 pwmwidth);
void InitSoftSerialTxTimer(void);

void InitI2C(uint8 I2CCurr);
void UnstickI2C(uint8 I2CCurr);
void InitSPIGPIOPins(uint8 spiPort, boolean highClock);

void WSPinDMAEnable(uint16 wsBufferSize);
void InitWSPin(uint16 wsBufferSize);

void SoftSerialTxTimerStart(void);
void SoftSerialTxTimerStop(void);

boolean DigitalRead(const ConnectDef * d);
void DigitalWrite(const ConnectDef * d, boolean m);
void DigitalToggle(const ConnectDef * d);

#endif

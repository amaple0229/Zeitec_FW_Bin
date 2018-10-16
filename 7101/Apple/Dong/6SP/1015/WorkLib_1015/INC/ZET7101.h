/**
 * @file ZET6275.h
 *
 *  ZET66241 Register Definition
 *
 * @version $Revision: 43 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/
#ifndef __ZET6241_H__
#define __ZET6241_H__

///==================================================================================================================================================================================///
///  PART I : Address Base Definition
///==================================================================================================================================================================================///
#define BASE_FLASH_BA         (0x00000000)
#define BASE_SRAM_BA          (0x00100000)

#define BASE_GCR_BA           (0x00180000)
#define BASE_CLK_BA           (0x00184000)
#define BASE_INT_BA           (0x00188000)
#define BASE_GPIO_BA          (0x0018C000)

#define BASE_TFC0_BA					(0x00190000)
#define BASE_TFC1_BA					(0x00192000)
#define BASE_TFC2_BA					(0x00194000)

#define BASE_TRX_BA           (0x00190000)
#define BASE_TRY_BA           (0x00194000)

#define BASE_WDT_BA           (0x00198000)
#define BASE_TMR01_BA         (0x0019C000)

#define BASE_I2C0_BA          (0x001A0000)

#define BASE_ADC_BA           (0x001B8000)

#define BASE_DAC_BA           (0x001BA000)

#define BASE_DGF_BA           (0x001BC000)

#define BASE_ANA_BA           (0x001C0000)
#define BASE_FLASH_OPT_BA     (0x001C4000)

///==================================================================================================================================================================================///
///  PART II : Register  Definition
///==================================================================================================================================================================================///

///===========================================================///
///                                                           ///
///  REGISTER CLASS #1: GLOBAL CONTROL REGISTERS              ///
///                                                           ///
///===========================================================///
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// BASE_SRAM_RX_SINE0 --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define SRAM_RX_SINE0 (BASE_SRAM_BA + 0x010000)
#define SRAM_RX_SINE1 (BASE_SRAM_BA + 0x012000)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// BASE_SRAM_RX_COEF --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define SRAM_RX_COEF	(BASE_SRAM_BA + 0x014000)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// SRAM_PN_DEMOD --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define SRAM_PN_DEMOD	(BASE_SRAM_BA + 0x016000)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// BASE_SRAM_TX_SINE -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define SRAM_TX_SINE	(BASE_SRAM_BA + 0x018000)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_PRT_REG -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PRT_REG (BASE_GCR_BA + 0x00)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_PCON --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PCON    (BASE_GCR_BA + 0x04)
	#define PCON_IDLE_EN                  (1<<7)     ///< 1: For PCON_IDLE(PCON[0]) valid, 0: For PCON_IDLE(PCON[0]) invalid
  #define DUAL_MODE                     (1<<4)     ///< Green mode enable control bit --> 0:MCU is in Normal mode, 1:MCU is in Green  mode  
  #define PCON_GF1                      (1<<3)     ///< General Purpose User Flag 1
  #define PCON_SW_RST                   (1<<2)     ///< Software reset MCU enable bit(default value 0). 
                                                   ///  -->0: default value / after any reset, this bit is set to "0" automatically
                                                   ///  -->1: set to "1", MCU will enter to reset status 
  #define PCON_STOP                     (1<<1)     ///< Stop Mode Select. Setting this bit will stop program execution, halt the CPU oscillator, and internal timers, and place the CPU in a low-power mode. 
                                                   ///  This bit will always be read as a 0. Setting this bit while the Idle bit is set will place the device in an undefined state.                                                   
  #define PCON_IDLE                     (1<<0)     ///< Idle Mode Select. Setting this bit will stop program execution but 


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_PC -->  : Protection Register  (DONE)
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PC      (BASE_GCR_BA + 0x0C)

#ifdef FEATURE_7101_DEVELOPMENT
#define SYSREG (BASE_GCR_BA + 0x10)
  #define IO_CTRL_BIT (1<<7)
#endif


///===========================================================///
///                                                           ///
///  REGISTER CLASS #2: CLOCK CONTROL REGISTERS               ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_CK_CTR0 -->  : Clock control register0
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_CK_CTR0      (BASE_CLK_BA + 0x00)
  #define DP512K_EN         (1<<7)           ///< For IRC 512K test. CLK output pin is PORT 1[3] --> 0: disable IRC 512k dump function, 1: enable 
  #define OSCO_PHEN1        (1<<6)           ///< 0: disable OSCO 4.7k pull high function, 1: enable
  #define OSCO_PHEN0        (1<<5)           ///< 0: disable OSCO 200k pull high function, 1: enable 
  #define CLK_EXT           (1<<2)           ///< 0: disable external CLK input, 1: enable  
  #define CK512_LPEN        (1<<1)           ///< 0: disable low pass filter function., 1: enable low pass filter function.    
  #define CK16M_LPEN        (1<<0)           ///< 0: disable low pass filter function., 1: enable  

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_CK_CTR1 -->  : SRAM  access time option
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_CK_CTR1      (BASE_CLK_BA + 0x04)
  #define CK_512K_RS2         (1<<7)           ///< TRIM code for 512K OSC, when INMODE mode(TEST MODE)
  #define CK_512K_RS1         (1<<6)           ///< 
  #define CK_512K_RS0         (1<<5)           ///< 

  #define FSEL_REG_EN         (1<<4)           ///< 0: disable, IRC frequence selection bits value from code option (default value), 1: enable, IRC frequence selection bits value from register
  #define FSEL1               (1<<3)           ///< 11: 36MHz, 10: 32MHz
  #define FSEL0               (1<<2)           ///< 01: 18MHz, 00: 16MHz

  #define IRCTRIM_EN          (1<<1)           ///< IRC trim function enable bit --> 0: disable C trim function, 1: enable IRC trim function
  #define DP16M_EN            (1<<0)           ///< Dump OSC 16M function for testing, 1: enable OSC dump function, 0: disable , When Dump16M EN=1 , CK16M will report from P1[0]

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_CK_CTR2 -->  :  Clock Control regster#2
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_CK_CTR2      (BASE_CLK_BA + 0x08)
  #define CK_MODE1         (1<<7)           ///< IRIC  mode selection
  #define CK_MODE0         (1<<6)           ///< [00]: IRIC (OSCI pull low), [01]: --, [10]: External clock, [11]: Bandgap reference (OSCI pull low)
  #define CK_RS2           (1<<5)           ///< R trimming bit selection
  #define CK_RS1           (1<<4)           ///< 
  #define CK_RS0           (1<<3)           ///< 
  #define CK_CS2           (1<<2)           ///< C trimming bit selection
  #define CK_CS1           (1<<1)           ///<   
  #define CK_CS0           (1<<0)           ///<   
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
/// REG32_CK_CTR3 -->  :  Clock Control regster#3
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_CK_CTR3      (BASE_CLK_BA + 0x14)
  #define CK_CO_FSEL         (1<<7)           ///< FSEL decided by CodeOption  

	
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_CKCON -->  : MCU frequncy control register (DONE)
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_CKCON       (BASE_CLK_BA + 0x0C)
  #define WD2               (1L<<7)          ///<  : Watchdog Timer Mode Select 2-0
  #define WD1               (1L<<6)          ///<  : Watchdog Timer Mode Select 2-0
  #define WD0               (1L<<5)          ///<  : Watchdog Timer Mode Select 2-0
                                                  /// --------------------------------------------------------------- 
                                                  /// WD2    WD1   WD0   Interrupt Divider   Reset Divider
                                                  /// ---------------------------------------------------------------                                                
                                                  /// 0       0      0      2^12                 29 + 512  ///< the shortst
                                                  /// 0       0      1      2^13                210 + 512                                                 
                                                  /// 0       1      0      2^14                211 + 512                                                
                                                  /// 0       1      1      2^15                212 + 512                                                
                                                  /// 1       0      0      2^16                213 + 512                                                
                                                  /// 1       0      1      2^17                214 + 512                                                
                                                  /// 1       1      0      2^18                215 + 512                                                
                                                  /// 1       1      1      2^19                216 + 512 ///< the Longeest                                                  
                                                  /// ---------------------------------------------------------------   
  #define CKCON_OP_FRQ2    (1<<2)                 ///<  : MCU operation frequency control register 2-0
  #define CKCON_OP_FRQ1    (1<<1)                 ///<  : MCU operation frequency control register 2-0
  #define CKCON_OP_FRQ0    (1<<0)                 ///<  : MCU operation frequency control register 2-0
                                                 
    #define CKCON_CLK_DIV_1     (0<<0)            ///< 000: MCU FREQ = sytem clock
    #define CKCON_CLK_DIV_2     (1<<0)            ///< 001: MCU FREQ = sytem clock /2
    #define CKCON_CLK_DIV_4     (2<<0)            ///< 010: MCU FREQ = sytem clock /4
    #define CKCON_CLK_DIV_8     (3<<0)            ///< 011: MCU FREQ = sytem clock /8
    #define CKCON_CLK_DIV_16    (4<<0)            ///< 100: MCU FREQ = sytem clock /16
    #define CKCON_CLK_DIV_32    (5<<0)            ///< 101: MCU FREQ = sytem clock /32
    #define CKCON_CLK_DIV_64    (6<<0)            ///< 110: MCU FREQ = sytem clock /64
    #define CKCON_CLK_DIV_128   (7<<0)            ///< 111: MCU FREQ = sytem clock /128                                         


    #define CKCON_CLK_FULL_SPEED     (0x00)           


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PSEUDO -->  : Psuedo clock control
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PSEUDO      (BASE_CLK_BA + 0x10)
  #define RST_PSEUDO              (1<<7)           ///<  : 1: reset Analog IMO block pseduo-random circuit, 0:  enable Analog IMO block pseduo-random circuit
  
  #define PSEUDO_DIV2             (1<<4)           ///<  : pseduo clock divider / period for frequence change 
  #define PSEUDO_DIV1             (1<<3)           ///<  : --> 000=/4; 001=/8; 010=/16; 011=/32; 100=/64; 101=/128; 110=/512; 111=/1024;
  #define PSEUDO_DIV0             (1<<2)           ///<  : 
  #define PSEUDO_RANGE1           (1<<1)           ///<  : Select pseudo bias stregnth(frequence veration)
  #define PSEUDO_RANGE0           (1<<0)           ///<  : --> 00= 8 : 0 (non pseudo), 01= 6 : 2, 10= 5 : 3, 11= 3 : 5

///===========================================================///
///                                                           ///
///  REGISTER CLASS #3: TIMER CONTROL REGISTERS               ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TCON -->  : Timer control registers
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TCON        (BASE_TMR01_BA + 0x00)
  #define TCLK_SEL          (1L<<7)          ///<  :  Timer Clock select
     #define TCLK_FSEL        (0L<<7)        ///  0 = clock source by FSEL[1:0]
     #define TCLK_512K        (1L<<7)        ///  1 = clock source by 512K
     
  #define TMR_RST           (1L<<3)          ///<  :  Timer Reset --> 1 = Clear TMR counter
  
  #define TMR_RELOAD        (1L<<1)          ///<  : Timer Reload bit --> TMR_EN=1 && TMR_Reload =1  while counter = 32'hffff_ffff  reload TMR value to counter
  #define TMR_EN            (1L<<0)          ///<  : Timer Enable Bit
	
	
#ifdef FEATURE_7101_DEVELOPMENT	
#define REG_TMR_32M_STABLE  (BASE_TMR01_BA + 0x01)
 #define TMR_STABLE						(1L<<0) 				 ///<  : Timer clock Stable Bit
		
#define REG_TMR_40K_STABLE (BASE_TMR01_BA + 0x02)
#endif	
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TMR -->  : Timer Reload register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TMR        (BASE_TMR01_BA + 0x08)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TMR_IF -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TMR_IF      (BASE_TMR01_BA + 0x50)
  #define TMR_IF            (1L<<0) 
	
#ifdef	FEATURE_7101_DEVELOPMENT
#define REG32_TMR_CNT      (BASE_TMR01_BA + 0x5C)	


	#define REG32_TMR1_IF			 (BASE_TMR01_BA + 0x54)
	#define REG32_TMR2_IF			 (BASE_TMR01_BA + 0x58)

	
	#define REG32_TMR1_CNT		 (BASE_TMR01_BA + 0x60)
	#define REG32_TMR2_CNT		 (BASE_TMR01_BA + 0x64)

	#define REG32_TMR1         (BASE_TMR01_BA + 0x10)
	#define REG32_TMR2				 (BASE_TMR01_BA + 0x18) 

	#define REG32_TCON1        (BASE_TMR01_BA + 0x0C)
	#define REG32_TCON2        (BASE_TMR01_BA + 0x14)

  
	#define REG_TMR1_32M_STABLE  (BASE_TMR01_BA + 0x0C + 0x01)
	#define REG_TMR1_40K_STABLE  (BASE_TMR01_BA + 0x0C + 0x02)

	#define REG_TMR2_32M_STABLE  (BASE_TMR01_BA + 0x14 + 0x01)
	#define REG_TMR2_40K_STABLE  (BASE_TMR01_BA + 0x14 + 0x02)

	

	
#else
#define REG32_TMR_CNT     (BASE_TMR01_BA + 0x58)
#endif

///===========================================================///
///                                                           ///
///  REGISTER CLASS #4:  GPIO CONTROL REGISTERS               ///
///                                                           ///
///===========================================================///
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_SPI2_CTRL
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_SPI2_CTRL     (BASE_TMR01_BA + 0x120)
  #define SPI2_RDF          (1L<<3)
  #define SPI2_SHFT_START   (1L<<2)   
  #define SPI2_ENABLE       (1L<<0)   // 0 :disable

	
#define REG_SPI2_STATUS   (BASE_TMR01_BA + 0x120) 
  #define SPISRAM_FW_ACCESS    (1<<1)
  #define SPIBUF_WEN    (1<<4)
	
#define REG_SPI2_CMD0     (BASE_TMR01_BA + 0x150)      //  0x19C150
#define REG_SPI2_CMD1     (BASE_TMR01_BA + 0x154)      //  0x19C154
		
#if 1 //def FEATURE_SPI_TRANSFER
#define SPI2_IF               0x19C130
#define SPI2_SHIFT            0x19C124	       
#define SPI2_CTL              0x19C128	        
#define SPI2_DATA             0x19C12C	   

#define SPI2_DATA0            0x19C134	   
#define SPI2_DATA1            0x19C135	   
#define SPI2_DATA2            0x19C136	   
#define SPI2_DATA3            0x19C137	   
#define SPI2_DATA4            0x19C138	   
#define SPI2_DATA5            0x19C139	   
#define SPI2_DATA6            0x19C13a	   
#define SPI2_DATA7            0x19C13b	   

#define SPI2_BUF_ADR          0x19C160

#define REG_SPI2_RX_BUF           (BASE_TMR01_BA + 0x13C) // 0x19C13c	
#define REG_SPI2_FIFO_CTRL      (BASE_TMR01_BA + 0x200)   //0x19c200
#define REG_SPI2_TX_DMA_DEST_ADR      (BASE_TMR01_BA + 0x204)   //0x19c204
#define REG_SPI2_RX_DMA_SRC_ADR     (BASE_TMR01_BA + 0x208)   //0x19c208
	
#define REG_SPI2_TX_DMA_CNT     (BASE_TMR01_BA + 0x20C)   //0x19c20C
#define REG_SPI2_RX_DMA_CNT     (BASE_TMR01_BA + 0x210)   //0x19c210
	
#define REG_SPI2_TX_DMA_DEST_MAX_ADR      (BASE_TMR01_BA + 0x214)   //0x19c214
#define REG_SPI2_RX_DMA_SRC_MAX_ADR     (BASE_TMR01_BA + 0x218)   //0x19c218
#endif
	
#if 1 //def FEATURE_IR
#define IR_CON				0x19C080
#define IR_TMR				0x19C084
#define IR_IF				0x19C088
#define IR_DUTY				0x19C08C
#define IR_PWM_CNT			0x19C090
#define IR_RX_CNT			0x19C094

#define AD_CTRL14			0x1B8038 //control analog trace as Digital I/O: 1:enable, 0:disable

#define P2				0x18C020
#define P2_INT_EN			0x18C024 // interrupt enable
#define P2_INT    			0x18C028 // int flag

#define IODIR_P2			0x18C02C
#define P2_SEL18			0x18C030
#define P2_WAKE_EN			0x18C034 // wake-up enable
#define P2_TRG_EDGE			0x18C038 // wake or interrupt edge 1: pos-edge 0: negedge


#define P3				0x18C040 
#define P3_WU			0x18C048  
//bit0: P35 int flag, bit4: P37 int flag
//bit1: P35 wakeup enable, bit5: P37  wakeup enable
//bit2: P35 trigger edge, bit6: P37  trigger edge,  1: posedge 0: negedge
//bit3: P35 interrupt  enable, bit5: P37   interrupt

#endif


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PORT_CFG -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PORT_CFG     (BASE_GPIO_BA + 0x00)
  #define PH47K             (1L<<5) ///<  : Port Pull High 4.7K selection
  #define PH200K            (1L<<4) ///<  : Port Pull High 200K selection

  #define IOSEL1            (1L<<1) ///<  : IO drive cirrent selection
  #define IOSEL0            (1L<<0) ///<              

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_P1 -->  : Port1 Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_P1           (BASE_GPIO_BA + 0x04)
  #define DSD               (1L<<7)   
  #define CSD               (1L<<6)   
  #define P15               (1L<<5)   
  #define P14               (1L<<4)   
  #define SDI               (1L<<3)   ///<  : Data shifted-in pin while SPI interface enable ( SPIEN  = 1)
  #define SDO               (1L<<2)   ///<  : Data shifted-out pin while SPI interface enable ( SPIEN  = 1)
  #define SCK               (1L<<1)   ///<  : SPI clock pin while SPI interface enable ( SPIEN  = 1)
  #define SS                (1L<<0)   ///<  : Slave Mode Selection while SPI interface enable ( SPIEN  = 1)   

  #define P17_OUT    (1L<<7)
  #define P16_OUT    (1L<<6)
  #define P15_OUT    (1L<<5)
  #define P14_OUT    (1L<<4)
  #define P13_OUT    (1L<<3)
  #define P12_OUT    (1L<<2)
  #define P11_OUT    (1L<<1)
  #define P10_OUT    (1L<<0)  
  

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PORT1_PH -->  :  Pull high function enable register of PORT 1.
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PORT1_PH           (BASE_GPIO_BA + 0x08)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PORT1_WU -->  : PORT 1 change wakeup function enable register of PORT 1
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PORT1_WU           (BASE_GPIO_BA + 0x0C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_IODIR -->  : I/O direction select control of "Port1". --> 0: Input direction.,  1:Output direction
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_IODIR           (BASE_GPIO_BA + 0x10)
  #define IODIRP17               (1L<<7)
  #define IODIRP16               (1L<<6)
  #define IODIRP15               (1L<<5)
  #define IODIRP14               (1L<<4)
  #define IODIRP13               (1L<<3)
  #define IODIRP12               (1L<<2)
  #define IODIRP11               (1L<<1)
  #define IODIRP10               (1L<<0)   

	
#ifdef FEATURE_7101_DEVELOPMENT
#define REG32_IODIRP2         (BASE_GPIO_BA + 0x2C)
#endif

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_P3 -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_P3           (BASE_GPIO_BA + 0x40)
  #define SDA               (1L<<7)
  #define SCL               (1L<<6)
  #define T1                (1L<<5)
  #define T0                (1L<<4)
  #define INT1              (1L<<3)
  #define INT0              (1L<<2)

  #define P37               (1L<<7)
  #define P36               (1L<<6)
  #define P35               (1L<<5)
  #define P34               (1L<<4)  
  #define P33               (1L<<3)
  #define P32               (1L<<2)  
  #define P31               (1L<<1)
  #define P30               (1L<<0)  

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PORT3_PH -->  : Pull high function enable register of PORT 3
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PORT3_PH           (BASE_GPIO_BA + 0x44)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PORT3_WU -->  : PORT 3 change wakeup function enable register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PORT3_WU           (BASE_GPIO_BA + 0x48)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_IODIRP3 -->  : I/O direction select control of "Port3". --> 0: Input direction.,  1:Output direction
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_IODIRP3         (BASE_GPIO_BA + 0x4C)
  #define IODIRP37               (1L<<7)
  #define IODIRP36               (1L<<6)
  #define IODIRP35               (1L<<5)
  #define IODIRP34               (1L<<4)
  #define IODIRP33               (1L<<3)
  #define IODIRP32               (1L<<2)
  #define IODIRP31               (1L<<1)
  #define IODIRP30               (1L<<0)   

  #define P37_OUT    (1L<<7)
  #define P36_OUT    (1L<<6)
  #define P35_OUT    (1L<<5)
  #define P34_OUT    (1L<<4)
  #define P33_OUT    (1L<<3)
  #define P32_OUT    (1L<<2)
  #define P31_OUT    (1L<<1)
  #define P30_OUT    (1L<<0)  
  

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_IODIR -->  :  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_P3_SEL18    (BASE_GPIO_BA + 0x50)
	#define SEL18_P10 							(1L<<8)
  #define SEL18_P37               (1L<<7)
  #define SEL18_P36               (1L<<6)
  #define SEL18_P35               (1L<<5)
  #define SEL18_P34               (1L<<4)
  #define SEL18_P33               (1L<<3)
  #define SEL18_P32               (1L<<2)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_P3_PH200K -->  :  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_P3_PH200K    (BASE_GPIO_BA + 0x54)
  #define PH200K18EN               (1L<<5)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_P3_WK_EDGE -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_P3_WK_EDGE    (BASE_GPIO_BA + 0x58)
  #define WAKE_EDGE               (1L<<5)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRACE_PA_OUT -->  : Trace GPIO Read
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRACE_PA_OUT    (BASE_GPIO_BA + 0x120) ///<
#define REG32_TRACE_PB_OUT    (BASE_GPIO_BA + 0x140) ///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_IODIR_PA -->  : Trace GPIO Setup
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_IODIR_PA        (BASE_GPIO_BA + 0x12C)
#define REG32_IODIR_PB        (BASE_GPIO_BA + 0x14C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_VOLT_CTRL -->  : Trace GPIO Setup
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_EN_CTRL_PA      (BASE_ANA_BA + 0x20)
#define REG32_EN_CTRL_PB      (BASE_ANA_BA + 0x24)

	#define TRY_PORT_29				 (1L<<29) 					///< Set TRY29 as GPIO Input

///===========================================================///
///                                                           ///
///  REGISTER CLASS #5:  I2C CONTROL REGISTERS                ///
///                                                           ///
///===========================================================///
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2C_DADDR -->  : Slave address
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2C_DADDR    (BASE_I2C0_BA + 0x00)
  #define I2C_DADDR_BUSY    (1<<7)                ///<  : I2C busy flag. In slave mode, this flag will be set while I2C busy is busy, and it is non-active in master mode.
  #define I2C_DADDR_ADDR6   (1<<6)                ///<  : Data address on I2C bus BIT#6 -->  I2C DADDR stores the address of ZET62xx when ZET91xx used as I2C slave device
  #define I2C_DADDR_ADDR5   (1<<5)                ///<  : Data address on I2C bus BIT#5 
  #define I2C_DADDR_ADDR4   (1<<4)                ///<  : Data address on I2C bus BIT#4 
  #define I2_DCADDR_ADDR3   (1<<3)                ///<  : Data address on I2C bus BIT#3 
  #define I2_DCADDR_ADDR2   (1<<2)                ///<  : Data address on I2C bus BIT#2 
  #define I2_DCADDR_ADDR1   (1<<1)                ///<  : Data address on I2C bus BIT#1   
  #define I2_DCADDR_ADDR0   (1<<0)                ///<  : Data address on I2C bus BIT#0     


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2C_SADDR -->  :  Slave device address. I2C SADDR stores the slave device address of ZET6223S.
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2C_SADDR    (BASE_I2C0_BA + 0x04)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_FWADR -->  : FLASH Writer address register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FWADR        (BASE_I2C0_BA + 0x08)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2CON -->  : I2C Control Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2CON        (BASE_I2C0_BA + 0x0C)
  #define  STROBE_PEND    (1L<<7)         ///<  :--> In master mode, it is used as strobe signal to control IIC circuit to send SCL clock
                                          ///       Reset automatically after receiving or transmitting handshake signal (ACK or NACK)
                                          ///   --> In slave mode, it is used as pending signal, user should clear it after fill data into Tx buffer or
                                         	///       get data from Rx buffer to inform slave IIC circuit to release SCL signal.                                                                          
  #define  MASTER_MODE    (1L<<6)        	///<  : I2C Master/Slave mode select bit. -->1: Master, 0: Slave
  #define  FAST_MODE      (1L<<5)        	///<  : I2C Fast/Standard mode select bit. --> 1: Fast mode (400K bits/s), 0: Standard mode (100K bits/s)  
  #define  I2C_STOP       (1L<<4)        	///<  : --> In slave mode, if STOP=1 and R/nW=0 then ZET6223S must return nACK signal to master device.
                                          ///    --> In Master mode, if STOP=1 and R/nW=1 then ZET6223S must return nACK signal to slave device before send STOP signal.
                                          ///        If STOP=1 and R/nW=0 then ZET6223S send STOP signal after receive an ACK signal. 
                                          ///        Reset when ZET6223S send STOP signal to Slave device.                                     
  #define  SAR_EMPTY      (1L<<3)          ///< : Set when ZET6223S transmit 1 byte data from I2C Slave Address Register and receive ACK(or nACK) signal. Reset when MCU write 1 byte data to I2C Slave Address Register.
  #define  I2C_ACK        (1L<<2)          ///< : The Ack condition bit is set to 1 by hardware when the device responds acknowledge (ACK).Reset when the device responds not-acknowledge (nACK) signal.
  #define  I2C_FULL       (1L<<1)          ///< : Set by hardware when I2C receive buffer register is full. Reset by hardware when MCU read data from I2C receive buffer register. 
  #define  I2C_EMPTY      (1L<<0)          ///< : Set by hardware when I2C transmit buffer register is empty and receive ACK (or nACK) signal.Reset by hardware when MCU write new data to I2C transmit buffer register.

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2CON2 -->  : I2C Control Register2
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2CON2        (BASE_I2C0_BA + 0x10)
  #define RG_I2C_ADDR_PHENB       (1L<<6)
  #define RG_SCL_LOCKB            (1L<<5)          ///<  : 0 = SCL will be locked , 1 = The lock function is disabled.
  #define RG_DRVSEL_I2C_18V1      (1L<<3)
  #define RG_DRVSEL_I2C_18V0      (1L<<2)
  #define RG_DRVSEL_SPI_18V1      (1L<<1)
  #define RG_DRVSEL_SPI_18V0      (1L<<0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2C_CTRL -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2C_CTRL        (BASE_I2C0_BA + 0x14)
  #define I2CEN                   (1L<<9)   ///<  : I2C I/O enable control register & I2C Enable bit --> 0: I2C Disable & GPIO Enable, 1: I2C Enable
  #define I2C_RSTN                (1L<<8)
  #define I2C_DEGLITCH_EN         (1L<<7)
  #define I2C_DEGLITCH_MAX2       (1L<<6)
  #define I2C_DEGLITCH_MAX1       (1L<<5)
  #define I2C_DEGLITCH_MAX0       (1L<<4)
  #define STD_I2C_EN              (1L<<3)
  #define FLASH_I2C_EN            (1L<<2)  ///<  : Flash I2C function enable bit(default enable when Reset), disable/enable by firmware. One of Flash_I2C_EN & STD_I2C_EN is Active
  #define I2C_START_FLAG          (1L<<1)  ///<  : 0 = non I2C start pattern matched, 1 = I2C start pattern matched	
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2CBUF -->  : I2C TX/RX buffer
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2CBUF        (BASE_I2C0_BA + 0x18)  

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2C_TX_IF -->  : I2C TX interrupt flag
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2C_TX_IF       (BASE_I2C0_BA + 0x50)  
  #define I2CTX_IF                   (1L<<0)         ///<  :  I2C transmit interrupt flag. It will be set while I2C transmits 1 byte data and receive handshake signal (ACK or NACK), and cleared by firmware or I2C disable.
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_I2C_RX_IF -->  : I2C RX interrupt flag
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_I2C_RX_IF       (BASE_I2C0_BA + 0x54)  
  #define I2CRX_IF                   (1L<<0)        ///<  :  I2C receive interrupt flag. It will be set while I2C receiving 1byte data and responds ACK signal and cleared by firmware or I2C disable.
  #define I2C_RX_IF_CLEAR            (0x00)
  
#ifdef	FEATURE_7101_DEVELOPMENT
#define I2C_FIFO_CTRL  (BASE_I2C0_BA	+ 0x200)
#endif
///===========================================================///
///                                                           ///
///  REGISTER CLASS #6:  WDT CONTROL REGISTERS                ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_WDCON -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_WDCON       (BASE_WDT_BA + 0x00)
#ifdef FEATURE_7101_DEVELOPMENT
  #define WDT_RESET_TYPE                  (1L<<7)  // 1: WTD overflow, 0: WDT overflow + 512 system clock
#endif


#define REG32_WDCON0       (BASE_WDT_BA + 0x00)
#define REG32_WDCON1       (BASE_WDT_BA + 0x01)
  #define WARMUP_SEL2               (1L<<11)          ///<  : WDTSEL2,1,0 : WARM UP timer control bits, !!! not WDT timer control, WDT timer control --> REG32_CKCON[7:5]
  #define WARMUP_SEL1               (1L<<10)          ///<  : WDTSEL2,1,0 : WARM UP timer control bits, !!! not WDT timer control, WDT timer control --> REG32_CKCON[7:5]
  #define WARMUP_SEL0               (1L<<9)           ///<  : WDTSEL2,1,0 : WARM up timer control bits, !!! not WDT timer control, WDT timer control --> REG32_CKCON[7:5]
                                                  ///        --> 0 0 0 : 512  512KOSC clocks
                                                  ///        --> 0 0 1 : 64   512KOSC clocks
                                                  ///        --> 0 1 0 : 32   512KOSC clocks
                                                  ///        --> 0 1 1 : 16   512KOSC clocks  
                                                  ///        --> 0 0 0 : 8   512KOSC clocks
                                                  ///        --> 0 0 1 : 4   512KOSC clocks
                                                  ///        --> 0 1 0 : 2   512KOSC clocks
                                                  ///        --> 0 1 1 : 1   512KOSC clocks 
                                                  
  #define WDTEN                 (1L<<8)           ///<  : WDT enable control bit --> 0:disable & OFF 512K OSC, 1: Enable & ENable 512K OSC

  #define SMOD                  (1L<<7)           ///<  : ///< NO Function Serial Modification. This bit controls the doubling of the serial port 1 baud rate in modes 1, 2, and 3. 0 = Serial port 1 baud rate operates at normal speed 1 = Serial port 1 baud rate is doubled.
  #define POR                   (1L<<6)           ///<  : Power-on Reset Flag --> 0 = Last reset was from a source other than a power-on reset , 1 = Last reset was a power-on reset.
  #define EPF1                  (1L<<5)           ///<  : ///< NO Function Enable Power fail Interrupt --> 0= Power-fail interrupt disabled., 1= Power-fail interrupt enabled during normal operation. Power-fail interrupt enabled in Stop mode if BGS is set.
  #define PFI                   (1L<<4)           ///<  : ///< NO Function Power fail Interrupt Flag. ///< NO Function
  
  #define WTRF                  (1L<<2)          ///<  : Watchdog Timer Reset Flag.
  #define EWT                   (1L<<1)          ///<  : Enable Watchdog Timer Reset  1: WDT interrupr + WDT RST, WDT RST after WDT INT  0: WDT Interrupr Only 
  #define RWT                   (1L<<0)          ///<  : Reset Watchdog Timer   1: WDT counter reset, set bit by Firmware & clear by HW next cycle 
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_WDT_IF -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_WDT_IF       (BASE_WDT_BA + 0x50)
  #define WDT_IF                   (1L<<0)           ///<  :  Watch dog interrupt flag


///===========================================================///
///                                                           ///
///  REGISTER CLASS #7:  TOUCH FUNCTION CONTROL 0 REGISTERS   ///
///                                                           ///
///===========================================================///



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRY_PNCNT --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRY_PNCNT       							(BASE_TFC0_BA + 0x00)


#define REG8_PN_SINE_NUM_A                  (BASE_TFC0_BA + 0x00)
#define REG8_PN_SINE_NUM_B                  (BASE_TFC0_BA + 0x01)
#define REG8_PN_CHIP_NUM	                  (BASE_TFC0_BA + 0x02)
	#define RG_TX_DUAL_TONE	    								(1L<<7) 						
	#define RG_PN_CHIP_NUM5	    								(1L<<5) 						
	#define RG_PN_CHIP_NUM4	    								(1L<<4) 											
	#define RG_PN_CHIP_NUM3	    								(1L<<3) 											
	#define RG_PN_CHIP_NUM2	    								(1L<<2) 											
	#define RG_PN_CHIP_NUM1	    								(1L<<1) 											
	#define RG_PN_CHIP_NUM0	    								(1L<<0) 	

	#define PN_CHIP_EN_NUM    							  	(0x1F) 	

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PN_DURCNT --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PN_DURCNT       							(BASE_TFC0_BA + 0x04)						


#define REG16_CHIP_2_CHIP_DURA              (BASE_TFC0_BA + 0x04)           
#define REG16_FRM_2_FRM_DURA                (BASE_TFC0_BA + 0x06)           

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// SINE_FREQ_SCALE¡G --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_SINE_FREQ_SCALE       				(BASE_TFC0_BA + 0x08)	


#define REG8_TX_SINE_FREQSCALE_GRP_A        (BASE_TFC0_BA + 0x08) 
#define REG8_TX_SINE_FREQSCALE_GRP_B        (BASE_TFC0_BA + 0x09) 
#define REG16_TX_SINE_ROM_LEN               (BASE_TFC0_BA + 0x0A) 


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_PN_CODE¡G --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TX_PN_CODE       				(BASE_TFC0_BA + 0x0C)	



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_PN_CODE¡G -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FRONT_ADC_RST_CNT       				(BASE_TFC0_BA + 0x10)	


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PN_DEMOD_IF¡G -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PN_CTRL						       				(BASE_TFC1_BA + 0x00)
	#define RG_SQRT_OUT_SEL2											(1L<<6)						
	#define RG_SQRT_OUT_SEL1											(1L<<5)						
	#define RG_SQRT_OUT_SEL0											(1L<<4)						
	#define RG_LPF_COEFF_LENGTH_EVEN_OR_ODD				(1L<<3)						
	#define RG_RX_DUAL_TONE_EN										(1L<<2)						
	#define RG_STR_AMP														(1L<<1)						
	#define RG_AMP_FUNC_AUTO_EN										(1L<<0)						

#define PN_CTRL_NOISE_DETECT_ENABLE (RG_RX_DUAL_TONE_EN|RG_AMP_FUNC_AUTO_EN)



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_PN_DEMOD_IF¡G -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_PN_DEMOD_IF							       	(BASE_TFC1_BA + 0x08)
	#define RG_PN_DEMOD_IF												(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_PN_CODE_OUT0¡G -->
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_PN_CODE_OUT0							     (BASE_TFC1_BA + 0x10)
	#define RX_PN_CODE7_2													(1L<<30)						
	#define RX_PN_CODE7_1													(1L<<29)						
	#define RX_PN_CODE7_0													(1L<<28)						

	#define RX_PN_CODE6_2													(1L<<26)						
	#define RX_PN_CODE6_1													(1L<<25)						
	#define RX_PN_CODE6_0													(1L<<24)						


	#define RX_PN_CODE5_2													(1L<<22)						
	#define RX_PN_CODE5_1													(1L<<21)						
	#define RX_PN_CODE5_0													(1L<<20)						

	#define RX_PN_CODE4_2													(1L<<18)						
	#define RX_PN_CODE4_1													(1L<<17)						
	#define RX_PN_CODE4_0													(1L<<16)						


	#define RX_PN_CODE3_2													(1L<<14)						
	#define RX_PN_CODE3_1													(1L<<13)						
	#define RX_PN_CODE3_0													(1L<<12)						

	#define RX_PN_CODE2_2													(1L<<10)					
	#define RX_PN_CODE2_1													(1L<<9)						
	#define RX_PN_CODE2_0													(1L<<8)						

	#define RX_PN_CODE1_2													(1L<<6)						
	#define RX_PN_CODE1_1													(1L<<5)						
	#define RX_PN_CODE1_0													(1L<<4)						

	#define RX_PN_CODE0_2													(1L<<2)						
	#define RX_PN_CODE0_1													(1L<<1)						
	#define RX_PN_CODE0_0													(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_PN_CODE_OUT1¡G
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_PN_CODE_OUT1							     (BASE_TFC1_BA + 0x14)
	#define RX_PN_CODE15_2													(1L<<30)					
	#define RX_PN_CODE15_1													(1L<<29)					
	#define RX_PN_CODE15_0													(1L<<28)					

	#define RX_PN_CODE14_2													(1L<<26)					
	#define RX_PN_CODE14_1													(1L<<25)					
	#define RX_PN_CODE14_0													(1L<<24)					


	#define RX_PN_CODE13_2													(1L<<22)					
	#define RX_PN_CODE13_1													(1L<<21)					
	#define RX_PN_CODE13_0													(1L<<20)					

	#define RX_PN_CODE12_2													(1L<<18)					
	#define RX_PN_CODE12_1													(1L<<17)					
	#define RX_PN_CODE12_0													(1L<<16)					


	#define RX_PN_CODE11_2													(1L<<14)					
	#define RX_PN_CODE11_1													(1L<<13)					
	#define RX_PN_CODE11_0													(1L<<12)					

	#define RX_PN_CODE10_2													(1L<<10)					
	#define RX_PN_CODE10_1													(1L<<9)						
	#define RX_PN_CODE10_0													(1L<<8)						


	#define RX_PN_CODE9_2													(1L<<6)						
	#define RX_PN_CODE9_1													(1L<<5)						
	#define RX_PN_CODE9_0													(1L<<4)						

	#define RX_PN_CODE8_2													(1L<<2)						
	#define RX_PN_CODE8_1													(1L<<1)						
	#define RX_PN_CODE8_0													(1L<<0)						



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_PN_CODE_OUT2¡G --> 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_PN_CODE_OUT2							     (BASE_TFC1_BA + 0x18)


	#define RX_PN_CODE20_2													(1L<<18)					
	#define RX_PN_CODE20_1													(1L<<17)					
	#define RX_PN_CODE20_0													(1L<<16)					


	#define RX_PN_CODE19_2													(1L<<14)					
	#define RX_PN_CODE19_1													(1L<<13)					
	#define RX_PN_CODE19_0													(1L<<12)					

	#define RX_PN_CODE18_2													(1L<<10)					
	#define RX_PN_CODE18_1													(1L<<9)						
	#define RX_PN_CODE18_0													(1L<<8)						


	#define RX_PN_CODE17_2													(1L<<6)						
	#define RX_PN_CODE17_1													(1L<<5)						
	#define RX_PN_CODE17_0													(1L<<4)						

	#define RX_PN_CODE16_2													(1L<<2)						
	#define RX_PN_CODE16_1													(1L<<1)						
	#define RX_PN_CODE16_0													(1L<<0)						

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_FREQ_MULTIPLEA¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_FREQ_MULTIPLEA							     (BASE_TFC1_BA + 0x30)						
	#define RG_RX_FREQ_MULTIPLEA7													(1L<<7)					
	#define RG_RX_FREQ_MULTIPLEA6													(1L<<6)					
	#define RG_RX_FREQ_MULTIPLEA5													(1L<<5)					
	#define RG_RX_FREQ_MULTIPLEA4													(1L<<4)					
	#define RG_RX_FREQ_MULTIPLEA3													(1L<<3)					
	#define RG_RX_FREQ_MULTIPLEA2													(1L<<2)					
	#define RG_RX_FREQ_MULTIPLEA1													(1L<<1)					
	#define RG_RX_FREQ_MULTIPLEA0													(1L<<0)					

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_FREQ_MULTIPLEB¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_FREQ_MULTIPLEB							     (BASE_TFC1_BA + 0x34)						
	#define RG_RX_FREQ_MULTIPLEB7													(1L<<7)						
	#define RG_RX_FREQ_MULTIPLEB6													(1L<<6)						
	#define RG_RX_FREQ_MULTIPLEB5													(1L<<5)						
	#define RG_RX_FREQ_MULTIPLEB4													(1L<<4)						
	#define RG_RX_FREQ_MULTIPLEB3													(1L<<3)						
	#define RG_RX_FREQ_MULTIPLEB2													(1L<<2)						
	#define RG_RX_FREQ_MULTIPLEB1													(1L<<1)						
	#define RG_RX_FREQ_MULTIPLEB0													(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_SINE_ROM_LEN_RX¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_SINE_ROM_LEN_RX							     (BASE_TFC1_BA + 0x38)		
	#define RG_SINE_ROM_LEN_RX7													(1L<<7)						
	#define RG_SINE_ROM_LEN_RX6													(1L<<6)						
	#define RG_SINE_ROM_LEN_RX5													(1L<<5)						
	#define RG_SINE_ROM_LEN_RX4													(1L<<4)						
	#define RG_SINE_ROM_LEN_RX3													(1L<<3)						
	#define RG_SINE_ROM_LEN_RX2													(1L<<2)						
	#define RG_SINE_ROM_LEN_RX1													(1L<<1)						
	#define RG_SINE_ROM_LEN_RX0													(1L<<0)						



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_ADC_NUM¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_ADC_NUM							     (BASE_TFC1_BA + 0x3C)		
	#define RG_ADC_NUM4													(1L<<4)						
	#define RG_ADC_NUM3													(1L<<3)						
	#define RG_ADC_NUM2													(1L<<2)						
	#define RG_ADC_NUM1													(1L<<1)						
	#define RG_ADC_NUM0													(1L<<0)						
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_RX_HOLD_CYCE_ADDR¡G¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_RX_HOLD_CYCE_ADDR							     (BASE_TFC1_BA + 0x48)		
	#define RG_RX_HOLD_CYCE_ADDR14													(1L<<14)				
	#define RG_RX_HOLD_CYCE_ADDR13													(1L<<13)				
	#define RG_RX_HOLD_CYCE_ADDR12													(1L<<12)				
	#define RG_RX_HOLD_CYCE_ADDR11													(1L<<11)				
	#define RG_RX_HOLD_CYCE_ADDR10													(1L<<10)				
	#define RG_RX_HOLD_CYCE_ADDR7													(1L<<7)						
	#define RG_RX_HOLD_CYCE_ADDR8													(1L<<8)						
	#define RG_RX_HOLD_CYCE_ADDR7													(1L<<7)						
	#define RG_RX_HOLD_CYCE_ADDR6													(1L<<6)						
	#define RG_RX_HOLD_CYCE_ADDR5													(1L<<5)						
	#define RG_RX_HOLD_CYCE_ADDR4													(1L<<4)						
	#define RG_RX_HOLD_CYCE_ADDR3													(1L<<3)						
	#define RG_RX_HOLD_CYCE_ADDR2													(1L<<2)						
	#define RG_RX_HOLD_CYCE_ADDR1													(1L<<1)						
	#define RG_RX_HOLD_CYCE_ADDR0													(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_FREQ_MULTIPLE_ND1¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FREQ_MULTIPLE_ND1							     (BASE_TFC1_BA + 0x4C)		 
	#define RG_FREQ_MULTIPLE_DIFF_ND1_3									(1L<<11)				
	#define RG_FREQ_MULTIPLE_DIFF_ND1_2									(1L<<10)				
	#define RG_FREQ_MULTIPLE_DIFF_ND1_1									(1L<<9)					
	#define RG_FREQ_MULTIPLE_DIFF_ND1_0									(1L<<8)					


	#define RG_FREQ_MULTIPLE_MIN_ND1_7									(1L<<7)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_6									(1L<<6)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_5									(1L<<5)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_4									(1L<<4)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_3									(1L<<3)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_2									(1L<<2)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_1									(1L<<1)						
	#define RG_FREQ_MULTIPLE_MIN_ND1_0									(1L<<0)						


#define REG32_FREQ_MULTIPLE_MIN_ND1							      (BASE_TFC1_BA + 0x4C)	
#define REG32_FREQ_MULTIPLE_DIFF_ND1							    (BASE_TFC1_BA + 0x4D)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_FREQ_BIN_NUM_ND1¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FREQ_BIN_NUM_ND1							     (BASE_TFC1_BA + 0x50)		
	#define RG_FREQ_BIN_NUM_ND1_4													(1L<<4)						
	#define RG_FREQ_BIN_NUM_ND1_3													(1L<<3)						
	#define RG_FREQ_BIN_NUM_ND1_2													(1L<<2)						
	#define RG_FREQ_BIN_NUM_ND1_1													(1L<<1)						
	#define RG_FREQ_BIN_NUM_ND1_0													(1L<<0)						



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_FREQ_MULTIPLE_ND2¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FREQ_MULTIPLE_ND2							     (BASE_TFC1_BA + 0x54)		 
	#define RG_FREQ_MULTIPLE_DIFF_ND2_3									(1L<<11)				
	#define RG_FREQ_MULTIPLE_DIFF_ND2_2									(1L<<10)				
	#define RG_FREQ_MULTIPLE_DIFF_ND2_1									(1L<<9)					
	#define RG_FREQ_MULTIPLE_DIFF_ND2_0									(1L<<8)					

	#define RG_FREQ_MULTIPLE_MIN_ND2_7									(1L<<7)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_6									(1L<<6)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_5									(1L<<5)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_4									(1L<<4)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_3									(1L<<3)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_2									(1L<<2)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_1									(1L<<1)						
	#define RG_FREQ_MULTIPLE_MIN_ND2_0									(1L<<0)						

#define REG32_FREQ_MULTIPLE_MIN_ND2							      (BASE_TFC1_BA + 0x54)	
#define REG32_FREQ_MULTIPLE_DIFF_ND2							    (BASE_TFC1_BA + 0x55)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_FREQ_BIN_NUM_ND2¡G
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_FREQ_BIN_NUM_ND2							     (BASE_TFC1_BA + 0x58)		
	#define RG_FREQ_BIN_NUM_ND2_4													(1L<<4)						
	#define RG_FREQ_BIN_NUM_ND2_3													(1L<<3)						
	#define RG_FREQ_BIN_NUM_ND2_2													(1L<<2)						
	#define RG_FREQ_BIN_NUM_ND2_1													(1L<<1)						
	#define RG_FREQ_BIN_NUM_ND2_0													(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_LPF_COEFF_HALF_LEN¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_LPF_COEFF_HALF_LEN							     (BASE_TFC1_BA + 0x5C)
	#define RG_LPF_COEFF_HALF_LENGTH8									(1L<<8)						
	#define RG_LPF_COEFF_HALF_LENGTH7									(1L<<7)						
	#define RG_LPF_COEFF_HALF_LENGTH6									(1L<<6)						
	#define RG_LPF_COEFF_HALF_LENGTH5									(1L<<5)						
	#define RG_LPF_COEFF_HALF_LENGTH4									(1L<<4)						
	#define RG_LPF_COEFF_HALF_LENGTH3									(1L<<3)						
	#define RG_LPF_COEFF_HALF_LENGTH2									(1L<<2)						
	#define RG_LPF_COEFF_HALF_LENGTH1									(1L<<1)						
	#define RG_LPF_COEFF_HALF_LENGTH0									(1L<<0)						


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_LPF_IN_SGN_EXT¡G 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_LPF_IN_SGN_EXT							     (BASE_TFC1_BA + 0x60)
	#define RG_DO_ADC_NO_ND15													(1L<<15)					
	#define RG_DO_ADC_NO_ND14													(1L<<14)					
	#define RG_DO_ADC_NO_ND13													(1L<<13)					
	#define RG_DO_ADC_NO_ND12													(1L<<12)					
	#define RG_GAIN_PN_IN1														(1L<<9)						
	#define RG_GAIN_PN_IN0														(1L<<8)						

	#define RG_GAIN_LPF_IN1														(1L<<7)						
	#define RG_GAIN_LPF_IN0														(1L<<6)						
	#define RG_GAIN_DECORR_IN1												(1L<<5)						
	#define RG_GAIN_DECORR_IN0												(1L<<4)						
	#define RG_LPF_IN_SGN_EXT3												(1L<<3)						
	#define RG_LPF_IN_SGN_EXT2												(1L<<2)						
	#define RG_LPF_IN_SGN_EXT1												(1L<<1)						
	#define RG_LPF_IN_SGN_EXT0												(1L<<0)						

#define REG32_LPF_IN_SGN_EXT_ND							     (BASE_TFC1_BA + 0x61)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_STR_PN_CTRL :
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_STR_PN_CTRL							     (BASE_TFC2_BA + 0x00)
	#define RG_STR_ND														(1L<<24)						
	#define RG_STR_PN														(1L<<16)						
	#define RG_SINE_CONT_EN											(1L<<15)						
	#define DA_TRY_DOUT_15V6										(1L<<14)						
	#define DA_TRY_DOUT_15V5										(1L<<13)						
	#define DA_TRY_DOUT_15V4										(1L<<12)						
	#define DA_TRY_DOUT_15V3										(1L<<11)						
	#define DA_TRY_DOUT_15V2										(1L<<10)						
	#define DA_TRY_DOUT_15V1										(1L<<9)							
	#define DA_TRY_DOUT_15V0										(1L<<8)							

	#define DA_TEST_EN													(1L<<7)						

	#define RG_STR_AD														(1L<<5)						
	#define RG_DECORR_POWER_DOWN								(1L<<3)						
	#define RG_LPF_POWER_DOWN										(1L<<2)						
	#define RG_PN_DEMOD_POWER_DOWN							(1L<<1)						
	#define RG_MANUAL														(1L<<0)						
	
#define REG_STR_PN_CTRL_1								    (BASE_TFC2_BA + 0x01)
#define REG_STR_PN_CTRL_2								    (BASE_TFC2_BA + 0x02)
	#define START_PN_DETECT 									(1L<<0)
#define REG_STR_PN_CTRL_3								    (BASE_TFC2_BA + 0x03)
	#define START_ND_DETECT										(1L<<0)	
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT0 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT0							     (BASE_TFC2_BA + 0x04)
	#define TRNUM3_6														(1L<<30)						///< 
	#define TRNUM3_5														(1L<<29)						///< 
	#define TRNUM3_4														(1L<<28)						///< 
	#define TRNUM3_3														(1L<<27)						///< 
	#define TRNUM3_2														(1L<<26)						///<
	#define TRNUM3_1														(1L<<25)						///< 
	#define TRNUM3_0														(1L<<24)						///<
 
	#define TRNUM2_6														(1L<<22)						///< 
	#define TRNUM2_5														(1L<<21)						///< 
	#define TRNUM2_4														(1L<<20)						///< 
	#define TRNUM2_3														(1L<<19)						///< 
	#define TRNUM2_2														(1L<<18)						///<
	#define TRNUM2_1														(1L<<17)						///< 
	#define TRNUM2_0														(1L<<16)						///<
 
	#define TRNUM1_6														(1L<<14)						///< 
	#define TRNUM1_5														(1L<<13)						///< 
	#define TRNUM1_4														(1L<<12)						///< 
	#define TRNUM1_3														(1L<<11)						///< 
	#define TRNUM1_2														(1L<<10)						///<
	#define TRNUM1_1														(1L<<9)						///< 
	#define TRNUM1_0														(1L<<8)						///<
 
	#define TRNUM0_6														(1L<<6)						///< 
	#define TRNUM0_5														(1L<<5)						///< 
	#define TRNUM0_4														(1L<<4)						///< 
	#define TRNUM0_3														(1L<<3)						///< 
	#define TRNUM0_2														(1L<<2)						///<
	#define TRNUM0_1														(1L<<1)						///< 
	#define TRNUM0_0														(1L<<0)						///<
 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT1 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT1							     (BASE_TFC2_BA + 0x08)
	#define TRNUM7_6														(1L<<30)						///< 
	#define TRNUM7_5														(1L<<29)						///< 
	#define TRNUM7_4														(1L<<28)						///< 
	#define TRNUM7_3														(1L<<27)						///< 
	#define TRNUM7_2														(1L<<26)						///<
	#define TRNUM7_1														(1L<<25)						///< 
	#define TRNUM7_0														(1L<<24)						///<
 
	#define TRNUM6_6														(1L<<22)						///< 
	#define TRNUM6_5														(1L<<21)						///< 
	#define TRNUM6_4														(1L<<20)						///< 
	#define TRNUM6_3														(1L<<19)						///< 
	#define TRNUM6_2														(1L<<18)						///<
	#define TRNUM6_1														(1L<<17)						///< 
	#define TRNUM6_0														(1L<<16)						///<
 
	#define TRNUM5_6														(1L<<14)						///< 
	#define TRNUM5_5														(1L<<13)						///< 
	#define TRNUM5_4														(1L<<12)						///< 
	#define TRNUM5_3														(1L<<11)						///< 
	#define TRNUM5_2														(1L<<10)						///<
	#define TRNUM5_1														(1L<<9)						///< 
	#define TRNUM5_0														(1L<<8)						///<
 
	#define TRNUM4_6														(1L<<6)						///< 
	#define TRNUM4_5														(1L<<5)						///< 
	#define TRNUM4_4														(1L<<4)						///< 
	#define TRNUM4_3														(1L<<3)						///< 
	#define TRNUM4_2														(1L<<2)						///<
	#define TRNUM4_1														(1L<<1)						///< 
	#define TRNUM4_0														(1L<<0)						///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT2 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT2							     (BASE_TFC2_BA + 0x0C)
	#define TRNUM11_6														(1L<<30)						///< 
	#define TRNUM11_5														(1L<<29)						///< 
	#define TRNUM11_4														(1L<<28)						///< 
	#define TRNUM11_3														(1L<<27)						///< 
	#define TRNUM11_2														(1L<<26)						///<
	#define TRNUM11_1														(1L<<25)						///< 
	#define TRNUM11_0														(1L<<24)						///<
 
	#define TRNUM10_6														(1L<<22)						///< 
	#define TRNUM10_5														(1L<<21)						///< 
	#define TRNUM10_4														(1L<<20)						///< 
	#define TRNUM10_3														(1L<<19)						///< 
	#define TRNUM10_2														(1L<<18)						///<
	#define TRNUM10_1														(1L<<17)						///< 
	#define TRNUM10_0														(1L<<16)						///<
 
	#define TRNUM9_6														(1L<<14)						///< 
	#define TRNUM9_5														(1L<<13)						///< 
	#define TRNUM9_4														(1L<<12)						///< 
	#define TRNUM9_3														(1L<<11)						///< 
	#define TRNUM9_2														(1L<<10)						///<
	#define TRNUM9_1														(1L<<9)						///< 
	#define TRNUM9_0														(1L<<8)						///<
 
	#define TRNUM8_6														(1L<<6)						///< 
	#define TRNUM8_5														(1L<<5)						///< 
	#define TRNUM8_4														(1L<<4)						///< 
	#define TRNUM8_3														(1L<<3)						///< 
	#define TRNUM8_2														(1L<<2)						///<
	#define TRNUM8_1														(1L<<1)						///< 
	#define TRNUM8_0														(1L<<0)						///<


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT3 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT3							     (BASE_TFC2_BA + 0x10)
	#define TRNUM15_6														(1L<<30)						///< 
	#define TRNUM15_5														(1L<<29)						///< 
	#define TRNUM15_4														(1L<<28)						///< 
	#define TRNUM15_3														(1L<<27)						///< 
	#define TRNUM15_2														(1L<<26)						///<
	#define TRNUM15_1														(1L<<25)						///< 
	#define TRNUM15_0														(1L<<24)						///<
 
	#define TRNUM14_6														(1L<<22)						///< 
	#define TRNUM14_5														(1L<<21)						///< 
	#define TRNUM14_4														(1L<<20)						///< 
	#define TRNUM14_3														(1L<<19)						///< 
	#define TRNUM14_2														(1L<<18)						///<
	#define TRNUM14_1														(1L<<17)						///< 
	#define TRNUM14_0														(1L<<16)						///<
 
	#define TRNUM13_6														(1L<<14)						///< 
	#define TRNUM13_5														(1L<<13)						///< 
	#define TRNUM13_4														(1L<<12)						///< 
	#define TRNUM13_3														(1L<<11)						///< 
	#define TRNUM13_2														(1L<<10)						///<
	#define TRNUM13_1														(1L<<9)						///< 
	#define TRNUM13_0														(1L<<8)						///<
 
	#define TRNUM12_6														(1L<<6)						///< 
	#define TRNUM12_5														(1L<<5)						///< 
	#define TRNUM12_4														(1L<<4)						///< 
	#define TRNUM12_3														(1L<<3)						///< 
	#define TRNUM12_2														(1L<<2)						///<
	#define TRNUM12_1														(1L<<1)						///< 
	#define TRNUM12_0														(1L<<0)						///<



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT4 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT4							     (BASE_TFC2_BA + 0x14)
	#define TRNUM19_6														(1L<<30)						///< 
	#define TRNUM19_5														(1L<<29)						///< 
	#define TRNUM19_4														(1L<<28)						///< 
	#define TRNUM19_3														(1L<<27)						///< 
	#define TRNUM19_2														(1L<<26)						///<
	#define TRNUM19_1														(1L<<25)						///< 
	#define TRNUM19_0														(1L<<24)						///<
 
	#define TRNUM18_6														(1L<<22)						///< 
	#define TRNUM18_5														(1L<<21)						///< 
	#define TRNUM18_4														(1L<<20)						///< 
	#define TRNUM18_3														(1L<<19)						///< 
	#define TRNUM18_2														(1L<<18)						///<
	#define TRNUM18_1														(1L<<17)						///< 
	#define TRNUM18_0														(1L<<16)						///<
 
	#define TRNUM17_6														(1L<<14)						///< 
	#define TRNUM17_5														(1L<<13)						///< 
	#define TRNUM17_4														(1L<<12)						///< 
	#define TRNUM17_3														(1L<<11)						///< 
	#define TRNUM17_2														(1L<<10)						///<
	#define TRNUM17_1														(1L<<9)						///< 
	#define TRNUM17_0														(1L<<8)						///<
 
	#define TRNUM16_6														(1L<<6)						///< 
	#define TRNUM16_5														(1L<<5)						///< 
	#define TRNUM16_4														(1L<<4)						///< 
	#define TRNUM16_3														(1L<<3)						///< 
	#define TRNUM16_2														(1L<<2)						///<
	#define TRNUM16_1														(1L<<1)						///< 
	#define TRNUM16_0														(1L<<0)						///<


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUM_OUT5 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUM_OUT5							     (BASE_TFC2_BA + 0x18)
	#define TRNUM20_6														(1L<<6)						///< 
	#define TRNUM20_5														(1L<<5)						///< 
	#define TRNUM20_4														(1L<<4)						///< 
	#define TRNUM20_3														(1L<<3)						///< 
	#define TRNUM20_2														(1L<<2)						///<
	#define TRNUM20_1														(1L<<1)						///< 
	#define TRNUM20_0														(1L<<0)						///<




///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT0 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT0							     (BASE_TFC2_BA + 0x24)
	#define TRNUMB3_6														(1L<<30)						///< 
	#define TRNUMB3_5														(1L<<29)						///< 
	#define TRNUMB3_4														(1L<<28)						///< 
	#define TRNUMB3_3														(1L<<27)						///< 
	#define TRNUMB3_2														(1L<<26)						///<
	#define TRNUMB3_1														(1L<<25)						///< 
	#define TRNUMB3_0														(1L<<24)						///<
 
	#define TRNUMB2_6														(1L<<22)						///< 
	#define TRNUMB2_5														(1L<<21)						///< 
	#define TRNUMB2_4														(1L<<20)						///< 
	#define TRNUMB2_3														(1L<<19)						///< 
	#define TRNUMB2_2														(1L<<18)						///<
	#define TRNUMB2_1														(1L<<17)						///< 
	#define TRNUMB2_0														(1L<<16)						///<
 
	#define TRNUMB1_6														(1L<<14)						///< 
	#define TRNUMB1_5														(1L<<13)						///< 
	#define TRNUMB1_4														(1L<<12)						///< 
	#define TRNUMB1_3														(1L<<11)						///< 
	#define TRNUMB1_2														(1L<<10)						///<
	#define TRNUMB1_1														(1L<<9)						///< 
	#define TRNUMB1_0														(1L<<8)						///<
 
	#define TRNUMB0_6														(1L<<6)						///< 
	#define TRNUMB0_5														(1L<<5)						///< 
	#define TRNUMB0_4														(1L<<4)						///< 
	#define TRNUMB0_3														(1L<<3)						///< 
	#define TRNUMB0_2														(1L<<2)						///<
	#define TRNUMB0_1														(1L<<1)						///< 
	#define TRNUMB0_0														(1L<<0)						///<
 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT1 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT1							     (BASE_TFC2_BA + 0x28)
	#define TRNUMB7_6														(1L<<30)						///< 
	#define TRNUMB7_5														(1L<<29)						///< 
	#define TRNUMB7_4														(1L<<28)						///< 
	#define TRNUMB7_3														(1L<<27)						///< 
	#define TRNUMB7_2														(1L<<26)						///<
	#define TRNUMB7_1														(1L<<25)						///< 
	#define TRNUMB7_0														(1L<<24)						///<
 
	#define TRNUMB6_6														(1L<<22)						///< 
	#define TRNUMB6_5														(1L<<21)						///< 
	#define TRNUMB6_4														(1L<<20)						///< 
	#define TRNUMB6_3														(1L<<19)						///< 
	#define TRNUMB6_2														(1L<<18)						///<
	#define TRNUMB6_1														(1L<<17)						///< 
	#define TRNUMB6_0														(1L<<16)						///<
 
	#define TRNUMB5_6														(1L<<14)						///< 
	#define TRNUMB5_5														(1L<<13)						///< 
	#define TRNUMB5_4														(1L<<12)						///< 
	#define TRNUMB5_3														(1L<<11)						///< 
	#define TRNUMB5_2														(1L<<10)						///<
	#define TRNUMB5_1														(1L<<9)						///< 
	#define TRNUMB5_0														(1L<<8)						///<
 
	#define TRNUMB4_6														(1L<<6)						///< 
	#define TRNUMB4_5														(1L<<5)						///< 
	#define TRNUMB4_4														(1L<<4)						///< 
	#define TRNUMB4_3														(1L<<3)						///< 
	#define TRNUMB4_2														(1L<<2)						///<
	#define TRNUMB4_1														(1L<<1)						///< 
	#define TRNUMB4_0														(1L<<0)						///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT2 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT2							     (BASE_TFC2_BA + 0x2C)
	#define TRNUMB11_6														(1L<<30)						///< 
	#define TRNUMB11_5														(1L<<29)						///< 
	#define TRNUMB11_4														(1L<<28)						///< 
	#define TRNUMB11_3														(1L<<27)						///< 
	#define TRNUMB11_2														(1L<<26)						///<
	#define TRNUMB11_1														(1L<<25)						///< 
	#define TRNUMB11_0														(1L<<24)						///<
 
	#define TRNUMB10_6														(1L<<22)						///< 
	#define TRNUMB10_5														(1L<<21)						///< 
	#define TRNUMB10_4														(1L<<20)						///< 
	#define TRNUMB10_3														(1L<<19)						///< 
	#define TRNUMB10_2														(1L<<18)						///<
	#define TRNUMB10_1														(1L<<17)						///< 
	#define TRNUMB10_0														(1L<<16)						///<
 
	#define TRNUMB9_6														(1L<<14)						///< 
	#define TRNUMB9_5														(1L<<13)						///< 
	#define TRNUMB9_4														(1L<<12)						///< 
	#define TRNUMB9_3														(1L<<11)						///< 
	#define TRNUMB9_2														(1L<<10)						///<
	#define TRNUMB9_1														(1L<<9)						///< 
	#define TRNUMB9_0														(1L<<8)						///<
 
	#define TRNUMB8_6														(1L<<6)						///< 
	#define TRNUMB8_5														(1L<<5)						///< 
	#define TRNUMB8_4														(1L<<4)						///< 
	#define TRNUMB8_3														(1L<<3)						///< 
	#define TRNUMB8_2														(1L<<2)						///<
	#define TRNUMB8_1														(1L<<1)						///< 
	#define TRNUMB8_0														(1L<<0)						///<


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT3 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT3							     (BASE_TFC2_BA + 0x30)
	#define TRNUMB15_6														(1L<<30)						///< 
	#define TRNUMB15_5														(1L<<29)						///< 
	#define TRNUMB15_4														(1L<<28)						///< 
	#define TRNUMB15_3														(1L<<27)						///< 
	#define TRNUMB15_2														(1L<<26)						///<
	#define TRNUMB15_1														(1L<<25)						///< 
	#define TRNUMB15_0														(1L<<24)						///<
 
	#define TRNUMB14_6														(1L<<22)						///< 
	#define TRNUMB14_5														(1L<<21)						///< 
	#define TRNUMB14_4														(1L<<20)						///< 
	#define TRNUMB14_3														(1L<<19)						///< 
	#define TRNUMB14_2														(1L<<18)						///<
	#define TRNUMB14_1														(1L<<17)						///< 
	#define TRNUMB14_0														(1L<<16)						///<
 
	#define TRNUMB13_6														(1L<<14)						///< 
	#define TRNUMB13_5														(1L<<13)						///< 
	#define TRNUMB13_4														(1L<<12)						///< 
	#define TRNUMB13_3														(1L<<11)						///< 
	#define TRNUMB13_2														(1L<<10)						///<
	#define TRNUMB13_1														(1L<<9)						///< 
	#define TRNUMB13_0														(1L<<8)						///<
 
	#define TRNUMB12_6														(1L<<6)						///< 
	#define TRNUMB12_5														(1L<<5)						///< 
	#define TRNUMB12_4														(1L<<4)						///< 
	#define TRNUMB12_3														(1L<<3)						///< 
	#define TRNUMB12_2														(1L<<2)						///<
	#define TRNUMB12_1														(1L<<1)						///< 
	#define TRNUMB12_0														(1L<<0)						///<



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT4 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT4							     (BASE_TFC2_BA + 0x34)
	#define TRNUMB19_6														(1L<<30)						///< 
	#define TRNUMB19_5														(1L<<29)						///< 
	#define TRNUMB19_4														(1L<<28)						///< 
	#define TRNUMB19_3														(1L<<27)						///< 
	#define TRNUMB19_2														(1L<<26)						///<
	#define TRNUMB19_1														(1L<<25)						///< 
	#define TRNUMB19_0														(1L<<24)						///<
 
	#define TRNUMB18_6														(1L<<22)						///< 
	#define TRNUMB18_5														(1L<<21)						///< 
	#define TRNUMB18_4														(1L<<20)						///< 
	#define TRNUMB18_3														(1L<<19)						///< 
	#define TRNUMB18_2														(1L<<18)						///<
	#define TRNUMB18_1														(1L<<17)						///< 
	#define TRNUMB18_0														(1L<<16)						///<
 
	#define TRNUMB17_6														(1L<<14)						///< 
	#define TRNUMB17_5														(1L<<13)						///< 
	#define TRNUMB17_4														(1L<<12)						///< 
	#define TRNUMB17_3														(1L<<11)						///< 
	#define TRNUMB17_2														(1L<<10)						///<
	#define TRNUMB17_1														(1L<<9)						///< 
	#define TRNUMB17_0														(1L<<8)						///<
 
	#define TRNUMB16_6														(1L<<6)						///< 
	#define TRNUMB16_5														(1L<<5)						///< 
	#define TRNUMB16_4														(1L<<4)						///< 
	#define TRNUMB16_3														(1L<<3)						///< 
	#define TRNUMB16_2														(1L<<2)						///<
	#define TRNUMB16_1														(1L<<1)						///< 
	#define TRNUMB16_0														(1L<<0)						///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TRNUMB_OUT5 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TRNUMB_OUT5							     (BASE_TFC2_BA + 0x38)
	#define TRNUMB20_6														(1L<<6)						///< 
	#define TRNUMB20_5														(1L<<5)						///< 
	#define TRNUMB20_4														(1L<<4)						///< 
	#define TRNUMB20_3														(1L<<3)						///< 
	#define TRNUMB20_2														(1L<<2)						///<
	#define TRNUMB20_1														(1L<<1)						///< 
	#define TRNUMB20_0														(1L<<0)						///<









///===========================================================///
///                                                           ///
///  REGISTER CLASS #7:  TRACE CONTROL REGISTERS            ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT0 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT0							     (BASE_TFC2_BA + 0x40) 
	#define TR_SEL_OUT3_5														(1L<<29)						///< 
	#define TR_SEL_OUT3_4														(1L<<28)						///< 
	#define TR_SEL_OUT3_3														(1L<<27)						///< 
	#define TR_SEL_OUT3_2														(1L<<26)						///<
	#define TR_SEL_OUT3_1														(1L<<25)						///< 
	#define TR_SEL_OUT3_0														(1L<<24)						///<
 
	#define TR_SEL_OUT2_5														(1L<<21)						///< 
	#define TR_SEL_OUT2_4														(1L<<20)						///< 
	#define TR_SEL_OUT2_3														(1L<<19)						///< 
	#define TR_SEL_OUT2_2														(1L<<18)						///<
	#define TR_SEL_OUT2_1														(1L<<17)						///< 
	#define TR_SEL_OUT2_0														(1L<<16)						///<
 
	#define TR_SEL_OUT1_5														(1L<<13)						///< 
	#define TR_SEL_OUT1_4														(1L<<12)						///< 
	#define TR_SEL_OUT1_3														(1L<<11)						///< 
	#define TR_SEL_OUT1_2														(1L<<10)						///<
	#define TR_SEL_OUT1_1														(1L<<9)							///< 
	#define TR_SEL_OUT1_0														(1L<<8)							///<
 
	#define TR_SEL_OUT0_5														(1L<<5)							///< 
	#define TR_SEL_OUT0_4														(1L<<4)							///< 
	#define TR_SEL_OUT0_3														(1L<<3)							///< 
	#define TR_SEL_OUT0_2														(1L<<2)							///<
	#define TR_SEL_OUT0_1														(1L<<1)							///< 
	#define TR_SEL_OUT0_0														(1L<<0)							///<
 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT1 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT1							     (BASE_TFC2_BA + 0x44)

	#define TR_SEL_OUT7_5														(1L<<29)						///< 
	#define TR_SEL_OUT7_4														(1L<<28)						///< 
	#define TR_SEL_OUT7_3														(1L<<27)						///< 
	#define TR_SEL_OUT7_2														(1L<<26)						///<
	#define TR_SEL_OUT7_1														(1L<<25)						///< 
	#define TR_SEL_OUT7_0														(1L<<24)						///<
 
	#define TR_SEL_OUT6_5														(1L<<21)						///< 
	#define TR_SEL_OUT6_4														(1L<<20)						///< 
	#define TR_SEL_OUT6_3														(1L<<19)						///< 
	#define TR_SEL_OUT6_2														(1L<<18)						///<
	#define TR_SEL_OUT6_1														(1L<<17)						///< 
	#define TR_SEL_OUT6_0														(1L<<16)						///<
 
	#define TR_SEL_OUT5_5														(1L<<13)						///< 
	#define TR_SEL_OUT5_4														(1L<<12)						///< 
	#define TR_SEL_OUT5_3														(1L<<11)						///< 
	#define TR_SEL_OUT5_2														(1L<<10)						///<
	#define TR_SEL_OUT5_1														(1L<<9)							///< 
	#define TR_SEL_OUT5_0														(1L<<8)							///<
 
	#define TR_SEL_OUT4_5														(1L<<5)							///< 
	#define TR_SEL_OUT4_4														(1L<<4)							///< 
	#define TR_SEL_OUT4_3														(1L<<3)							///< 
	#define TR_SEL_OUT4_2														(1L<<2)							///<
	#define TR_SEL_OUT4_1														(1L<<1)							///< 
	#define TR_SEL_OUT4_0														(1L<<0)							///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT2 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT2							     (BASE_TFC2_BA + 0x48) 
	#define TR_SEL_OUT11_5														(1L<<29)						///< 
	#define TR_SEL_OUT11_4														(1L<<28)						///< 
	#define TR_SEL_OUT11_3														(1L<<27)						///< 
	#define TR_SEL_OUT11_2														(1L<<26)						///<
	#define TR_SEL_OUT11_1														(1L<<25)						///< 
	#define TR_SEL_OUT11_0														(1L<<24)						///<
 
	#define TR_SEL_OUT10_5														(1L<<21)						///< 
	#define TR_SEL_OUT10_4														(1L<<20)						///< 
	#define TR_SEL_OUT10_3														(1L<<19)						///< 
	#define TR_SEL_OUT10_2														(1L<<18)						///<
	#define TR_SEL_OUT10_1														(1L<<17)						///< 
	#define TR_SEL_OUT10_0														(1L<<16)						///<
 
	#define TR_SEL_OUT9_5														(1L<<13)						///< 
	#define TR_SEL_OUT9_4														(1L<<12)						///< 
	#define TR_SEL_OUT9_3														(1L<<11)						///< 
	#define TR_SEL_OUT9_2														(1L<<10)						///<
	#define TR_SEL_OUT9_1														(1L<<9)						///< 
	#define TR_SEL_OUT9_0														(1L<<8)						///<
 
	#define TR_SEL_OUT8_5														(1L<<5)						///< 
	#define TR_SEL_OUT8_4														(1L<<4)						///< 
	#define TR_SEL_OUT8_3														(1L<<3)						///< 
	#define TR_SEL_OUT8_2														(1L<<2)						///<
	#define TR_SEL_OUT8_1														(1L<<1)						///< 
	#define TR_SEL_OUT8_0														(1L<<0)						///<


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT3 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT3							     (BASE_TFC2_BA + 0x4C) 
	#define TR_SEL_OUT15_5														(1L<<29)						///< 
	#define TR_SEL_OUT15_4														(1L<<28)						///< 
	#define TR_SEL_OUT15_3														(1L<<27)						///< 
	#define TR_SEL_OUT15_2														(1L<<26)						///<
	#define TR_SEL_OUT15_1														(1L<<25)						///< 
	#define TR_SEL_OUT15_0														(1L<<24)						///<
 
	#define TR_SEL_OUT14_5														(1L<<21)						///< 
	#define TR_SEL_OUT14_4														(1L<<20)						///< 
	#define TR_SEL_OUT14_3														(1L<<19)						///< 
	#define TR_SEL_OUT14_2														(1L<<18)						///<
	#define TR_SEL_OUT14_1														(1L<<17)						///< 
	#define TR_SEL_OUT14_0														(1L<<16)						///<
 
	#define TR_SEL_OUT13_5														(1L<<13)						///< 
	#define TR_SEL_OUT13_4														(1L<<12)						///< 
	#define TR_SEL_OUT13_3														(1L<<11)						///< 
	#define TR_SEL_OUT13_2														(1L<<10)						///<
	#define TR_SEL_OUT13_1														(1L<<9)						///< 
	#define TR_SEL_OUT13_0														(1L<<8)						///<
 
	#define TR_SEL_OUT12_5														(1L<<5)						///< 
	#define TR_SEL_OUT12_4														(1L<<4)						///< 
	#define TR_SEL_OUT12_3														(1L<<3)						///< 
	#define TR_SEL_OUT12_2														(1L<<2)						///<
	#define TR_SEL_OUT12_1														(1L<<1)						///< 
	#define TR_SEL_OUT12_0														(1L<<0)						///<



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT4 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT4							     (BASE_TFC2_BA + 0x50) 
	#define TR_SEL_OUT19_5														(1L<<29)						///< 
	#define TR_SEL_OUT19_4														(1L<<28)						///< 
	#define TR_SEL_OUT19_3														(1L<<27)						///< 
	#define TR_SEL_OUT19_2														(1L<<26)						///<
	#define TR_SEL_OUT19_1														(1L<<25)						///< 
	#define TR_SEL_OUT19_0														(1L<<24)						///<
  
	#define TR_SEL_OUT18_5														(1L<<21)						///< 
	#define TR_SEL_OUT18_4														(1L<<20)						///< 
	#define TR_SEL_OUT18_3														(1L<<19)						///< 
	#define TR_SEL_OUT18_2														(1L<<18)						///<
	#define TR_SEL_OUT18_1														(1L<<17)						///< 
	#define TR_SEL_OUT18_0														(1L<<16)						///<
 
	#define TR_SEL_OUT17_5														(1L<<13)						///< 
	#define TR_SEL_OUT17_4														(1L<<12)						///< 
	#define TR_SEL_OUT17_3														(1L<<11)						///< 
	#define TR_SEL_OUT17_2														(1L<<10)						///<
	#define TR_SEL_OUT17_1														(1L<<9)						///< 
	#define TR_SEL_OUT17_0														(1L<<8)						///<
  
	#define TR_SEL_OUT16_5														(1L<<5)						///< 
	#define TR_SEL_OUT16_4														(1L<<4)						///< 
	#define TR_SEL_OUT16_3														(1L<<3)						///< 
	#define TR_SEL_OUT16_2														(1L<<2)						///<
	#define TR_SEL_OUT16_1														(1L<<1)						///< 
	#define TR_SEL_OUT16_0														(1L<<0)						///<


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT5 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT5							     (BASE_TFC2_BA + 0x54) 
	#define TR_SEL_OUT23_5														(1L<<29)						///< 
	#define TR_SEL_OUT23_4														(1L<<28)						///< 
	#define TR_SEL_OUT23_3														(1L<<27)						///< 
	#define TR_SEL_OUT23_2														(1L<<26)						///<
	#define TR_SEL_OUT23_1														(1L<<25)						///< 
	#define TR_SEL_OUT23_0														(1L<<24)						///<
  
	#define TR_SEL_OUT22_5														(1L<<21)						///< 
	#define TR_SEL_OUT22_4														(1L<<20)						///< 
	#define TR_SEL_OUT22_3														(1L<<19)						///< 
	#define TR_SEL_OUT22_2														(1L<<18)						///<
	#define TR_SEL_OUT22_1														(1L<<17)						///< 
	#define TR_SEL_OUT22_0														(1L<<16)						///<
 
	#define TR_SEL_OUT21_5														(1L<<13)						///< 
	#define TR_SEL_OUT21_4														(1L<<12)						///< 
	#define TR_SEL_OUT21_3														(1L<<11)						///< 
	#define TR_SEL_OUT21_2														(1L<<10)						///<
	#define TR_SEL_OUT21_1														(1L<<9)						///< 
	#define TR_SEL_OUT21_0														(1L<<8)						///<
  
	#define TR_SEL_OUT20_5														(1L<<5)						///< 
	#define TR_SEL_OUT20_4														(1L<<4)						///< 
	#define TR_SEL_OUT20_3														(1L<<3)						///< 
	#define TR_SEL_OUT20_2														(1L<<2)						///<
	#define TR_SEL_OUT20_1														(1L<<1)						///< 
	#define TR_SEL_OUT20_0														(1L<<0)						///<

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT6 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT6							     (BASE_TFC2_BA + 0x58) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT7 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT7							     (BASE_TFC2_BA + 0x5C) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT8 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT8							     (BASE_TFC2_BA + 0x60) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT9 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT9							     (BASE_TFC2_BA + 0x64) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT10 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT10							     (BASE_TFC2_BA + 0x68) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT11 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT11							     (BASE_TFC2_BA + 0x6C) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT12 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT12							     (BASE_TFC2_BA + 0x70) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT13 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT13							     (BASE_TFC2_BA + 0x74) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT14 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT14							     (BASE_TFC2_BA + 0x78) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT15 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT15							     (BASE_TFC2_BA + 0x7C) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT16 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT16							     (BASE_TFC2_BA + 0x80) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT17 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT17							     (BASE_TFC2_BA + 0x84) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT18 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT18							     (BASE_TFC2_BA + 0x88) 

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TR_SEL_OUT_OUT19 : 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///
#define REG32_TR_SEL_OUT_OUT19							     (BASE_TFC2_BA + 0x8C) 




///===========================================================///
///                                                           ///
///  REGISTER CLASS #9:   ANALOG CONTROL REGISTERS            ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_CHG_PUMP -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_CHG_PUMP                    (BASE_ANA_BA + 0x00)
  #define CHG_FREQ2                         (1L<<6)           
  #define CHG_FREQ1                         (1L<<5)           
  #define CHG_FREQ0                         (1L<<4)           
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_8    (0<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_9    (1<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_10   (2<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_11   (3<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_12   (4<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_14   (5<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_15   (6<<4)          
      #define CHG_FREQ_SYS_CLK_DIV_2_EXP_16   (7<<4)          
  
  #define RG_OPT_VCPEXTB_18V                (1L<<2)           
  
  #define RG_ENCP_18V                       (1L<<0)           

	#define RG_CHGPUMP_EN_BIT                 (RG_OPT_VCPEXTB_18V|RG_ENCP_18V)
	#define RG_CHGPUMP_ENABLE                 (0x05)	
	#define RG_CHGPUMP_DISABLE                (0x04)	

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_VOLT_CTRL 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_VOLT_CTRL                    (BASE_ANA_BA + 0x10)
  #define RG_PWD_LVD         (1<<7)           
  #define RG_SEL_LVD1        (1<<6)           
  #define RG_SEL_LVD0        (1<<5)           

  #define RG_LVDWKEN         (1<<1)           
  #define RG_LVD_STATUS      (1<<0)           



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_BIAS --> BIAS Control Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_BIAS                    (BASE_ANA_BA + 0x14)
  #define RG_PWD_BIAS33_15V										(1<<7)           ///<  
  #define RG_OPT_BIAS33_IB5U_OP1_15V2					(1<<6)           ///<  
  #define RG_OPT_BIAS33_IB5U_OP1_15V1					(1<<5)           ///<  
  #define RG_OPT_BIAS33_IB5U_OP1_15V0					(1<<4)           ///<  
  #define RG_PWD_BIAS66_15V  									(1<<3)           ///<  
  #define RG_OPT_BIAS66_IB5U_OP1_15V2					(1<<2)           ///<  
  #define RG_OPT_BIAS66_IB5U_OP1_15V1					(1<<1)           ///<  
  #define RG_OPT_BIAS66_IB5U_OP1_15V0					(1<<0)           ///<  
	
	#define BIAS_POWER_DOWN							(0xBB)
	#define BIAS_POWER_ON								(0x33)	


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_VBG --> BIAS Control Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_VBG                    (BASE_ANA_BA + 0x18)
 
  #define RG_PWD_REG18_15V  					(1<<4)           ///<  
 
  #define RG_PWD_VBGBUFF_15V					(1<<0)           ///<  


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_LVD_IF -->  : Protection Register
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_LVD_IF                    (BASE_ANA_BA + 0x50)
  #define LVD_IF                       (1L<<0)        

///===========================================================///
///                                                           ///
///  REGISTER CLASS #10:   ADC CONTROL REGISTERS              ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL0 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL0                    (BASE_ADC_BA + 0x00)
  #define RG_RX_FRONT_SCALE_15V2             (1L<<6)          ///<  
  #define RG_RX_FRONT_SCALE_15V1             (1L<<5)          ///< 
  #define RG_RX_FRONT_SCALE_15V0             (1L<<4)          ///< 

  #define RG_OPT_TR_DRIVE_15V1               (1L<<1)           ///< 
  #define RG_OPT_TR_DRIVE_15V0               (1L<<0)           ///<     

	#define AD_CTRL0_FREQHOP_SETUP   (RG_RX_FRONT_SCALE_15V2|RG_RX_FRONT_SCALE_15V1|RG_RX_FRONT_SCALE_15V0)

	
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL1 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL1                    (BASE_ADC_BA + 0x04)
  #define RG_RX_LPF_GAIN_15V2                (1L<<6)           ///< 
  #define RG_RX_LPF_GAIN_15V1                (1L<<5)           ///<     
  #define RG_RX_LPF_GAIN_15V0                (1L<<4)           ///< 


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL2 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL2                    (BASE_ADC_BA + 0x08)
  #define RG_ADC_PWD_15V15                     (1L<<15)         
  #define RG_ADC_PWD_15V14                     (1L<<14)         
  #define RG_ADC_PWD_15V13                     (1L<<13)         
  #define RG_ADC_PWD_15V12                     (1L<<12)         
  #define RG_ADC_PWD_15V11                     (1L<<11)         
  #define RG_ADC_PWD_15V10                     (1L<<10)         
  #define RG_ADC_PWD_15V9                     (1L<<9)           
  #define RG_ADC_PWD_15V8                     (1L<<8)           
  #define RG_ADC_PWD_15V7                     (1L<<7)           
  #define RG_ADC_PWD_15V6                     (1L<<6)           
  #define RG_ADC_PWD_15V5                     (1L<<5)           
  #define RG_ADC_PWD_15V4                     (1L<<4)           
  #define RG_ADC_PWD_15V3                     (1L<<3)           
  #define RG_ADC_PWD_15V2                     (1L<<2)           
  #define RG_ADC_PWD_15V1                     (1L<<1)           
  #define RG_ADC_PWD_15V0                     (1L<<0)           



  #define ALL_ADC_POWER_DOWN                   (0xFFFF)
  #define ALL_ADC_POWER_ON                     (0x0000)



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL4 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL4                    (BASE_ADC_BA + 0x10)
  #define RG_RX_LPF_PWD_15V15                     (1L<<15)         
  #define RG_RX_LPF_PWD_15V14                     (1L<<14)         
  #define RG_RX_LPF_PWD_15V13                     (1L<<13)         
  #define RG_RX_LPF_PWD_15V12                     (1L<<12)         
  #define RG_RX_LPF_PWD_15V11                     (1L<<11)         
  #define RG_RX_LPF_PWD_15V10                     (1L<<10)         
  #define RG_RX_LPF_PWD_15V9                     (1L<<9)           
  #define RG_RX_LPF_PWD_15V8                     (1L<<8)           
  #define RG_RX_LPF_PWD_15V7                     (1L<<7)           
  #define RG_RX_LPF_PWD_15V6                     (1L<<6)           
  #define RG_RX_LPF_PWD_15V5                     (1L<<5)           
  #define RG_RX_LPF_PWD_15V4                     (1L<<4)           
  #define RG_RX_LPF_PWD_15V3                     (1L<<3)           
  #define RG_RX_LPF_PWD_15V2                     (1L<<2)           
  #define RG_RX_LPF_PWD_15V1                     (1L<<1)           
  #define RG_RX_LPF_PWD_15V0                     (1L<<0)           



  #define ALL_RX_LPF_POWER_ON                     (0x0000)
  #define ALL_RX_LPF_POWER_DOWN                   (0xFFFF)





///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL5 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL5                    (BASE_ADC_BA + 0x14)
  #define RG_IOPSEL_RXLPF_15V2                      (1<<6)           ///< 
  #define RG_IOPSEL_RXLPF_15V1                      (1<<5)           ///< 
  #define RG_IOPSEL_RXLPF_15V0                      (1<<4)           ///< 


  #define RG_IOPSEL_FRONT_15V2                      (1<<2)           ///< 
  #define RG_IOPSEL_FRONT_15V1                      (1<<1)           ///< 
  #define RG_IOPSEL_FRONT_15V0                     	(1<<0)           ///< 

  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL6 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL6                    (BASE_ADC_BA + 0x18)
  #define RG_IOPSEL_SARBUF_15V2                      (1<<6)           ///< 
  #define RG_IOPSEL_SARBUF_15V1                      (1<<5)           ///< 
  #define RG_IOPSEL_SARBUF_15V0                    	 (1<<4)           ///< 
  



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL8 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL8                    (BASE_ADC_BA + 0x20)
  #define RG_ADREG_RSTB_15V        					(1<<0)           ///< 


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL9 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL9                    (BASE_ADC_BA + 0x24)
  #define RG_RX_FRONT_PWD_15V15                     (1L<<15)          
  #define RG_RX_FRONT_PWD_15V14                     (1L<<14)          
  #define RG_RX_FRONT_PWD_15V13                     (1L<<13)          
  #define RG_RX_FRONT_PWD_15V12                     (1L<<12)          
  #define RG_RX_FRONT_PWD_15V11                     (1L<<11)          
  #define RG_RX_FRONT_PWD_15V10                     (1L<<10)          
  #define RG_RX_FRONT_PWD_15V9                     (1L<<9)           
  #define RG_RX_FRONT_PWD_15V8                     (1L<<8)           
  #define RG_RX_FRONT_PWD_15V7                     (1L<<7)           
  #define RG_RX_FRONT_PWD_15V6                     (1L<<6)           
  #define RG_RX_FRONT_PWD_15V5                     (1L<<5)           
  #define RG_RX_FRONT_PWD_15V4                     (1L<<4)           
  #define RG_RX_FRONT_PWD_15V3                     (1L<<3)           
  #define RG_RX_FRONT_PWD_15V2                     (1L<<2)           
  #define RG_RX_FRONT_PWD_15V1                     (1L<<1)           
  #define RG_RX_FRONT_PWD_15V0                     (1L<<0)           

  #define ALL_RX_FRONT_POWER_ON                     (0x0000)
  #define ALL_RX_FRONT_POWER_DOWN                   (0xFFFF)
  
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL10 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL10                    (BASE_ADC_BA + 0x28)
  #define RG_PWD_VDD_VDET_15V                  			(1L<<7)   ///< 
  #define RG_VDET_VDD_SEL66_15V                     (1L<<6)   ///< 
  #define RG_RX_FRONT_MODE_15V1                  		(1L<<5)   ///< 
  #define RG_RX_FRONT_MODE_15V0                     (1L<<4)   ///<  
  
  #define RG_RX_LPF_C4_EN_15V                       (1L<<2)   ///<                     			
  #define RG_RX_FRONT_RST_15V                       (1L<<1)   ///<    
  #define RG_RX_VCMBUF_PWD_15V                      (1L<<0)   ///<   

	#define ADC10_MODE_BIT	 													(0x30) 
	#define ADC10_BUFF_MODE                       		(0x00) 
	#define ADC10_SCAL_MODE                       		(0x10) 

#define AD_CTRL10_FREQHOP_SETUP				(RG_PWD_VDD_VDET_15V|RG_RX_FRONT_MODE_15V0)
#define AD_CTRL10_INITIAL_SETUP				(RG_PWD_VDD_VDET_15V|RG_RX_FRONT_MODE_15V0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL11 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL11                    (BASE_ADC_BA + 0x2C)
  #define RG_RX_SELF_EN_15V                  (1L<<1)     ///< 


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL12 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL12                    (BASE_ADC_BA + 0x30)
  #define RG_RX_FRONT_TEST_EN_15V                     (1L<<15)           
  #define RG_RX_FRONT_TEST_EN_15V14                     (1L<<14)         
  #define RG_RX_FRONT_TEST_EN_15V13                     (1L<<13)         
  #define RG_RX_FRONT_TEST_EN_15V12                     (1L<<12)         
  #define RG_RX_FRONT_TEST_EN_15V11                     (1L<<11)         
  #define RG_RX_FRONT_TEST_EN_15V10                     (1L<<10)         
  #define RG_RX_FRONT_TEST_EN_15V9                     (1L<<9)           
  #define RG_RX_FRONT_TEST_EN_15V8                     (1L<<8)           
  #define RG_RX_FRONT_TEST_EN_15V7                     (1L<<7)          
  #define RG_RX_FRONT_TEST_EN_15V6                     (1L<<6)          
  #define RG_RX_FRONT_TEST_EN_15V5                     (1L<<5)          
  #define RG_RX_FRONT_TEST_EN_15V4                     (1L<<4)          
  #define RG_RX_FRONT_TEST_EN_15V3                     (1L<<3)          
  #define RG_RX_FRONT_TEST_EN_15V2                     (1L<<2)          
  #define RG_RX_FRONT_TEST_EN_15V1                     (1L<<1)          
  #define RG_RX_FRONT_TEST_EN_15V0                     (1L<<0)          

	#define AD_CTRL12_FRONT_TEST		                     (0x0000)
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL13 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_AD_CTRL13                    (BASE_ADC_BA + 0x34)
  #define RG_RX_LPF_TEST_EN_15V                     (1L<<15)           
  #define RG_RX_LPF_TEST_EN_15V14                     (1L<<14)         
  #define RG_RX_LPF_TEST_EN_15V13                     (1L<<13)         
  #define RG_RX_LPF_TEST_EN_15V12                     (1L<<12)         
  #define RG_RX_LPF_TEST_EN_15V11                     (1L<<11)         
  #define RG_RX_LPF_TEST_EN_15V10                     (1L<<10)         
  #define RG_RX_LPF_TEST_EN_15V9                     (1L<<9)           
  #define RG_RX_LPF_TEST_EN_15V8                     (1L<<8)           
  #define RG_RX_LPF_TEST_EN_15V7                     (1L<<7)           
  #define RG_RX_LPF_TEST_EN_15V6                     (1L<<6)           
  #define RG_RX_LPF_TEST_EN_15V5                     (1L<<5)           
  #define RG_RX_LPF_TEST_EN_15V4                     (1L<<4)           
  #define RG_RX_LPF_TEST_EN_15V3                     (1L<<3)           
  #define RG_RX_LPF_TEST_EN_15V2                     (1L<<2)           
  #define RG_RX_LPF_TEST_EN_15V1                     (1L<<1)           
  #define RG_RX_LPF_TEST_EN_15V0                     (1L<<0)           

#ifdef FEATURE_7101_DEVELOPMENT 
  #define AD_CTRL13_LPF_TEST                         (0x0010) 
#else 
  #define AD_CTRL13_LPF_TEST                         (0x0000) 
#endif

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_AD_CTRL14 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
	#define REG32_AD_CTRL14                    (BASE_ADC_BA + 0x38)
  #define RG_RX2TX_MODE_EN_15V                      (1L<<15)           
  #define RG_RX2TX_MODE_EN_15V14                    (1L<<14)         
  #define RG_RX2TX_MODE_EN_15V13                    (1L<<13)         
  #define RG_RX2TX_MODE_EN_15V12                    (1L<<12)         
  #define RG_RX2TX_MODE_EN_15V11                    (1L<<11)         
  #define RG_RX2TX_MODE_EN_15V10                    (1L<<10)         
  #define RG_RX2TX_MODE_EN_15V9                     (1L<<9)           
  #define RG_RX2TX_MODE_EN_15V8                     (1L<<8)           
  #define RG_RX2TX_MODE_EN_15V7                     (1L<<7)           
  #define RG_RX2TX_MODE_EN_15V6                     (1L<<6)           
  #define RG_RX2TX_MODE_EN_15V5                     (1L<<5)           
  #define RG_RX2TX_MODE_EN_15V4                     (1L<<4)           
  #define RG_RX2TX_MODE_EN_15V3                     (1L<<3)           
  #define RG_RX2TX_MODE_EN_15V2                     (1L<<2)           
  #define RG_RX2TX_MODE_EN_15V1                     (1L<<1)           
  #define RG_RX2TX_MODE_EN_15V0                     (1L<<0)           

	#define AD_CTRL14_RX2TX_MODE		                  (0x0000)
	
	///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
	/// REG32_SELF_CTRL
	///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
	#ifdef FEATURE_SELF_SCAN
	#define REG32_SELF_CTRL                (BASE_ADC_BA + 0x50)
	#endif

	
///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_VREF6V 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_VREF6V                    (BASE_ADC_BA + 0x60)
  #define RG_VREF_DRV_EN_15V                     (1L<<5)           
  #define RG_VREF6V_TO_3V_EN_15V                       (1L<<4)     
  #define RG_VREF6V_PWD_15V                            (1L<<3)     

  #define RG_VREF6V_VCM_SEL_15V1                     (1L<<1)       
  #define RG_VREF6V_VCM_SEL_15V0                     (1L<<0)       

	#define VREF6V_CHGPUMP_BIT						(RG_VREF6V_TO_3V_EN_15V|RG_VREF6V_PWD_15V)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_VREF3V 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_VREF3V                    (BASE_ADC_BA + 0x64)
  #define RG_VREF3V_PWD_15V                         (1L<<3)          

  #define RG_VREF3V_VCM_SEL_15V1                    (1L<<1)          
  #define RG_VREF3V_VCM_SEL_15V0                    (1L<<0)          

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TEST_CTRL
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TEST_CTRL                 (BASE_ADC_BA + 0x90)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_DUMMY_DSM 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_DUMMY_DSM                    (BASE_ADC_BA + 0x3FEC)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_DUMMY_IMO 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_DUMMY_IMO                    (BASE_ADC_BA + 0x3FF0)



///===========================================================///
///                                                           ///
///  REGISTER CLASS #?:   DAC CONTROL REGISTERS            ///
///                                                           ///
///===========================================================///



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_DAC_CTRL0 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_DAC_CTRL0                    (BASE_DAC_BA + 0x00)     ///< 
  #define RG_DAC1_GCTRL_15V2                        (1L<<14)      ///< 
  #define RG_DAC1_GCTRL_15V1                        (1L<<13)      ///<      
  #define RG_DAC1_GCTRL_15V0                        (1L<<12)      ///<      

  #define RG_DAC0_GCTRL_15V2                        (1L<<10)      ///< 
  #define RG_DAC0_GCTRL_15V1                        (1L<<9)      ///<      
  #define RG_DAC0_GCTRL_15V0                        (1L<<8)      ///< 

  #define RG_DAC_CKIN_DELSEL_15V1                   (1L<<5)      ///< 
  #define RG_DAC_CKIN_DELSEL_15V0                   (1L<<4)      ///<      
  #define RG_DAC_CKIN_INV_15V                       (1L<<3)      ///<      


  #define RG_DAC1_PWD_15V                           (1L<<1)      ///<      
  #define RG_DAC0_PWD_15V                           (1L<<0)      ///<   


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_DAC_CTRL1 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_DAC_CTRL1                    (BASE_DAC_BA + 0x04)     ///< 
 
  #define RG_IOPSEL_DACOP_15V2                   (1L<<6)      ///< 
  #define RG_IOPSEL_DACOP_15V1                   (1L<<5)      ///< 
  #define RG_IOPSEL_DACOP_15V0                   (1L<<4)      ///<      
     

 	#define RG_IOPSEL_DACBIAS_15V2                 (1L<<2)      ///< 
  #define RG_IOPSEL_DACBIAS_15V1                 (1L<<1)      ///<      
  #define RG_IOPSEL_DACBIAS_15V0                 (1L<<0)      ///<  


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_DAC_CTRL2 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_DAC_CTRL2                    (BASE_DAC_BA + 0x08)     ///< 
  #define RG_DAC_GCTRL_15V2                        (1L<<14)      ///< 
  #define RG_DAC_GCTRL_15V1                        (1L<<13)      ///<      
  #define RG_DAC_GCTRL_15V0                        (1L<<12)      ///<  
  
  #define RG_DAC_OP_CKSEL_15V1                     (1L<<11)      ///< 
  #define RG_DAC_OP_CKSEL_15V0                     (1L<<10)      ///< 
  
  #define RG_DAC_BG_CKSEL_15V1                     (1L<<9)      ///<      
  #define RG_DAC_BG_CKSEL_15V0                     (1L<<8)      ///<
  
  #define RG_VREF_VBGDAC_EN_15V                    (1L<<7)      ///<      
  #define RG_DAC_VBG_SEL_15V                       (1L<<6)      ///< 
  #define RG_DAC_OP_CHOP_EN_15V                    (1L<<5)      ///< 
  #define RG_DAC_I_SEL_15V                   			 (1L<<4)      ///<      
  #define RG_DAC_CHOP_CKINV_15V                    (1L<<3)      ///<      
  #define RG_DAC_CHOP_CK_RN_15V                    (1L<<2)      ///< 
  #define RG_DAC_BG_PWD_15V                        (1L<<1)      ///<      
  #define RG_DAC_BG_CHOP_EN_15V                    (1L<<0)      ///<   


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_CTRL 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TX_CTRL                    (BASE_DAC_BA + 0x10)    
  #define RG_TX_LPF1_GCTRL_15V2                       (1L<<14)      ///< 
  #define RG_TX_LPF1_GCTRL_15V1                       (1L<<13)      ///<      
  #define RG_TX_LPF1_GCTRL_15V0                       (1L<<12)      ///<  
  

  #define RG_TX_LPF0_GCTRL_15V2                        (1L<<10)      ///< 
  #define RG_TX_LPF0_GCTRL_15V1                        (1L<<9)      ///<      
  #define RG_TX_LPF0_GCTRL_15V0                        (1L<<8)      ///<
  
     
  #define RG_TX_VREF_TEST_15V2                        (1L<<6)      ///< 
  #define RG_TX_VREF_TEST_15V1                   			(1L<<5)      ///< 
  #define RG_TX_VREF_TEST_15V0                  			(1L<<4)      ///<   

  
  #define RG_TX_PEN_EN_15V			                       (1L<<3)      ///<      
  #define RG_TX_LPF0_PWD_15V2                           (1L<<2)      ///< 
  #define RG_TX_VCMBUF_PWD_15V                          (1L<<1)      ///<      
  #define RG_TX_LPF0_PWD_15V                          	(1L<<0)      ///<   


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_BUF 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TX_BUF                    			(BASE_DAC_BA + 0x14)     ///<     
  #define RG_TX_VCM6V_EN_15V                       (1L<<2)      ///< 
  #define RG_TXBUF1_PWD_15V                        (1L<<1)      ///<      
  #define RG_TXBUF0_PWD_15V                        (1L<<0)      ///<   



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_OP_CTRL0 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TX_OP_CTRL0                    (BASE_DAC_BA + 0x18)     ///< 
  #define RG_IOPSEL_TXBUF_15V2                      (1L<<2)      ///< 
  #define RG_IOPSEL_TXBUF_15V1                      (1L<<1)      ///<      
  #define RG_IOPSEL_TXBUF_15V0                      (1L<<1)      ///<   



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_OP_CTRL1 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TX_OP_CTRL1                    (BASE_DAC_BA + 0x1C)     ///< 
  #define RG_IOPSEL_TXVREF3V_2_15V2                   (1L<<6)      ///< 
  #define RG_IOPSEL_TXVREF3V_2_15V1                   (1L<<5)      ///<      
  #define RG_IOPSEL_TXVREF3V_2_15V0                   (1L<<4)      ///<  
 

 	#define RG_IOPSEL_TXVREF3V_1_15V2                   (1L<<2)      ///< 
  #define RG_IOPSEL_TXVREF3V_1_15V1                   (1L<<1)      ///<      
  #define RG_IOPSEL_TXVREF3V_1_15V0                   (1L<<1)      ///<   
   

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG32_TX_OP_CTRL2 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG32_TX_OP_CTRL2                    (BASE_DAC_BA + 0x20)     ///< 
 	#define RG_IOPSEL_TXVREF6V_15V2                      (1L<<2)      ///< 
  #define RG_IOPSEL_TXVREF6V_15V1                      (1L<<1)      ///<      
  #define RG_IOPSEL_TXVREF6V_15V0                      (1L<<1)      ///<   
   


///===========================================================///
///                                                           ///
///  REGISTER CLASS #11: LPF OUTPUT REGISTERS     ///
///                                                           ///
///===========================================================///
#define PAD_MODE_SEL_GND 								 (0<<0)
#define PAD_MODE_SEL_VCM 								 (1<<0)
#define PAD_MODE_SEL_DAC2								 (2<<0)
#define PAD_MODE_SEL_DAC2N							 (3<<0)
#define PAD_MODE_SEL_HIZEN							 (4<<0)
#define PAD_MODE_SEL_DAC1								 (5<<0) 
#define PAD_MODE_SEL_DAC1N							 (6<<0)
#define PAD_MODE_SEL_VDD								 (7<<0)
#define PAD_MODE_SEL_DOUT								 (8<<0)
#define PAD_MODE_SEL_DIN								 (9<<0)



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT0 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT0                    (BASE_DGF_BA + 0x50)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT1 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT1                    (BASE_DGF_BA + 0x52)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT2 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT2                    (BASE_DGF_BA + 0x54)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT3 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT3                    (BASE_DGF_BA + 0x56)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT4 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT4                    (BASE_DGF_BA + 0x58)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT5 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT5                    (BASE_DGF_BA + 0x5A)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT6 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT6                    (BASE_DGF_BA + 0x5C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT7 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT7                    (BASE_DGF_BA + 0x5E)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT8 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT8                    (BASE_DGF_BA + 0x60)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT9 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT9                    (BASE_DGF_BA + 0x62)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT10 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT10                    (BASE_DGF_BA + 0x64)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT11 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT11                    (BASE_DGF_BA + 0x66)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT12 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT12                    (BASE_DGF_BA + 0x68)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT13 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT13                    (BASE_DGF_BA + 0x6A)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT14 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT14                    (BASE_DGF_BA + 0x6C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS1_OUT15 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS1_OUT15                    (BASE_DGF_BA + 0x6E)



///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT0 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT0                    (BASE_DGF_BA + 0xD0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT1 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT1                    (BASE_DGF_BA + 0xD2)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT2 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT2                    (BASE_DGF_BA + 0xD4)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT3 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT3                    (BASE_DGF_BA + 0xD6)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT4 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT4                    (BASE_DGF_BA + 0xD8)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT5 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT5                    (BASE_DGF_BA + 0xDA)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT6 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT6                    (BASE_DGF_BA + 0xDC)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT7 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT7                    (BASE_DGF_BA + 0xDE)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT8 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT8                    (BASE_DGF_BA + 0xE0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT9 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT9                    (BASE_DGF_BA + 0xE2)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT10 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT10                    (BASE_DGF_BA + 0xE4)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT11 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT11                    (BASE_DGF_BA + 0xE6)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT12 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT12                    (BASE_DGF_BA + 0xE8)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT13 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT13                    (BASE_DGF_BA + 0xEA)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT14 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT14                    (BASE_DGF_BA + 0xEC)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_COS2_OUT15 
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_COS2_OUT15                    (BASE_DGF_BA + 0xEE)







///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT0 --> LPF SIN TONEA OUTPUT#0
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT0                    (BASE_DGF_BA + 0x150)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT1 --> LPF SIN TONEA OUTPUT#1
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT1                    (BASE_DGF_BA + 0x152)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT2 --> LPF SIN TONEA OUTPUT#2
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT2                    (BASE_DGF_BA + 0x154)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT3 --> LPF SIN TONEA OUTPUT#3
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT3                    (BASE_DGF_BA + 0x156)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT4 --> LPF SIN TONEA OUTPUT#4
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT4                    (BASE_DGF_BA + 0x158)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT5 --> LPF SIN TONEA OUTPUT#5
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT5                    (BASE_DGF_BA + 0x15A)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT6 --> LPF SIN TONEA OUTPUT#6
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT6                    (BASE_DGF_BA + 0x15C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT7 --> LPF SIN TONEA OUTPUT#7
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT7                    (BASE_DGF_BA + 0x15E)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT8 --> LPF SIN TONEA OUTPUT#8
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT8                    (BASE_DGF_BA + 0x160)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT9 --> LPF SIN TONEA OUTPUT#9
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT9                    (BASE_DGF_BA + 0x162)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT10 --> LPF SIN TONEA OUTPUT#10
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT10                    (BASE_DGF_BA + 0x164)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT11 --> LPF SIN TONEA OUTPUT#11
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT11                    (BASE_DGF_BA + 0x166)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT12 --> LPF SIN TONEA OUTPUT#12
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT12                    (BASE_DGF_BA + 0x168)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT13 --> LPF SIN TONEA OUTPUT#13
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT13                    (BASE_DGF_BA + 0x16A)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT14 --> LPF SIN TONEA OUTPUT#14
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT14                    (BASE_DGF_BA + 0x16C)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN1_OUT15 --> LPF SIN TONEA OUTPUT#15
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN1_OUT15                    (BASE_DGF_BA + 0x16E)





///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT0 --> LPF SIN TONEB OUTPUT#0
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT0                    (BASE_DGF_BA + 0x1D0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT1 --> LPF SIN TONEB OUTPUT#1
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT1                    (BASE_DGF_BA + 0x1D2)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT2 --> LPF SIN TONEB OUTPUT#2
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT2                    (BASE_DGF_BA + 0x1D4)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT3 --> LPF SIN TONEB OUTPUT#3
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT3                    (BASE_DGF_BA + 0x1D6)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT4 --> LPF SIN TONEB OUTPUT#4
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT4                    (BASE_DGF_BA + 0x1D8)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT5 --> LPF SIN TONEB OUTPUT#5
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT5                    (BASE_DGF_BA + 0x1DA)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT6 --> LPF SIN TONEB OUTPUT#6
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT6                    (BASE_DGF_BA + 0x1DC)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT7 --> LPF SIN TONEB OUTPUT#7
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT7                    (BASE_DGF_BA + 0x1DE)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT8 --> LPF SIN TONEB OUTPUT#8
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT8                    (BASE_DGF_BA + 0x1E0)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT9 --> LPF SIN TONEB OUTPUT#9
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT9                    (BASE_DGF_BA + 0x1E2)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT10 --> LPF SIN TONEB OUTPUT#10
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT10                    (BASE_DGF_BA + 0x1E4)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT11 --> LPF SIN TONEB OUTPUT#11
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT11                    (BASE_DGF_BA + 0x1E6)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT12 --> LPF SIN TONEB OUTPUT#12
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT12                    (BASE_DGF_BA + 0x1E8)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT13 --> LPF SIN TONEB OUTPUT#13
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT13                    (BASE_DGF_BA + 0x1EA)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT14 --> LPF SIN TONEB OUTPUT#14
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT14                    (BASE_DGF_BA + 0x1EC)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_LPF_SIN2_OUT15 --> LPF SIN TONEB OUTPUT#15
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_LPF_SIN2_OUT15                    (BASE_DGF_BA + 0x1EE)



///===========================================================///
///                                                           ///
///  REGISTER CLASS #12: FLASH OPTION CONTROL REGISTERS       ///
///                                                           ///
///===========================================================///

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT0 -->  : Flash Option#0
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT0_0                (BASE_FLASH_OPT_BA + 0x00)
#define REG_FLASH_OPT0_1                (BASE_FLASH_OPT_BA + 0x01)
#define REG_FLASH_OPT0_2                (BASE_FLASH_OPT_BA + 0x02)
#define REG_FLASH_OPT0_3                (BASE_FLASH_OPT_BA + 0x03)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT1 -->  : Flash Option#1
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT1_0                (BASE_FLASH_OPT_BA + 0x04)
#define REG_FLASH_OPT1_1                (BASE_FLASH_OPT_BA + 0x05)
#define REG_FLASH_OPT1_2                (BASE_FLASH_OPT_BA + 0x06)
#define REG_FLASH_OPT1_3                (BASE_FLASH_OPT_BA + 0x07)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT2 -->  : Flash Option#2
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT2_0                (BASE_FLASH_OPT_BA + 0x08)
#define REG_FLASH_OPT2_1                (BASE_FLASH_OPT_BA + 0x09)
#define REG_FLASH_OPT2_2                (BASE_FLASH_OPT_BA + 0x0A)
#define REG_FLASH_OPT2_3                (BASE_FLASH_OPT_BA + 0x0B)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT3 -->  : Flash Option#3
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT3_0                (BASE_FLASH_OPT_BA + 0x0C)
#define REG_FLASH_OPT3_1                (BASE_FLASH_OPT_BA + 0x0D)
#define REG_FLASH_OPT3_2                (BASE_FLASH_OPT_BA + 0x0E)
#define REG_FLASH_OPT3_3                (BASE_FLASH_OPT_BA + 0x0F)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT4 -->  : Flash Option#4
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT4_0                (BASE_FLASH_OPT_BA + 0x10)
#define REG_FLASH_OPT4_1                (BASE_FLASH_OPT_BA + 0x11)
#define REG_FLASH_OPT4_2                (BASE_FLASH_OPT_BA + 0x12)
#define REG_FLASH_OPT4_3                (BASE_FLASH_OPT_BA + 0x13)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT5 -->  : Flash Option#5
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT5_0                (BASE_FLASH_OPT_BA + 0x14)
#define REG_FLASH_OPT5_1                (BASE_FLASH_OPT_BA + 0x15)
#define REG_FLASH_OPT5_2                (BASE_FLASH_OPT_BA + 0x16)
#define REG_FLASH_OPT5_3                (BASE_FLASH_OPT_BA + 0x17)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT6 -->  : Flash Option#6
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT6_0                (BASE_FLASH_OPT_BA + 0x18)
#define REG_FLASH_OPT6_1                (BASE_FLASH_OPT_BA + 0x19)
#define REG_FLASH_OPT6_2                (BASE_FLASH_OPT_BA + 0x1A)
#define REG_FLASH_OPT6_3                (BASE_FLASH_OPT_BA + 0x1B)

///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
///  REG_FLASH_OPT7 -->  : Flash Option#7
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_FLASH_OPT7_0                (BASE_FLASH_OPT_BA + 0x1C)
#define REG_FLASH_OPT7_1                (BASE_FLASH_OPT_BA + 0x1D)
#define REG_FLASH_OPT7_2                (BASE_FLASH_OPT_BA + 0x1E)
#define REG_FLASH_OPT7_3                (BASE_FLASH_OPT_BA + 0x1F)


///---------------------------------------------------------------------------------------------------------------------------------------------------------------///  
/// REG_EXT_SRAM_CTRL --> [KV] : TX/RX/PN demod SRAM FW access enable/disable ctrl
///---------------------------------------------------------------------------------------------------------------------------------------------------------------/// 
#define REG_EXT_SRAM_CTRL               (BASE_FLASH_OPT_BA + 0x80)
	#define REG_EXT_SRAM_CTRL_0                      (1L<<0)      ///< 
  #define REG_EXT_SRAM_CTRL_1                      (1L<<1)      ///<      
  #define REG_EXT_SRAM_CTRL_2                      (1L<<2)      ///<   
	#define REG_EXT_SRAM_CTRL_3 										 (1L<<3)			///< 
	#define REG_EXT_SRAM_CTRL_4 										 (1L<<4)			///<			
	#define REG_EXT_SRAM_CTRL_5 										 (1L<<5)			///<	 
	#define REG_EXT_SRAM_CTRL_FW_ENABLE 						 (0x1F)			///<	 
	#define REG_EXT_SRAM_CTRL_FW_DISABLE 						 (0x00)			///<	 	


#endif ///< for __ZET6241_H__


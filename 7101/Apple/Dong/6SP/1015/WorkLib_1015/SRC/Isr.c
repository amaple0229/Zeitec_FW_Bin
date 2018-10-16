/**
 * @file Isr.c
 *
 *  interrupt service routine
 *
 *
 * @version $Revision: 70 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/
#include "ZetDef.h"

extern void __cpu_init();
extern void __c_init();
extern void __soc_init();
extern int main();

///------------------------------------------------------------------///
/// OFFSET, ENTRY POINT (9 exception + 7 interrupt)
///------------------------------------------------------------------///
/// 0,      RESET : Cold Reset
/// 1,      TLB FILL
/// 2,      PTE not present
/// 3,      TLB misc
/// 4,      TLB VLPT miss
/// 5,      Machine error
/// 6,      Debug related
/// 7,      General exception
/// 8,      Syscall
///------------------------------------------------------------------///
/// 9,      HW0
/// 10,     HW1
/// 11,     HW2
/// 12,     HW3
///------------------------------------------------------------------///


/**
 * @brief IntInit
 *
 *  Interrupt Init
 *
 * @return NULL
 *
 */
void IntInit(void)
{
  __nds32__mtsr(INT_ALL, NDS32_SR_INT_MASK2);
}

/**
 * @brief IntInit
 *
 *  Interrupt Init
 *
 * @return NULL
 *
 */
void I2CTxRxReset(void)
{
	ZetVar.bI2cStatus &= ~I2C_STA_PACKET_HEAD;
	
	if(READ_REG16(REG32_I2C_CTRL) & I2C_START_FLAG) 
	{
		ZetVar.bI2cStatus |= I2C_STA_PACKET_HEAD; 
		ZetVar.wI2CnSPIRxIdx = 0; ///< svn 122 is error mark , so svn 159 Reopen ///Reopen in svn 116 ; mark in svn Rev 100
		//ZetVar.wI2CnSPITxIdx = 0; ///< mark in svn Rev 100
	}
}

/**
 * @brief IntAdcEnable()
 *
 *  Enable ADC interrupt
 *
 * @return NULL
 *
 */
void IntAdcEnable()
{
  U4 reg  = __nds32__mfsr(NDS32_SR_INT_MASK2);
  reg|=(INT_ADC);
  __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
}


/**
 * @brief IntTimerEnable()
 *
 *  Enable Timer interrupt
 *
 * @return NULL
 *
 */
void IntTimerEnable()
{
  U4 reg  = __nds32__mfsr(NDS32_SR_INT_MASK2);
  reg|=(INT_TM0);
  __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
}

/**
 * @brief IntEnableI2C()
 *
 *  Enable I2C at interrupt
 *
 * @return NULL
 *
 */
 
//U4 IntEnableI2C()
U4 IntEnable(U4 dwAllowInt)
{
  U4 retReg;
  U4 reg  = __nds32__mfsr(NDS32_SR_INT_MASK2);
  retReg = reg;
  
  reg &= dwAllowInt; //(INT_RX|INT_TX);
  reg |= dwAllowInt; //(INT_RX|INT_TX);
  
  __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
  __nds32__setgie_en();
  
  return retReg;

}

/**
 * @brief IntRestore()
 *
 *  Restored stored interrupt mask during interrupt service routine
 *
 * @return NULL
 *
 */
void IntRestore(U4 reg)
{
  __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
}


void IntAdcDisable()
{
   U4 volatile reg  = __nds32__mfsr(NDS32_SR_INT_MASK2);
  reg&=~(INT_ADC);
   __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
}

void IntTimerDisable()
{
   U4 volatile reg  = __nds32__mfsr(NDS32_SR_INT_MASK2);
  reg&=~(INT_TM0);
   __nds32__mtsr(reg, NDS32_SR_INT_MASK2);
}


void nmi_handler(int *reg_ptr)
{
}

#if 0
//inline void InterruptEnable()
{
	__nds32__setgie_en();
	__nds32__dsb();
}

inline void InterruptDisable()
{
	__nds32__setgie_dis();
	__nds32__dsb();
}
#endif

unsigned int intr_save_mask(unsigned int num)
{
	/// get current intr enable mask
	unsigned int old_mask = __nds32__mfsr(NDS32_SR_INT_MASK2);
	unsigned int mask = old_mask & (~(1 << num));
	__nds32__mtsr(mask, NDS32_SR_INT_MASK2);
	__nds32__dsb();
	return old_mask;
}

void intr_restore_mask(unsigned int mask)
{
	InterruptDisable();
	/// restore intr enable mask 
	__nds32__mtsr(mask, NDS32_SR_INT_MASK2);
	__nds32__dsb();
}


void clear_swi()
{
}

/*
 * Interrupt vectors
 * vectors: 32
 * HW0 ~ HW5 and SW0 : IVIC version (IVB.IVIC_VER) = 0
 * HW0 ~ HW31        : IVIC version (IVB.IVIC_VER) = 1
 * */
void NDS32ATTR_RESET("vectors=32;nmi_func=nmi_handler;warm_func=nmi_handler") reset_handler(void)
{
	__cpu_init();
	__c_init();
	__soc_init();
	main();
}

/**
* @brief InterruptI2CRx
*
*  UART#0 Interupt service routine 
*
* @return null
*/
void NDS32ATTR_ISR("not_nested;id=0") HW0_ISR(void)
{
	I2CTxRxReset();
  ///--------------------------------------///
  /// Disable Interrupt
  ///--------------------------------------///  
  InterruptDisable();

  ///--------------------------------------///
  /// Do isr
  ///--------------------------------------///  
  I2CRxIsr();
}

/*
 * HW1 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * I2C-->TX 
 *
 */
void NDS32ATTR_ISR("not_nested;id=1") HW1_ISR(void)  //TX
{
	I2CTxRxReset();

  ///--------------------------------------///
  /// Disable Interrupt
  ///--------------------------------------///  
  InterruptDisable();

  ///--------------------------------------///
  /// Do isr
  ///--------------------------------------///  
  I2CTxIsr();
}

/*
 * HW1 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * ADC 
 * 
 */
void NDS32ATTR_ISR("ready_nested;id=2") HW2_ISR(int vid, NDS32_CONTEXT * ptr)
	{
		U4 volatile reg;
		BYTE bSelect = 0;
		///--------------------------------------------------------///
		///  Restore the interrupt
		///--------------------------------------------------------///
		//reg = IntEnableI2C();
		reg = IntEnable((INT_RX|INT_TX|INT_SPI2));
	
#ifdef FEATURE_TX_RX_IR
		if(sIRScan.bIR_ADmode == 1)
		{
			bSelect = 1;
		}
#endif
#ifdef FEATURE_SELF_SCAN
		if(sSelfScan.bSelfMode== 1)
		{ 	
			bSelect = 2;
		}
#endif	
		///--------------------------------------------------------///
		///  Do ADC Servie routine
		///--------------------------------------------------------///
		if(bSelect == 0)
		{
			AdcIsr();
		}
#ifdef FEATURE_TX_RX_IR
		else if(bSelect == 1)
		{
			U2 ir_value;
		
			SET_BIT(REG_EXT_SRAM_CTRL,1<<4);	
			ir_value =	(U2)SramTablePnDemod[0][0] ;
	
#ifdef FEATURE_IR_BASE
			///////////////////////////////////////////////////////////////////////  check unsigned 16bit value
			if(sIRScan.bIRBaseRound==1)
			{
				sIRScan.wIRBase = ir_value;
				sIRScan.bIRBaseRound = 0;
			}
			else
			{
				short ir_delta_value;
				ir_delta_value =	ir_value - sIRScan.wIRBase;
				if(ir_delta_value> ZetDF.cIRsetupPage.scIRsetup.bIRSensitivity) 												 /////	 0 ~ 65535
				{ 	
					CustomerVar.bIRValid = 1;
					if(CustomerVar.bIRFirstDetect == 1)
					{
						CustomerVar.bIRKeepStatus = 1;
					} 
				}
				else
				{ 
					CustomerVar.bIRValid = 0;
					if(CustomerVar.bIRFirstDetect == 1)
					{
						CustomerVar.bIRKeepStatus = 0;
						CustomerVar.bIRFirstDetect = 0;
					} 
				}
	
				sIRScan.bIRBaseRound = 1;
			}
 #else
			///////////////////////////////////////////////////////////////////////  check unsigned 16bit value
		 if(ir_value> ZetDF.cIRsetupPage.scIRsetup.bIRSensitivity)													///// 	0 ~ 65535
		 {	 
			 CustomerVar.bIRValid = 1;
		 }
		 else
		 {
			 CustomerVar.bIRValid = 0;
		 }
 #endif
	
			CLR_BIT(REG_EXT_SRAM_CTRL,1<<4);	
			CLR_BIT(REG32_PN_DEMOD_IF,1<<0);
			sIRScan.bIR_ADmode = 0;
			sIRScan.bIR_ADmode_INT = 1;
		}
#endif
#ifdef FEATURE_SELF_SCAN
		else if(bSelect == 2)
		{
			/// On PN-SRAM
			SET_BIT(REG_EXT_SRAM_CTRL,1<<4);	
			///////////////////////////////////////////////////// 	
			//////////////////////	Read unsigned 16bit value
			///////////////////////////////////////////////////////
			////	ADC0 value // 49
			sSelfScan.wSelfValue[0] =	(U2)SramTablePnDemod[0][0] + (U2)SramTablePnDemod[0][1] ;
			////	ADC1 value // 49
			//sSelfScan.wSelfValue[1] =	(U2)SramTablePnDemod[0][1] ;	
			////	ADC0 value // 50
			/*sSelfScan.wSelfValue[2] =	(U2)SramTablePnDemod[0][2] ;
			////	ADC1 value // 51
			sSelfScan.wSelfValue[3] =	(U2)SramTablePnDemod[0][3] ;*/		
			/// Off PN-SRAM
			CLR_BIT(REG_EXT_SRAM_CTRL,1<<4);	
	
			CLR_BIT(REG32_PN_DEMOD_IF,1<<0);
			sSelfScan.bSelfMode= 0;
			sSelfScan.bSelfModeINT = 1;
		}
#endif
	
		///--------------------------------------------------------///
		///  Restore the interrupt
		///--------------------------------------------------------///
		__nds32__setgie_dis();	
		IntRestore(reg);	
	}


/*
 * HW3 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * WDT
 * 
 */
void NDS32ATTR_ISR("ready_nested;id=3") HW3_ISR(int vid, NDS32_CONTEXT * ptr)
{
	U4 volatile reg;
  ///--------------------------------------------------------///
  ///  Restore the interrupt
  ///--------------------------------------------------------///
  
  //reg = IntEnableI2C();
  reg = IntEnable((INT_RX|INT_TX|INT_SPI2));  
	
	// CLR_BIT(REG32_WDT_IF, WDT_IF);
	if(ZetVar.bTimerStatus & TIMER_INITIAL_FLAG)
	{
			CLR_BIT(REG32_WDT_IF, WDT_IF); 
			ZetVar.bTimerStatus &= ~TIMER_INITIAL_FLAG;
			CLR_BIT(REG32_TMR_IF, TMR_IF);
			CLR_BIT(REG32_TCON, TMR_EN);
			WRITE_REG(REG32_CKCON, (READ_REG(REG32_CKCON)|(WD2|WD1|WD0)));
	}
	else
	{
		 SYSWatchDogRSTEnable();
		 while(1);
	}

  ///--------------------------------------------------------///
  ///  Restore the interrupt
  ///--------------------------------------------------------///
  __nds32__setgie_dis();  
  IntRestore(reg);  
}

/*
 * HW4 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * TIMER
 * 
 */
void NDS32ATTR_ISR("ready_nested;id=4") HW4_ISR(int vid, NDS32_CONTEXT * ptr)
{
  U4 volatile reg;
  ///--------------------------------------------------------///
  ///  Restore the interrupt
  ///--------------------------------------------------------///
  
  //reg = IntEnableI2C();
  reg = IntEnable((INT_RX|INT_TX|INT_SPI2));
  

  ///--------------------------------------------------------///
  ///  Do Timer Servie routine
  ///--------------------------------------------------------///
  if(ZetVar.bTimerStatus & TIMER_INITIAL_FLAG)
  {	
  	CLR_BIT(REG32_TMR_IF, TMR_IF);
		if(ZetVar2.bTimerTrimCounter != 0xff)
		{
    	ZetVar2.bTimerTrimCounter++;
		}
  }
  else
  {
  	TimerIsr();
  }
  ///--------------------------------------------------------///
  ///  Restore the interrupt
  ///--------------------------------------------------------///
  __nds32__setgie_dis();  
  IntRestore(reg);  
}

/*
 * HW5 interrupt handler
 *
 *
 * 
 */
void NDS32ATTR_ISR("not_nested;id=5") HW5_ISR(void)  
{
}

/*
 * SW0 interrupt handler
 * not_nested for software interrupt
 * */
void NDS32ATTR_ISR("not_nested;id=6") SW0_ISR(int vid)
{
	//puts("A software interrupt occurs ...\n");
	clear_swi();
	return;
}

#ifdef FEATURE_SPI_TRANSFER

void NDS32ATTR_ISR("not_nested;id=7") HW7_ISR(void)
{
	//SPI_flash master mode for burning flash 
	WRITE_REG32(SPI_IF, 0x0);
	//return;
}


void NDS32ATTR_ISR("not_nested;id=8") HW8_ISR(void)
{
	//SPI slave mode 
	SPIRxIsr();		
}

#endif




#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
/* HW9, HW13 and HW19 interrupt will only occur on IVIC version (IVB.IVIC_VER) = 1. */
/*
 * HW9 interrupt handler
 * not_nested for top priority interrupt
 * */
void NDS32ATTR_ISR("ready_nested;id=9") HW9_ISR(int vid, NDS32_CONTEXT * ptr)
{
   U4 volatile reg;
		///--------------------------------------------------------///
		///  Restore the interrupt
		///--------------------------------------------------------///		
	 //reg = IntEnableI2C();
	 reg = IntEnable((INT_RX|INT_TX|INT_SPI2));
		
	 
     CustomerTimer2Isr();
		///--------------------------------------------------------///
		 ///	Restore the interrupt
		 ///--------------------------------------------------------///
		 __nds32__setgie_dis();  
		 IntRestore(reg);  

//	__nds32__mtsr((1 << vid), NDS32_SR_INT_PEND2);
//	SW0_ISR(vid);
}
#endif

/*
 * HW13 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * */
void NDS32ATTR_ISR("ready_nested;id=13") HW13_ISR(int vid, NDS32_CONTEXT * ptr)
{

}


#ifdef FEATURE_ZET7101_GPIO_ISR
void NDS32ATTR_ISR("ready_nested;id=14") HW14_ISR(int vid, NDS32_CONTEXT * ptr)
{
	    U4 volatile reg;
			///--------------------------------------------------------///
			///  Restore the interrupt
			///--------------------------------------------------------///		
		 //reg = IntEnableI2C();
		 reg = IntEnable((INT_RX|INT_TX|INT_SPI2));
					 
		 GpioIsr();
		///--------------------------------------------------------///
		 ///	Restore the interrupt
		 ///--------------------------------------------------------///
		 __nds32__setgie_dis();  
		 IntRestore(reg);
}
#endif
/*
 * HW19 interrupt handler
 * ready_nested for user to enable GIE when they need.
 * */
void NDS32ATTR_ISR("ready_nested;id=19") HW19_ISR(int vid, NDS32_CONTEXT * ptr)
{

}

/*
 * ExceptionHandler
 * 
 * */
void ExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  int i;
  int pc = ptr->ipc;
  U4 reg =  __nds32__mfsr(__NDS32_REG_ITYPE__);

  ///---------------------------------------------------------///
  /// Exception Type
  ///---------------------------------------------------------///
  reg = reg & 0xF;
  for(i = 0 ; i < reg; i++)
  { 
    GIO1_ON();
    GIO1_OFF();
  }
  
  while(1)
  {
    for(i = 0 ; i < vid; i++)
    { 
      GIO2_ON();
      GIO2_OFF();
    }

    for(i = 0 ; i < 32; i++)
    {
      GIO1_ON();
      if(pc &(1L<<(31-i)))
      {
        GIO2_ON();
        GIO2_OFF();        
      }
      else
      {
        GIO2_OFF();        
        GIO2_OFF();        
      }
      GIO1_OFF();    
      if(i%4==3)
      {
        GIO1_OFF();    
        GIO1_OFF();    
        GIO1_OFF();    
        GIO1_OFF();    
      }
    }
    
  }
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=1") TLBFillExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=2") PTENotPresentExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=3") TLBMiscExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=4") TLBVLTPExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=5") MachineErrorExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=6") DebugRelatedExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=7") GeneralExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}

/*
 * General exception handler
 * not_nested for exception interrupt
 * */
void NDS32ATTR_EXCEPT("save_all_regs;ready_nested;id=8") SystemCallExceptionHandler(int vid, NDS32_CONTEXT * ptr)
{
  ExceptionHandler(vid, ptr);
}


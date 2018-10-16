/**
 * @file Customer.c
 *
 *  Customer Function
 *
 *
 * @version $Revision$
 * @author Maple Huang <maple.huang@zeitecsemi.com>
 * @note Copyright (c) 2017, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/

#include "ZetDef.h"

#ifdef FEATURE_FORCE_TOUCH
EXTERN WORD gwForceValue[5];
EXTERN BYTE gbFirstForceCounter[5];
EXTERN BYTE gbForceArea[5];
#endif

#ifdef FEATURE_ZET7101_GPIO_ISR
BYTE gbTestP2Pin5;
BYTE gbTestP2Pin6;
#endif

#ifdef FEATURE_ZET7101_ANA2GPIO
void ANA2GpioInit(void)
{
   ///////////////////////////////////////////
   //bit[0:12] ==> P2_0 ~ P2_12, total 13 Pin, Phyical trace(TRn) mapping in Pin table
   //AD_CTRL14: decide which Pin configure as GPIO from Analog Pin(1),    
   //P2_SEL18: decide which GPIO Pin's VDDIO as 1.8V(1) or 3.3V(0) 
   //IODIR_P2: decide which GPIO Pin configure as input(0) or output(1) Pin
   //IODIR_P2: It need to set the Analog usage PIN as output mode!!!
   //P2: set or get Pin status, High(1), Low(0).
   ///////////////////////////////////////////   
  //GPIO output case
  SET_BIT(AD_CTRL14,1<<0);	// TR7 ==> P2_0 Pin(1)
  SET_BIT(P2_SEL18,1<<0);  // P2_0 as  1.8V(1) 	
	SET_BIT(IODIR_P2,1<<0);/// P2_0 as output(1)
	SET_BIT(P2,1<<0);/// P2_0 High (1)		

	SET_BIT(AD_CTRL14,1<<1);	// TR8 ==> P2_1 Pin(1)
  SET_BIT(P2_SEL18,1<<1);  // P2_1 as  1.8V(1) 
	SET_BIT(IODIR_P2,1<<1);/// P2_1 as output(1)	
	SET_BIT(P2,1<<1);/// P2_1 High (1)

	//IODIR_P2: It need to set the Analog usage PIN as output mode!!!
	WRITE_REG16(IODIR_P2, (READ_REG16(IODIR_P2)|(~(READ_REG16(AD_CTRL14)))));	
	
  //GPIO input case
	SET_BIT(AD_CTRL14,1<<2);	// TR9 ==> P2_2 Pin(1)
	CLR_BIT(P2_SEL18,1<<2);  // P2_2 as  3.3 V(0) 
	CLR_BIT(IODIR_P2,1<<2);/// P2_2 as input(0)		
}

void ANA2GpioDataOut(BYTE bData)
{  	
	int data i;
	int data Org;
	int data temp;

	Org= READ_REG(P2);
	Org &= (~(0x01|0x02));		// simulate 0x01 Clock  0x02 Data
	//Org &= (~0x000C );  
	for(i=8;i>0;i--)
	{
	 if(i==4)
	 {
		 DELAY_1US_NOP();
	 }
	 if(bData & 1<<(i-1))
	 {
		 temp = Org | (0x01|0x02);
	 }
	 else
	 {
		 temp = Org | (0x01);
	 }
	 WRITE_REG(P2, temp);
	 DELAY_10US_NOP();
	 WRITE_REG(P2, Org);			
	}
}

#endif

#ifdef FEATURE_ZET7101_GPIO_ISR
void GpioISRInit(void)
{
   //GPIO input case & interrupt enable
	SET_BIT(AD_CTRL14,1<<5);	// TR12 ==> P2_5 Pin(1)
  CLR_BIT(P2_SEL18,1<<5);  // P2_5 as  3.3 V(0) 
	CLR_BIT(IODIR_P2,1<<5);/// P2_5 as input(0)	
	SET_BIT(P2_INT_EN,1<<5);	// P2_5 interrupt enable(1)
	SET_BIT(P2_WAKE_EN,1<<5);	// P2_5 wake-up enable(1)
	SET_BIT(P2_TRG_EDGE,1<<5); // P2_5 trigger type(1): 1:Pos-edge, 0:Neg-edge 
	CLR_BIT(P2_INT,1<<5);	// P2_5 Default clear int. first(0)


  SET_BIT(AD_CTRL14,1<<6);	// TR13 ==> P2_6 Pin(1)
  SET_BIT(P2_SEL18,1<<6);  // P2_6 as  1.8 V(1) 
	CLR_BIT(IODIR_P2,1<<6);/// P2_6 as input(0)	
	SET_BIT(P2_INT_EN,1<<6);	// P2_6 interrupt enable(1)
	SET_BIT(P2_WAKE_EN,1<<6);	// P2_6  wake-up enable(1)
	CLR_BIT(P2_TRG_EDGE,1<<6); // P2_6  trigger type(0): 1:Pos-edge, 0:Neg-edge  
	CLR_BIT(P2_INT,1<<6); // P2_6 Default clear int. first(0)

//enable INT_P2IO Interrupt MASK, move to INT_ALL setting
//__nds32__mtsr(INT_ALL|INT_P2IO, NDS32_SR_INT_MASK2);
//InterruptEnable();

   //sample code test
   gbTestP2Pin5=0;
	 gbTestP2Pin6=0;

}

void GpioIsr(void)
{
   if((READ_REG(P2_INT)&(1<<5))==(1<<5)) //P2_5 interrupt 
 	 {
      gbTestP2Pin5++;
		  ///--------------------------------------------///
			/// Clear interrupt flag
			///--------------------------------------------///
			CLR_BIT(P2_INT, (1<<5));  //clear P2_5 interrupt 
 	 }
	 else if((READ_REG(P2_INT)&(1<<6))==(1<<6)) //P2_6 interrupt 
 	 {
		   gbTestP2Pin6++;
			 ///--------------------------------------------///
			 /// Clear interrupt flag
			 ///--------------------------------------------///
			 CLR_BIT(P2_INT, (1<<6));  //clear P2_6 interrupt 
 	 }
	 else
 	 {
 	    ///--------------------------------------------///
		  /// Clear interrupt flag
		  ///--------------------------------------------///
	    WRITE_REG16(P2_INT, 0x0000);  //clear all P2_n interrupt if other cse
 	 }
}



#endif


#ifdef FEATURE_ZET7101_CUSTOMER_TIMER2_ISR
void CustomerTimer2Init(WORD wTimerPeriod)
{
  U4 data dwTimerPeriod;

  //GPIO output case
	SET_BIT(AD_CTRL14,1<<3);	// TR10 ==> P2_3 Pin(1)
	SET_BIT(P2_SEL18,1<<3);  // P2_3 as  1.8V(1) 				  
  SET_BIT(P2,1<<3); /// P2_3 High (1)			
	SET_BIT(IODIR_P2,1<<3);/// P2_3 as output(1)		

  ///------------------------------------------------///
  /// Reset Timer
  ///------------------------------------------------///
	CLR_BIT(REG32_TCON2, TMR_EN);   //stop timer
  CLR_BIT(REG32_TMR2_IF, TMR_IF); //clear Interrupt flag
  SET_BIT(REG32_TCON2, TMR_RST);  //reset timer count
  CLR_BIT(REG32_TCON2, TMR_RST);  

  ///------------------------------------------------///
  /// Set Timer Period
  ///------------------------------------------------///
  dwTimerPeriod = (wTimerPeriod * RC512K_TRIM_REFERENCE) / ZetVar2.bTimerTrimCounter;
  dwTimerPeriod = 0 - dwTimerPeriod;
  WRITE_REG(REG32_TCON2, TMR_RELOAD);  //reload enable when timer expire

	CLR_BIT(REG32_TCON2, TCLK_SEL); //switch to fast clock
	while((READ_REG(REG_TMR2_32M_STABLE) & TMR_STABLE) !=TMR_STABLE);
	
	WRITE_REG32(REG32_TMR2, dwTimerPeriod); //set timer period value
	SET_BIT(REG32_TCON2, TCLK_SEL);				//switch back to slow clock
	while((READ_REG(REG_TMR2_40K_STABLE) & TMR_STABLE) !=TMR_STABLE);
	
	SET_BIT(REG32_TCON2, TMR_EN); //start timer	

}

void CustomerTimer2Isr()
{
  if((CustomerVar.bToggleCnt%2)==0)
	{
     SET_BIT(P2,1<<3); /// P2_3 High (1)		
	}
	else
	{
     CLR_BIT(P2,1<<3); /// P2_3 low (0)		
	}
	
	CustomerVar.bToggleCnt=(CustomerVar.bToggleCnt%2)+1;
	CLR_BIT(REG32_TMR2_IF, TMR_IF);
}


#endif



#ifdef FEATURE_CUSTOMER_PROTOCOL

CustomerVarType xdata CustomerVar;

#ifdef FEATURE_6SP_PROTOCOL
void CustomerDataFormat(void)
{
 //Fill coordinate information to Double buffer
	BYTE i;
	BYTE bPacketNumber = 0;
	BYTE bCNT = 0;
	BYTE bFingerNum = 5;
	WORD wValidPoint = 0;
	U4 lPosX;
	U4 lPosY;
	WORD wCustomChecksum = 0;
	PointType ptReportPoint;

	if(ZetVar.bWorkingState!=WORKING_STATE_CUSTOMER_NORMAL)
	{
		return;
	}

	// Workaround for 6SP issue
	if(CustomerVar.bTimeToReport < 10)
	{
		CustomerVar.bTimeToReport++;
		return;
	}
	
	// Check finger status
  for(i=0;i<bFingerNum; i++)
	{
		if((ZetVar2.FingerCoordinateRecord[i].bFingerDown == FINGER_DOWN)
			&&	((!(ZetVar2.FingerCoordinateRecord[i].wFingerStatus & FINGER_STATUS_FORCE_NO_FINGER_CHECK)) || (ZetVar.bLcdOn == 0))) // dont check force no finger when LCD off
		{
			bCNT++;
			wValidPoint |= 0x0001<< i;
		}
	} 			
		
	// Change Timer
	if(wValidPoint > 0)
	{
		ZetVar.wSysMode2 |= SYS_MODE2_RED_MODE;

		ZetVar2.wGreenModeCounter = 0;
		if(ZetVar.wSysMode2 & SYS_MODE2_GREEN_MODE)
		{
			ZetVar.wSysMode2 &= ~SYS_MODE2_GREEN_MODE;
		}
		else if(ZetVar.wSysMode2 & SYS_MODE2_YELLOW_MODE)
		{
			ZetVar.wSysMode2 &= ~SYS_MODE2_YELLOW_MODE;
			ZetVar2.bTimerCounter = ZetDF.cAlgoPage.scAlgoPowerSave.bGreenModeDebounce;
		}

		if(ZetVar2.bTimerCounter > 0)
		{
			ZetVar2.bTimerCounter --;
		}
	}
	else
	{
		ZetVar.wSysMode2 &= ~SYS_MODE2_RED_MODE;
	}

	bPacketNumber=0;		
	if(CustomerVar.wCustomFramePeriod < 65535)
	{
		CustomerVar.wCustomFramePeriod+=16;
	}
	else
	{
		CustomerVar.wCustomFramePeriod = 0;
	}
	// Fill buffer
	if((CustomerVar.bCustomerLastReportFingerCNT>0)||(bCNT>0)||(CustomerVar.bRepeatPoint>0)
	#ifdef FEATURE_TX_RX_IR		
			||(CustomerVar.bIREnable == 1)
	#endif
		)
	{	
		if(bCNT>0)
		{ 
			CustomerVar.bDoubleBuffer[0]=0xe1; 
			//CustomerVar.bDoubleBuffer[1]=0x1A+0x1e*bPacketNumber; 
			//CustomerVar.bDoubleBuffer[2]=0;	
			CustomerVar.bDoubleBuffer[3]=0xeb;					
			CustomerVar.bDoubleBuffer[4]=0x01;//02
			//CustomerVar.bDoubleBuffer[5]=0; 
			CustomerVar.bDoubleBuffer[6]=0xeb;
			CustomerVar.bDoubleBuffer[7]=0x02;//01
			//CustomerVar.bDoubleBuffer[8]=0;					
			CustomerVar.bDoubleBuffer[9]=0xeb;	
			CustomerVar.bDoubleBuffer[10]=0x02;//01
			//CustomerVar.bDoubleBuffer[11]=0;
			//CustomerVar.bDoubleBuffer[12]=0;	
			//CustomerVar.bDoubleBuffer[13]=0;
			//CustomerVar.bDoubleBuffer[14]=LOBYTE(CustomerVar.wCustomChecksum);		
			//CustomerVar.bDoubleBuffer[15]=HIBYTE(CustomerVar.wCustomChecksum);	

			CustomerVar.bDoubleBuffer[16]=0xea; 
			//CustomerVar.bDoubleBuffer[17]=0x01;  					// by host
			//CustomerVar.bDoubleBuffer[18]=CustomerVar.bDoubleBuffer[1];
			//CustomerVar.bDoubleBuffer[19]=0;					
			//CustomerVar.bDoubleBuffer[20]=0xdd;	//dc			// by host	
			CustomerVar.bDoubleBuffer[21]=0x44; 
			//CustomerVar.bDoubleBuffer[22]=CustomerVar.bReportTouchCnt;
			CustomerVar.bDoubleBuffer[23]=0x18;
			CustomerVar.bDoubleBuffer[24]=0x02;					
			CustomerVar.bDoubleBuffer[25]=LOBYTE(CustomerVar.wCustomFramePeriod);	
			CustomerVar.bDoubleBuffer[26]=HIBYTE(CustomerVar.wCustomFramePeriod); 
			CustomerVar.bDoubleBuffer[27]=0;
			//CustomerVar.bDoubleBuffer[28]=0;	
			//CustomerVar.bDoubleBuffer[29]=0;					
			CustomerVar.bDoubleBuffer[30]=0x01;			
			CustomerVar.bDoubleBuffer[31]=0x07; 
			
			CustomerVar.bDoubleBuffer[32]=0x97;
			CustomerVar.bDoubleBuffer[33]=0x08;	
			//CustomerVar.bDoubleBuffer[34]=0;					
			//CustomerVar.bDoubleBuffer[35]=0;		
			//CustomerVar.bDoubleBuffer[36]=0; 
			//CustomerVar.bDoubleBuffer[37]=bPacketNumber;
			CustomerVar.bDoubleBuffer[38]=0x1e;	
			CustomerVar.bDoubleBuffer[39]=0x02;
			
			//CustomerVar.bDoubleBuffer[40]=0;			
			CustomerVar.bDoubleBuffer[41]=0x10; 
			//CustomerVar.bDoubleBuffer[42]=0;			
			CustomerVar.bDoubleBuffer[43]=0;	
			CustomerVar.bDoubleBuffer[44]=0;
		}
#ifdef FEATURE_TX_RX_IR		 
		if((CustomerVar.bIRValid == 1) && (CustomerVar.bLcdoff  == 0) && (CustomerVar.bIRKeepStatus == 0)) // turn Lcd off
		{
			CustomerVar.bDoubleBuffer[0]=0xe1; 
			CustomerVar.bDoubleBuffer[1]=0x1A; 
			//CustomerVar.bDoubleBuffer[2]=0;	
			CustomerVar.bDoubleBuffer[3]=0xe4;					
			CustomerVar.bDoubleBuffer[4]=0xaf;//02
			//CustomerVar.bDoubleBuffer[5]=0; 
			CustomerVar.bDoubleBuffer[6]=0xe1;
			CustomerVar.bDoubleBuffer[7]=0x00;//01
			//CustomerVar.bDoubleBuffer[8]=0;					
			CustomerVar.bDoubleBuffer[9]=0xe1;	
			CustomerVar.bDoubleBuffer[10]=0x00;//01
			//CustomerVar.bDoubleBuffer[11]=0;
			//CustomerVar.bDoubleBuffer[12]=0;	
			//CustomerVar.bDoubleBuffer[13]=0;
			//CustomerVar.bDoubleBuffer[14]=LOBYTE(CustomerVar.wCustomChecksum);		
			//CustomerVar.bDoubleBuffer[15]=HIBYTE(CustomerVar.wCustomChecksum);	

			CustomerVar.bDoubleBuffer[16]=0xea; 
			//CustomerVar.bDoubleBuffer[17]=0x02;  					// by host
			//CustomerVar.bDoubleBuffer[18]=CustomerVar.bDoubleBuffer[1];
			//CustomerVar.bDoubleBuffer[19]=0;					
			//CustomerVar.bDoubleBuffer[20]=0xdd;	//dc			// by host	
			CustomerVar.bDoubleBuffer[21]=0x44; 
			//CustomerVar.bDoubleBuffer[22]=CustomerVar.bReportTouchCnt;
			CustomerVar.bDoubleBuffer[23]=0x18;
			CustomerVar.bDoubleBuffer[24]=0x02;					
			CustomerVar.bDoubleBuffer[25]=LOBYTE(CustomerVar.wCustomFramePeriod);	
			CustomerVar.bDoubleBuffer[26]=HIBYTE(CustomerVar.wCustomFramePeriod); 
			CustomerVar.bDoubleBuffer[27]=5;
			//CustomerVar.bDoubleBuffer[28]=0;	
			//CustomerVar.bDoubleBuffer[29]=0;					
			CustomerVar.bDoubleBuffer[30]=0x02;			
			CustomerVar.bDoubleBuffer[31]=0x07; 
			
			CustomerVar.bDoubleBuffer[32]=0x97;
			CustomerVar.bDoubleBuffer[33]=0x00;	
			//CustomerVar.bDoubleBuffer[34]=0;					
			//CustomerVar.bDoubleBuffer[35]=0;		
			//CustomerVar.bDoubleBuffer[36]=0; 
			//CustomerVar.bDoubleBuffer[37]=bPacketNumber;
			CustomerVar.bDoubleBuffer[38]=0x1e;	
			CustomerVar.bDoubleBuffer[39]=0x07;
			
			CustomerVar.bDoubleBuffer[40]=0x00;			
			CustomerVar.bDoubleBuffer[41]=0x40; 
			//CustomerVar.bDoubleBuffer[42]=0;			
			CustomerVar.bDoubleBuffer[43]=0x73; 
			CustomerVar.bDoubleBuffer[44]=0x03;
			CustomerVar.bINTtriggerCnt=0;
			CustomerVar.bLcdoff = 1;
		}
		else if((CustomerVar.bIRValid == 0) && (CustomerVar.bLcdoff  == 1)) // turn LCD on
		{
			CustomerVar.bDoubleBuffer[0]=0xe1; 
			CustomerVar.bDoubleBuffer[1]=0x1A; 
			//CustomerVar.bDoubleBuffer[2]=0;	
			CustomerVar.bDoubleBuffer[3]=0xeb;					
			CustomerVar.bDoubleBuffer[4]=0x01;//02
			//CustomerVar.bDoubleBuffer[5]=0; 
			CustomerVar.bDoubleBuffer[6]=0xeb;
			CustomerVar.bDoubleBuffer[7]=0x02;//01
			//CustomerVar.bDoubleBuffer[8]=0;					
			CustomerVar.bDoubleBuffer[9]=0xe1;	
			CustomerVar.bDoubleBuffer[10]=0x00;//01
			//CustomerVar.bDoubleBuffer[11]=0;
			//CustomerVar.bDoubleBuffer[12]=0;	
			//CustomerVar.bDoubleBuffer[13]=0;
			//CustomerVar.bDoubleBuffer[14]=LOBYTE(CustomerVar.wCustomChecksum);		
			//CustomerVar.bDoubleBuffer[15]=HIBYTE(CustomerVar.wCustomChecksum);	

			CustomerVar.bDoubleBuffer[16]=0xea; 
			//CustomerVar.bDoubleBuffer[17]=0x02;  					// by host
			//CustomerVar.bDoubleBuffer[18]=CustomerVar.bDoubleBuffer[1];
			//CustomerVar.bDoubleBuffer[19]=0;					
			//CustomerVar.bDoubleBuffer[20]=0xdd;	//dc			// by host	
			CustomerVar.bDoubleBuffer[21]=0x44; 
			//CustomerVar.bDoubleBuffer[22]=CustomerVar.bReportTouchCnt;
			CustomerVar.bDoubleBuffer[23]=0x18;
			CustomerVar.bDoubleBuffer[24]=0x02;					
			CustomerVar.bDoubleBuffer[25]=LOBYTE(CustomerVar.wCustomFramePeriod);	
			CustomerVar.bDoubleBuffer[26]=HIBYTE(CustomerVar.wCustomFramePeriod); 
			CustomerVar.bDoubleBuffer[27]=5;
			//CustomerVar.bDoubleBuffer[28]=0;	
			//CustomerVar.bDoubleBuffer[29]=0;					
			CustomerVar.bDoubleBuffer[30]=0x12;			
			CustomerVar.bDoubleBuffer[31]=0x07; 
			
			CustomerVar.bDoubleBuffer[32]=0x97;
			CustomerVar.bDoubleBuffer[33]=0x00;	
			//CustomerVar.bDoubleBuffer[34]=0;					
			//CustomerVar.bDoubleBuffer[35]=0;		
			//CustomerVar.bDoubleBuffer[36]=0; 
			//CustomerVar.bDoubleBuffer[37]=bPacketNumber;
			CustomerVar.bDoubleBuffer[38]=0x1e;	
			CustomerVar.bDoubleBuffer[39]=0x04;
			
			CustomerVar.bDoubleBuffer[40]=0x00;			
			CustomerVar.bDoubleBuffer[41]=0x00; 
			//CustomerVar.bDoubleBuffer[42]=0;			
			CustomerVar.bDoubleBuffer[43]=0x00; 
			CustomerVar.bDoubleBuffer[44]=0x00;	
			CustomerVar.bINTtriggerCnt=0;
			CustomerVar.bLcdoff = 0;
		}
		else if((CustomerVar.bIRValid == 1) && (CustomerVar.bLcdoff  == 1)) // avoid touch flag when LCD off
		{
			return;
		}
#endif	
		if(CustomerVar.bReportTouchCnt < 255)
		{
			CustomerVar.bReportTouchCnt ++;
		}
		else
		{
			CustomerVar.bReportTouchCnt = 0;
		}

		CustomerVar.bDoubleBuffer[22]=CustomerVar.bReportTouchCnt;

		for(i=0;i<bFingerNum; i++)
		{
			if(((wValidPoint>>i)&0x0001)==0x0001)
			{
				if(ZetDF.cAlgoPage.scSmoothCtrl.bTimeSmoothEnable == TRUE)
				{
					CoorExePointGet(i, COOR_EXE_IDX_LAST, &ptReportPoint);
				}
				else
				{
					ptReportPoint.x = ZetVar2.FingerCoordinateRecord[i].ptCoordinate.x;
					ptReportPoint.y = ZetVar2.FingerCoordinateRecord[i].ptCoordinate.y;
				}
				//also need to check Coord. changed!		 
				if(ZetDF.cFormatCtrl.scDataCtrl.bXYCoorExchange == TRUE)
				{
					lPosX = ptReportPoint.y;
					lPosY = ptReportPoint.x;
				}
				else
				{
					lPosX = ptReportPoint.x;
					lPosY = ptReportPoint.y;
				}

				lPosX = (lPosX<<2) + 0x0000FF00;
				lPosY = (lPosY<<2) ;

				if(lPosY < 70)
				{	
					lPosY = 0xff10;
				}
				else if(lPosY < 140)
				{
					lPosY =0xff20;
				}
				else //if(lPosY < 256)
				{
					lPosY += 0x0000FF00;
				}
			
				CustomerVar.bDoubleBuffer[45+bPacketNumber*30]=0x02+i;	
				if(((CustomerVar.wCustomerLastValidPoint>>i)&0x0001)==0x0000) //first down
				{
					CustomerVar.bDoubleBuffer[46+bPacketNumber*30]=0x03;	
				}
				else // keep down
				{
					CustomerVar.bDoubleBuffer[46+bPacketNumber*30]=0x04;	
				}
				CustomerVar.bDoubleBuffer[47+bPacketNumber*30]=0x02;
				CustomerVar.bDoubleBuffer[48+bPacketNumber*30]=0x01;
				CustomerVar.bDoubleBuffer[49+bPacketNumber*30]=(BYTE)(lPosX & 0x000000FF);//xl					
				CustomerVar.bDoubleBuffer[50+bPacketNumber*30]=(BYTE)((lPosX >> 8) & 0x000000FF);//xh			
				CustomerVar.bDoubleBuffer[51+bPacketNumber*30]=(BYTE)(lPosY & 0x000000FF);//yl 
				CustomerVar.bDoubleBuffer[52+bPacketNumber*30]=(BYTE)((lPosY >> 8) & 0x000000FF);//yh
				//CustomerVar.bDoubleBuffer[53+bPacketNumber*30]=0;	
				//CustomerVar.bDoubleBuffer[54+bPacketNumber*30]=0;					
				//CustomerVar.bDoubleBuffer[55+bPacketNumber*30]=0;	
				
				//CustomerVar.bDoubleBuffer[56+bPacketNumber*30]=0; 
				CustomerVar.bDoubleBuffer[57+bPacketNumber*30]=0xa8;
				CustomerVar.bDoubleBuffer[58+bPacketNumber*30]=0x02;	
				CustomerVar.bDoubleBuffer[59+bPacketNumber*30]=0x5d;					
				CustomerVar.bDoubleBuffer[60+bPacketNumber*30]=0x01;			
				//CustomerVar.bDoubleBuffer[61+bPacketNumber*30]=0; 
				CustomerVar.bDoubleBuffer[62+bPacketNumber*30]=0x40;
				CustomerVar.bDoubleBuffer[63+bPacketNumber*30]=0x86;
				
				//CustomerVar.bDoubleBuffer[64+bPacketNumber*30]=0;					
				CustomerVar.bDoubleBuffer[65+bPacketNumber*30]=0xea;		
				//CustomerVar.bDoubleBuffer[66+bPacketNumber*30]=0; 
				//CustomerVar.bDoubleBuffer[67+bPacketNumber*30]=0;
				//CustomerVar.bDoubleBuffer[68+bPacketNumber*30]=0;	
				//CustomerVar.bDoubleBuffer[69+bPacketNumber*30]=0;					
				//CustomerVar.bDoubleBuffer[70+bPacketNumber*30]=0;

				if(gbFirstForceCounter[i] <= 30)
				{
					gbFirstForceCounter[i]++;
					CustomerVar.bDoubleBuffer[71+bPacketNumber*30] = 0x02;
					CustomerVar.bDoubleBuffer[72+bPacketNumber*30] = 0;
				}
				else
				{
					WORD wCoordDiff = ALCalculateCoordDiff();
					if(wCoordDiff < 100)
					{
						CustomerVar.bDoubleBuffer[71+bPacketNumber*30]= (BYTE)(gwForceValue[i]&0x00FF);			// force L
						CustomerVar.bDoubleBuffer[72+bPacketNumber*30]= (BYTE)((gwForceValue[i]>>8)&0x00FF);	// force H
					}
				}

				if(((CustomerVar.bDoubleBuffer[50+bPacketNumber*30]==0xff) || (CustomerVar.bDoubleBuffer[50+bPacketNumber*30]==0x00))&&(CustomerVar.bDoubleBuffer[72+bPacketNumber*30]>1))
				{					
					CustomerVar.bDoubleBuffer[73+bPacketNumber*30]=0x91;	
				}
				else if((CustomerVar.bDoubleBuffer[50+bPacketNumber*30]==0xff) || (CustomerVar.bDoubleBuffer[50+bPacketNumber*30]==0x00))
				{
					CustomerVar.bDoubleBuffer[73+bPacketNumber*30]=0x51;
				}
				else
				{					
					CustomerVar.bDoubleBuffer[73+bPacketNumber*30]=0x11;	
				}
				CustomerVar.bDoubleBuffer[74+bPacketNumber*30]=0x00;					
				bPacketNumber++;				
			}
			else
			{
				if(((CustomerVar.wCustomerLastValidPoint>>i)&0x0001)==0x0001)
				{    									
					CustomerVar.bDoubleBuffer[46+bPacketNumber*30]=0x07;//first up			    
					bPacketNumber++;
					CustomerVar.bRepeatPoint |= 0x01 << i;
				}
				else
				{
					if(((CustomerVar.bRepeatPoint>>i)&0x01)==0x01)
					{
						CustomerVar.bDoubleBuffer[46+bPacketNumber*30]=0x00; //Repeat up					
						bPacketNumber++;
						CustomerVar.bRepeatPoint &= ~(0x01 << i);				
						gbFirstForceCounter[i] = 0;
						gbForceArea[i] = 5;
					}
				}
			}
		}        

 		if(bPacketNumber!=0)
 		{
 			CustomerVar.bINTtriggerCnt=0; //force to trigger INT Low
 		}
	}    
	CustomerVar.bDoubleBuffer[1]=0x1A+0x1e*bPacketNumber;
	CustomerVar.bDoubleBuffer[18]=CustomerVar.bDoubleBuffer[1];	
	for(i=0;i<14;i++)
	{			
		wCustomChecksum+=CustomerVar.bDoubleBuffer[i];
	}
	CustomerVar.bDoubleBuffer[14]=LOBYTE(wCustomChecksum);		
	CustomerVar.bDoubleBuffer[15]=HIBYTE(wCustomChecksum);	

	CustomerVar.bDoubleBuffer[37]=bPacketNumber;

	if(CustomerVar.bINTtriggerCnt==0)   
	{   			
		if((I2C_INT()==TRUE) && (ZetVar.bWorkingState == WORKING_STATE_CUSTOMER_NORMAL)) 
		{
			CustomerResponseArray((BYTE *)(&CustomerVar.bDoubleBuffer[0]),16,0);
			I2C_INT_LOW(); 
		}			
		CustomerVar.bINTtriggerCnt=0xFF; //Max value means disable    
	} 	
	else if((CustomerVar.bINTtriggerCnt!=0xFF)&&(CustomerVar.bINTtriggerCnt>0))  
	{
		CustomerVar.bINTtriggerCnt--;  
	}
	CustomerVar.wCustomerLastValidPoint = wValidPoint;
	CustomerVar.bCustomerLastReportFingerCNT = bCNT;	
}

void CustomerInit(void)
{  
	MEMSET((void *)&CustomerVar, 0, sizeof(CustomerVar));
	CustomerVar.bINTtriggerCnt=0xFF;
	CustomerVar.wCustomFramePeriod = 0x1248;
}
#else
void CustomerDataFormat(void)
{
 //Fill coordinate information to Double buffer
	BYTE data i;
	BYTE data bPacketNumber = 0;
	BYTE data bCNT = 0;
	BYTE data bFingerNum = ZetDF.cFormatCtrl.scDataCtrl.bFingerNum;
	WORD data wValidPoint = 0;
	WORD data wPosX;
	WORD data wPosY;
	BYTE xdata bLeaveStatus =0;
	BYTE xdata bReportPoint =0;
	PointType xdata ptReportPoint;

	if(ZetVar.bWorkingState!=WORKING_STATE_CUSTOMER_NORMAL)
	{
		return;
	}

	// Check finger status
	for(i=0;i<bFingerNum; i++)
	{
		if((ZetVar2.FingerCoordinateRecord[i].bFingerDown == FINGER_DOWN)
			&&	((!(ZetVar2.FingerCoordinateRecord[i].wFingerStatus & FINGER_STATUS_FORCE_NO_FINGER_CHECK)) || (ZetVar.bLcdOn == 0))) // dont check force no finger when LCD off
		{
			bCNT++;
			wValidPoint |= 0x0001<< i;
		}
	} 			

	// Change Timer
	if(wValidPoint > 0)
	{
		ZetVar.wSysMode2 |= SYS_MODE2_RED_MODE;

		ZetVar2.wGreenModeCounter = 0;
		if(ZetVar.wSysMode2 & SYS_MODE2_GREEN_MODE)
		{
			ZetVar.wSysMode2 &= ~SYS_MODE2_GREEN_MODE;
		}
		else if(ZetVar.wSysMode2 & SYS_MODE2_YELLOW_MODE)
		{
			ZetVar.wSysMode2 &= ~SYS_MODE2_YELLOW_MODE;
			ZetVar2.bTimerCounter = ZetDF.cAlgoPage.scAlgoPowerSave.bGreenModeDebounce;
		}

		if(ZetVar2.bTimerCounter > 0)
		{
			ZetVar2.bTimerCounter --;
		}
	}
	else
	{
		ZetVar.wSysMode2 &= ~SYS_MODE2_RED_MODE;
	}

	CustomerVar.bDoubleBufferStoreIdx=((CustomerVar.bDoubleBufferStoreIdx+1)%2);

	// Fill buffer
	if((CustomerVar.bCustomerLastReportFingerCNT>0)||(bCNT>0))
	{ 
		CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][0]=bCNT;

		for(i=0;i<bFingerNum; i++)
		{
			if(((wValidPoint>>i)&0x0001)==0x0001)
			{
				CoorExePointGet(i, COOR_EXE_IDX_LAST2, &ptReportPoint);
				//also need to check Coord. changed!		 
				if(ZetDF.cFormatCtrl.scDataCtrl.bXYCoorExchange == TRUE)
				{
					wPosX = ptReportPoint.y;
					wPosY = ptReportPoint.x;
				}
				else
				{
					wPosX = ptReportPoint.x;
					wPosY = ptReportPoint.y;
				}

				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][3+bPacketNumber*7] = (BYTE)(((wPosX>>8)&0x000F)<<4); 
				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][4+bPacketNumber*7] = (BYTE)(wPosX&0x00FF);
				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][5+bPacketNumber*7] = (BYTE)(((wPosY>>8)&0x000F)<<4); 
				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][6+bPacketNumber*7] = (BYTE)(wPosY&0x00FF); 				
				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][7+bPacketNumber*7] = 0x10;
				CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][8+bPacketNumber*7] = 0;

				bReportPoint |= 0x0001<< i;

				if(CustomerVar.bCustomReportCnt!=0xFF) 
				{
					CustomerVar.bCustomReportCnt++;
				} 
				else 
				{
					CustomerVar.bCustomReportCnt=0;
				} 
				bPacketNumber++;				
			}
			else if(((wValidPoint>>i)&0x0001)==0x0000)
			{ 	 
				if(((CustomerVar.wCustomerLastValidPoint>>i)&0x0001)==0x0001) // Finger UP
				{
					if(CustomerVar.bCustomerLastReportPoint == 0)
					{
						WORD data wDiff;
						DeltaDistanceGet(i,DISTANCE_IDX_NEW, &wDiff);
						CoorRawPointGet(i, COOR_EXE_IDX_LAST, &ptReportPoint);
						if(((ptReportPoint.x > 50) &&  
							 (ptReportPoint.x < (ZetDF.cGlobalData.scPanelInformation.wSenAxisRes - 50)) &&
							 (ptReportPoint.y > 50) &&	
							 (ptReportPoint.y < (ZetDF.cGlobalData.scPanelInformation.wDriAxisRes - 50))) || (wDiff < 30))
						{
							CoorExePointGet(i, COOR_EXE_IDX_LAST, &ptReportPoint);
						}
						//also need to check Coord. changed!		 
						if(ZetDF.cFormatCtrl.scDataCtrl.bXYCoorExchange == TRUE)
						{
							wPosX = ptReportPoint.y;//ZetVar2.FingerCoordinateRecord[i].ptCoordinate.y;
							wPosY = ptReportPoint.x;//ZetVar2.FingerCoordinateRecord[i].ptCoordinate.x;
						}
						else
						{
							wPosX = ptReportPoint.x;//ZetVar2.FingerCoordinateRecord[i].ptCoordinate.x;
							wPosY = ptReportPoint.y;//ZetVar2.FingerCoordinateRecord[i].ptCoordinate.y;
						}
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][3+bPacketNumber*7] = (BYTE)(((wPosX>>8)&0x000F)<<4); 
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][4+bPacketNumber*7] = (BYTE)(wPosX&0x00FF);
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][5+bPacketNumber*7] = (BYTE)(((wPosY>>8)&0x000F)<<4); 
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][6+bPacketNumber*7] = (BYTE)(wPosY&0x00FF); 				
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][7+bPacketNumber*7] = 0x10;
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][8+bPacketNumber*7] = 0;
						CustomerVar.bCustomerLastReportPoint = 1;
						wValidPoint |= 0x0001<< i;
						bCNT ++;
					}
					else
					{
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][2+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][2+bPacketNumber*7];
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][3+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][3+bPacketNumber*7];
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][4+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][4+bPacketNumber*7];
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][5+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][5+bPacketNumber*7];  
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][6+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][6+bPacketNumber*7]; 		
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][7+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][7+bPacketNumber*7]; 					 
						CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][8+bPacketNumber*7]=CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][8+bPacketNumber*7]; 
						bLeaveStatus = 1; 	
						CustomerVar.bCustomerLastReportPoint = 0;
					}
					bReportPoint |= 0x0001<< i;
				}
				else
				{
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][2+bPacketNumber*7]=0x00;
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][3+bPacketNumber*7]=0x00;
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][4+bPacketNumber*7]=0x00;
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][5+bPacketNumber*7]=0x00; 				
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][6+bPacketNumber*7]=0x00;
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][7+bPacketNumber*7]=0x00; 				
					CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][8+bPacketNumber*7]=0x00; 					
				}

				if(CustomerVar.bCustomReportCnt!=0xFF) 
				{
					CustomerVar.bCustomReportCnt++;
				} 
				else 
				{
					CustomerVar.bCustomReportCnt=0;
				} 

				bPacketNumber++;						
			}
		} 			 
			 
		if(bPacketNumber!=0)
		{
			CustomerVar.bINTtriggerCnt=0; //force to trigger INT Low
			CustomerVar.bCustomerReSendCNT=CUSTOMER_RESNED_NUM; //resend n times
			CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][1]=bReportPoint;

			if(bLeaveStatus==1)
			{
				CustomerVar.bCustomerReSendCNT++;
			}
		}
	} 			 
	else
	{
		CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferStoreIdx][0]=0;	
		if(CustomerVar.bCustomerReSendCNT == 2)
		{
			CustomerVar.bDoubleBufferStoreIdx= ((CustomerVar.bDoubleBufferStoreIdx+1)%2); 
		}
	}
	CustomerVar.wCustomerLastValidPoint=wValidPoint;
	CustomerVar.bCustomerLastReportFingerCNT=bCNT;				
	CustomerVar.bDoubleBufferSendIdx=CustomerVar.bDoubleBufferStoreIdx;

	if(CustomerVar.bCustomerReSendCNT>0) 
	{
		CustomerVar.bCustomerReSendCNT--;
	}
	if(CustomerVar.bCustomerReSendCNT!=0) 	 
	{
		CustomerVar.bINTtriggerCnt=0; //force to trigger INT Low							 
	}

	if(CustomerVar.bINTtriggerCnt==0) 	
	{ 	
		ZetVar.pbI2CnSPITxData = (BYTE *)(&CustomerVar.bDoubleBuffer[CustomerVar.bDoubleBufferSendIdx][0]);
		ZetVar.wI2CnSPITxIdx= 0;
		ZetVar.wI2CnSPITxLen = bPacketNumber*7+2; 
		ZetVar.wI2CnSPIRxIdx= 0;		
		if(I2C_INT()==TRUE) 
		{ 	
			I2C_INT_LOW(); 
		} 		
		CustomerVar.bINTtriggerCnt=0xFF; //Max value means disable		
	} 	
	else if(CustomerVar.bINTtriggerCnt!=0xFF&&CustomerVar.bINTtriggerCnt>0)  
	{
		CustomerVar.bINTtriggerCnt--;  
	}
}

void CustomerInit(void)
{  
	MEMSET((void *)&CustomerVar, 0, sizeof(CustomerVar));
}

#endif

#endif

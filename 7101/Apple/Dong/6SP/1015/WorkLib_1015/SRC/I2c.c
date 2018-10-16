/**
 * @file I2C.c
 *
 * I2C Related Function
 *
 * @version $Revision: 62 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/
#include"ZetDEF.h"
#ifdef FEATURE_CUSTOMER_PROTOCOL
#ifdef SPI_FLASH_WR_ENABLE
BYTE volatile xdata bFlashBuffer[20];
#endif

//reserve switching cmd
BYTE const code bCustomKey4CmdFFF101[]={3,0xFF,0xF1,0x01};
BYTE const code bCustomKey4CmdFFF102[]={3,0xFF,0xF1,0x02};
BYTE const code bCustomKey4CmdFFF103[]={3,0xFF,0xF1,0x03};
BYTE const code bCustomKey4CmdFFFF[]={2,0xFF,0xFF};
BYTE const code bCustomKey4Cmd07[]={1,0x07};
BYTE const code bCustomKey4Cmd08[]={1,0x08};

void CustomHfunc4CmdFFF101();
void CustomHfunc4CmdFFF102();
void CustomHfunc4CmdFFF103();
void CustomHfunc4CmdFFFF();
void CustomHfunc4Cmd07();
void CustomHfunc4Cmd08();

I2CCmdType const code I2CCmd[]=
{
	{bCustomKey4CmdFFF101,  CustomHfunc4CmdFFF101},
	{bCustomKey4CmdFFF102, CustomHfunc4CmdFFF102},
	{bCustomKey4CmdFFF103, CustomHfunc4CmdFFF103},
	{bCustomKey4CmdFFFF,  CustomHfunc4CmdFFFF},
	{bCustomKey4Cmd07,  CustomHfunc4Cmd07},
	{bCustomKey4Cmd08,  CustomHfunc4Cmd08}
};

WORD const code wCustomCmdElementNum=sizeof(I2CCmd)/sizeof(I2CCmd[0]); // 29
#ifdef SPI_FLASH_WR_ENABLE
static BYTE bDebugBuf[16]={0}; 
#else
static BYTE bDebugBuf[8]={0};
#endif

static BYTE bSwitchResponse[8]={0};
#endif
//#define MAX_RX_BUF_LEN    20 //  (10)
//static BYTE bI2CRxBuf[MAX_RX_BUF_LEN]={0}; ///< Current Received TP command
BYTE bI2CRxBuf[MAX_RX_BUF_LEN]; ///< Current Received TP command

static BYTE volatile xdata INTPinBit; 
static BYTE volatile xdata INTPortReg;


void LinkBuffer(BYTE* Buffer)
{
	/// Link the buffer
	ZetVar.pbI2CnSPITxData = Buffer;
	ZetVar.wI2CnSPIRxIdx = 0;
	ZetVar.wI2CnSPITxIdx = 0;
	#ifdef FEATURE_SPI_TRANSFER	
  SPISramReConfigure();	
	#endif
}



void I2CDataFormatDataInCheckSub(void)
{
    ///-------------------------------------------------------------------///
    /// 1. Check if any finger in
    ///-------------------------------------------------------------------///
    if((ZetVar2.bFingerCountsFound >= 1)
     ||((ZetVar.wAlgoStatus2 & (ALG2_HEAVY_PRESS_REPORT_FROZEN|ALG2_HEAVY_PRESS_REPORT_FROZEN_LAST)) == ALG2_HEAVY_PRESS_REPORT_FROZEN_LAST)
      )
    {
	    ZetVar.wSysMode |= SYS_MODE_FINGER_REPORT;
	    ZetVar2.bFingerUpRpCnt = ZetDF.cFormatCtrl.scDataCtrl.bFingerUpReportCnt;
	    ZetVar2.bDataIn = TRUE;
	    return;
    }

    ///-------------------------------------------------------------------///
    /// 2. Check if any key pressed
    ///-------------------------------------------------------------------///
#ifdef FEATURE_REAL_KEY
    if((ZetVar.KeyCtrl.bKeyValidByte != 0x00) || (ZetVar.bKeyLockRelease == FALSE))
    {
      ZetVar.wSysMode |= SYS_MODE_FINGER_REPORT;
      ZetVar2.bFingerUpRpCnt = ZetDF.cFormatCtrl.scDataCtrl.bFingerUpReportCnt;
      ZetVar2.bDataIn = TRUE;
      return;
    }
#endif ///< for FEATURE_REAL_KEY
    ///-------------------------------------------------------------------///
    /// 3. If there is no finger and no key,  return a null data for software
    ///-------------------------------------------------------------------///
    if( ZetVar.wSysMode & SYS_MODE_FINGER_REPORT)
    {
			ZetVar2.bDataIn = TRUE;
    }
    return;
}

/**
 * @brief I2CDataFormatIntLow
 *
 *  Trigger Int low to invoke host to receive data
 *
 * @return NULL
 *
 */
void I2CDataFormatDataInCheck(void)
{
    ZetVar2.bDataIn = FALSE;
#ifdef FEATURE_IIR_FINGER_UP
    if(ZetVar2.bFingerUpEn & FINGER_UP_REPORT_BIT0)
#else
    if(ZetDF.cFinger.scNormal.bFingerUpEnable & FINGER_UP_REPORT_BIT0)
#endif
    {
        //if(ZetVar.bHeavyPressDetectReportFingerFrozenLast == TRUE)
        if(ZetVar.wAlgoStatus2 & ALG2_HEAVY_PRESS_REPORT_FROZEN_LAST)
        {
            I2CDataFormatDataInCheckSub();
            return;
        }
        else
        {
            if(ZetVar.wAlgoStatus & ALGO_FINGER_UP_DEBOUNCE_REPORTING)
            {
                if(	(ZetVar2.bFingerCountsFound ==0)
#ifdef FEATURE_IIR_FINGER_UP
                        && (!(ZetVar2.bFingerUpEn & FINGER_UP_MV_BIT1))
#endif
#ifdef FEATURE_REAL_KEY
                        && (ZetVar.KeyCtrl.bKeyValidByte ==0)
#endif ///< for FEATURE_REAL_KEY
#ifdef FEATURE_FINGER_UP_DEBOUNCE_ALG
                        && (ZetVar2.bFingerReportCounts == 1)
#endif
                  )
                {
                    ZetVar2.bDataIn = NULLSTATE;
                }
                else
                {
                    ZetVar.wSysMode |= SYS_MODE_FINGER_REPORT;
                    ZetVar2.bDataIn = TRUE;
                }
                ZetVar2.bFingerUpRpCnt = ZetDF.cFormatCtrl.scDataCtrl.bFingerUpReportCnt;
            }
            else
            {
                I2CDataFormatDataInCheckSub();
                return;
            }
        }
    }
    else
    {
        I2CDataFormatDataInCheckSub();
        return;
    }

}

/**
 * @brief  I2CWaitTransferDone()
 *
 *  Wait transfer done
 *
 * @return NULL
 *
 */
void  I2CWaitTransferDone(void)
{
	while(I2C_INT()==FALSE)
	{
	    /// Watch dog clear
#ifdef FEATURE_WATCH_DOG
	   SYSWatchDogClear();
#endif ///< for FEATURE_WATCH_DOG
	}
}

/**
 * @brief  I2CInit()
 *
 *  Init the I2C module
 *
 * @return NULL
 *
 */
void I2CInit(void)
{
    int data iI2CCtrl;
    int data iI2CCon2;
		
#ifdef	FEATURE_7101_DEVELOPMENT
			WRITE_REG(I2C_FIFO_CTRL , 0x00);
#endif
		
    ///-------------------------------------------------------------------------------///
    /// 1. I2C_DADDR
    ///-------------------------------------------------------------------------------///
    /// I2C_DADDR  = ZetDF.cMiscRegister.scInterface.bRegI2CDADDR;
    WRITE_REG(REG32_I2C_DADDR, ZetDF.cMiscRegister.scInterface.bRegI2CDADDR);
    /// FWADR 	   = ZetDF.cMiscRegister.scInterface.bRegFWADR;
    WRITE_REG(REG32_FWADR, ZetDF.cMiscRegister.scInterface.bRegFWADR);

    ///-------------------------------------------------------------------------------///
    /// 1. Set I2C Variables
    ///-------------------------------------------------------------------------------///
    ZetVar.wI2CnSPIRxLen = MAX_RX_BUF_LEN;
    ZetVar.wI2CnSPIRxIdx = 0;		
		ZetVar.pbI2CnSPIRxData = (BYTE *)&bI2CRxBuf[0];
		
		ZetVar.wI2CnSPITxLen = B2_BUF_LEN ;
		ZetVar.wI2CnSPITxIdx = 0;
    ZetVar.pbI2CnSPITxData = (BYTE *)&ZetVar.bB2Buf[0];
   
    ZetVar.bI2cStatus = 0;
    ///------------------------------------------------------------///
    /// Calculate the data length
    /// 3 = PACKET_ID  + VALID[1:0]
    ///------------------------------------------------------------///
    ZetVar2.bI2cByteNumber =  (3+(ZetDF.cFormatCtrl.scDataCtrl.bFingerNum<<2));
    /// Check the key byte
#ifdef FEATURE_REAL_KEY
    if(ZetDF.cKeyCtrl.scKeyDriveTrace.bKeySupport == TRUE)
    {
        ZetVar2.bI2cByteNumber++;
    }
#endif
#ifdef FEATURE_VIRTUAL_KEY
    if (ZetVar.bVirtualKeyEnable == TRUE)
    {
        ZetVar2.bI2cByteNumber++;
    }
#endif ///< for FEATURE_VIRTUAL_KEY  	
    ///-------------------------------------------------------------------------------///
    /// 2. Set I2C Hardware
    ///-------------------------------------------------------------------------------///

    /// enable I2C I/O
    iI2CCtrl = READ_REG32(REG32_I2C_CTRL);
    iI2CCtrl &= ~(I2C_DEGLITCH_EN|I2C_DEGLITCH_MAX2|I2C_DEGLITCH_MAX1|I2C_DEGLITCH_MAX0);
    iI2CCtrl |=  (ZetDF.cMiscRegister.scInterface.bI2Ctrl & (I2C_DEGLITCH_EN|I2C_DEGLITCH_MAX2|I2C_DEGLITCH_MAX1|I2C_DEGLITCH_MAX0));
    WRITE_REG32(REG32_I2C_CTRL, iI2CCtrl);
    SET_BIT32(REG32_I2C_CTRL, I2CEN);

    /// Clear I2C Stop Flag  and Set to slave mode (set to 0)
    CLR_BIT32(REG32_I2CON, (MASTER_MODE| I2C_STOP));

    /// LOCK
    iI2CCon2 = READ_REG32(REG32_I2CON2);
    iI2CCon2 &= ~(RG_DRVSEL_SPI_18V1|RG_DRVSEL_SPI_18V0);
    iI2CCon2 |= (ZetDF.cMiscRegister.scInterface.bI2CCon2 & (RG_DRVSEL_SPI_18V1|RG_DRVSEL_SPI_18V0));
    WRITE_REG32(REG32_I2CON2, iI2CCon2);
    CLR_BIT(REG32_I2CON2, (RG_SCL_LOCKB));

    /// Disable mass trasnfer
    CLR_BIT32(REG32_I2C_CTRL, (FLASH_I2C_EN));
    SET_BIT32(REG32_I2C_CTRL, (STD_I2C_EN));

    ///-------------------------------------------------------------------------------///
    /// 3 INT pin init
    ///-------------------------------------------------------------------------------///
    /// Set the INT pin high
		#ifdef FEATURE_PROJECT_ZET7101
		 
		 #ifdef FEATURE_SPI_TRANSFER 
     if((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x01)==0x01) //INT switch to SPI INT(P2_10)
 	   {
			  //SPI INT setting(change INT PIn from P3_5 to P2_10, when SPI enable)
			  SET_BIT(AD_CTRL14+1,1<<2);	// TR17(SPI_INT_PIN) ==> P2_10 Pin(1)
			  SET_BIT(P2_SEL18+1,1<<2);  // P2_10 as  1.8V(1) 				  
				//SET_BIT(P2+1,1<<2); /// P2_10 High (1)	
				INTPortReg	= (INT_CTRL_P2+1);
        INTPinBit 	= INT_PIN_P2; //P2_10 ==> P2+1 , bit2
				I2C_INT_HIGH();  //INT high
				SET_BIT(IODIR_P2+1,1<<2);/// P2_10 as output(1)				
 	   }
		 else
		 #endif	
		 {
				INTPortReg	= INT_CTRL_P3;
				INTPinBit 	= INT_PIN_P5;
			
				SET_BIT32(REG32_IODIRP3, P35_OUT);
		 }
    #endif
		#ifdef FEATURE_PROJECT_ZET7100
    /// Set P35 as output pin for use as  I2C INT pin
    if((ZetDF.cGlobalData.scPanelInformation.bICPackageType & PACKAGE_INT_PIN_BIT4) == ZET7100_PACKAGE_TYPE_A)
    {
        INTPortReg	= INT_CTRL_P3;
        INTPinBit		= INT_PIN_P5;
        I2C_INT_HIGH();
        SET_BIT32(REG32_IODIRP3, P35_OUT);
    }
    else //if((ZetDF.cGlobalData.scPanelInformation.bICPackageType & PACKAGE_INT_PIN_BIT4) == ZET7100_PACKAGE_TYPE_B)
    {
        INTPortReg	= INT_CTRL_P1;
        INTPinBit 	= INT_PIN_P2;
        I2C_INT_HIGH();
        SET_BIT32(REG32_IODIR, P12_OUT);
    }
    #endif
    ///-------------------------------------------------------------------------------///
    /// Set SDA,SCL pull high
    ///-------------------------------------------------------------------------------///
    WRITE_REG32(REG32_PORT3_PH, ZetDF.cMiscRegister.scInterface.bRegPort3PH);

#ifdef FEATURE_PROJECT_ZET7100
#ifdef FEATURE_INT_PULL_HIGH_CONTROL
    ///-------------------------------------------------------------------------------///
    /// INT Pull High Control
    ///-------------------------------------------------------------------------------///
    WRITE_REG32(REG32_PORT1_PH, ZetDF.cMiscRegister.scInterface.bRegPort1PH);
    WRITE_REG32(REG32_PORT_CFG, ZetDF.cMiscRegister.scInterface.bRegPortCfg);
#endif
#endif
}

/**
 * @brief I2CDataFormatReset
 *
 *  Reset the default i2c transfer buffer to the dynamic mode buffer (for mainloop)
 *
 * @return NULL
 *
 */
void I2CDataFormatReset(void)
{
    ///------------------------------------------------------------///
    /// Replace the 0x3C as the reset NULL ID 0x00
    ///------------------------------------------------------------///
    ZetVar.pCurReportData[0]  = I2C_NULL_PACKET_ID;
    ZetVar.wI2CnSPITxLen = ZetVar2.bI2cByteNumber;
		
		//ZetVar.wI2CnSPITxIdx = 0;
    //ZetVar.pbI2CnSPITxData =(BYTE *) ZetVar.pCurReportData;
    LinkBuffer((BYTE *) ZetVar.pCurReportData);
    
    ///------------------------------------------------------------///
    /// Force the INT high
    ///------------------------------------------------------------///
    I2C_INT_HIGH();
}

/**
 * @brief  I2CTxCheckEnd()
 *
 *  Service the I2C Transfer End
 *
 * @return NULL
 *
 */
void I2CTxCheckEnd()
{
    if(ZetVar.wI2CnSPITxIdx >= ZetVar.wI2CnSPITxLen)
    {
				#ifdef FEATURE_CUSTOMER_PROTOCOL
        if(ZetVar.bWorkingState==WORKING_STATE_ZET_CMD_STATE)
        {
            I2CDataFormatReset();    
        }
        else
        {
					ZetVar.pCurReportData[0]  = I2C_NULL_PACKET_ID;
					ZetVar.pbI2CnSPITxData =(BYTE *) ZetVar.pCurReportData;
					ZetVar.wI2CnSPITxLen = ZetVar2.bI2cByteNumber;
					ZetVar.wI2CnSPITxIdx = 0;
					if(I2C_INT() == FALSE)
					{
						I2C_INT_HIGH();
					}
        }
			#else
        I2CDataFormatReset();   
			#endif
    }
}

/**
 * @brief  I2CTxNonAck()
 *
 *  Service the I2C None-ACK
 *
 * @return NULL
 *
 */
void I2CTxNonAck()
{
    ZetVar.wI2CnSPIRxIdx = 0;
    CLR_BIT(REG32_I2CON, STROBE_PEND);
    CLR_BIT(REG32_I2CON, I2C_STOP);
    CLR_BIT(REG32_I2C_TX_IF, I2CTX_IF);
}
/**
 * @brief  I2CTransmit()
 *
 *  Service the I2C TX operation
 *
 * @return NULL
 *
 */
void I2CTransmit(void)
{
    /// mix transmit data
    if((ZetVar.bTranType == TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV) )
    {
        ZetVar.wI2CnSPITxLen = ZetVar2.bI2cByteNumber + (ZetVar2.bDriveMaxAlg + 2) * (ZetVar2.bSenseMaxAlg + 2) * MUTUAL_DATA_TYPE;
#ifdef FEATURE_HTYPE_DEV_RE_MAPPING
        ZetVar.wSysMode2 |= SYS_MODE2_TX_MUTUAL_DEV_MODE;
#endif
        if(ZetVar.wI2CnSPITxIdx < ZetVar2.bI2cByteNumber)
        {
            WRITE_REG(REG32_I2CBUF,  ZetVar.pbI2CnSPITxData[ZetVar.wI2CnSPITxIdx]);
        }
        else
        {
            if(ZetVar.wSysMode2 & SYS_MODE2_RAW_MUTUAL_DEV)
            {
                ZetVar.pbI2CnSPITxData = (BYTE xdata *)&AlgorithmDataPtr->sRawDev[0][0];
            }
            else
            {
                ZetVar.pbI2CnSPITxData = (BYTE xdata *)&AlgorithmDataPtr->sDev[0][0];
            }
            WRITE_REG(REG32_I2CBUF,  ZetVar.pbI2CnSPITxData[ZetVar.wI2CnSPITxIdx- ZetVar2.bI2cByteNumber]);
        }

        if(ZetVar.wI2CnSPITxIdx == 0)
        {
            if(I2C_INT() == TRUE)
            {
                WRITE_REG(REG32_I2CBUF, I2C_NULL_PACKET_ID);
            }
            else
            {
                WRITE_REG(REG32_I2CBUF, I2C_MIX_DYNAMIC_MUTUALDEV_PACKET_ID);
            }
        }

    }
    else
    {
        WRITE_REG(REG32_I2CBUF,  ZetVar.pbI2CnSPITxData[ZetVar.wI2CnSPITxIdx]);
    }

}

/**
 * @brief  I2CTxIsr()
 *
 *  Service the I2C TX(Transceive) operation
 *
 * @return NULL
 *
 */
void I2CTxIsr(void)
{
    WRITE_REG(REG32_I2CON, STROBE_PEND);
    if(!(READ_REG(REG32_I2CON) & I2C_ACK))
    {
        ZetVar.bI2cStatus &= ~(I2C_STA_TX_START);
        I2CTxNonAck();
        return;
    }

    I2CTransmit();

    if(ZetVar.wI2CnSPITxIdx == 0)
    {
        ZetVar.bI2cStatus |= I2C_STA_TX_START;
    }

    /// transfer data index ++
    ZetVar.wI2CnSPITxIdx = ZetVar.wI2CnSPITxIdx + 1;

    /// Set timeout
    ZetVar2.bIntLowTimeout = ZetDF.cFormatCtrl.scDataCtrl.bI2CIntLowTimeOutCnt;

    /// The wrong fetch data from the host
#ifdef FEATURE_CUSTOMER_PROTOCOL
    if(ZetVar.bWorkingState==WORKING_STATE_ZET_CMD_STATE)
    {
        //if((I2C_INT() == TRUE) && (!(ZetVar.bI2cStatus & I2C_STA_SRAM_RW_MODE)))
        if((I2C_INT() == TRUE) && (!(ZetVar.bI2cStatus & I2C_STA_SRAM_RW_MODE))&& ZetVar.bTranType == TRAN_TYPE_DYNAMIC)
        {        
           I2CDataFormatReset();
        }
    }
    else
    {
       //To control INT High/Low in Tx isr according cutsomer protocol requirement
       CustomerTxIsrINTControl();
    }

#else
    if((I2C_INT() == TRUE) && (!(ZetVar.bI2cStatus & I2C_STA_SRAM_RW_MODE)))
    {
        I2CDataFormatReset();
    }
#endif
    ///-----------------------------------------------------///
    /// Stop Cycle:
    /// The last trasnfer data byte sent
    ///-----------------------------------------------------///
    I2CTxCheckEnd();

    /// Reset the Command Index to Zero
    ZetVar.wI2CnSPIRxIdx = 0;

    CLR_BIT(REG32_I2CON, STROBE_PEND);
    CLR_BIT(REG32_I2CON, I2C_STOP);

    CLR_BIT(REG32_I2C_TX_IF, I2CTX_IF);
}

/**
 * @brief  I2CRxIsr()
 *
 *  Service the I2C Rx(Receive) operation
 *
 * @return NULL
 *
 */
void I2CRxIsr(void)
{
  BYTE data bI2cData;
  /// Save the wI2CnSPIRxIdx
  bI2cData = READ_REG(REG32_I2CBUF);	
	ZetVar.bInterfaceType=I2C_TYPE;
  #ifdef FEATURE_SPI_CMD_REG_TYPE
	ZetVar.pbI2CnSPIRxData =(BYTE *)&bI2CRxBuf[0]; //set in I2Cinit, allocate others if I2C conflict with SPI
  #endif

	
  if(ZetVar.wI2CnSPIRxIdx < MAX_RX_BUF_LEN)
  {
      ZetVar.pbI2CnSPIRxData[ZetVar.wI2CnSPIRxIdx] = bI2cData;
  }
  else
  {
      ZetVar.pbI2CnSPIRxData[MAX_RX_BUF_LEN-1] = bI2cData;
  }
  ZetVar.bI2cData = bI2cData;

  ZetVar.bI2cStatus &= ~I2C_STA_TX_START;
  ZetVar.bI2cStatus &= ~I2C_STA_SRAM_RW_MODE; ///< 0: disable SRAM Read Mode

  ZetVar.wI2CnSPIRxIdx++;

  if(ZetVar.bI2cStatus & I2C_STA_PACKET_HEAD)
  {
      ZetVar.pbI2CnSPIRxData[0] = ZetVar.bI2cData; ///bI2CRxBuf[0];
  }

	#ifndef FEATURE_CUSTOMER_PROTOCOL // ndef
  if(ZetDF.cFormatCtrl.scDataCtrl.bMTKFormatEnable  == TRUE)
  {
    if (ZetVar.pbI2CnSPIRxData[0] > MAX_MTK_MODE_SKPI_TPCMD)
    {      
			I2CnSPIDispatchCmd();      
      ZetVar.wI2CnSPITxIdx = 0;
    }
  }
  else
	#endif
  {
  	I2CnSPIDispatchCmd();
  }

  if(ZetVar.wI2CnSPIRxLen > 0)
  {
  	ZetVar.wI2CnSPIRxLen--;
  }

  CLR_BIT(REG32_I2CON, STROBE_PEND);
  CLR_BIT(REG32_I2CON, I2C_STOP);

  ///--------------------------------------------///
  /// Clear interrupt flag
  ///--------------------------------------------///
  WRITE_REG(REG32_I2C_RX_IF, I2C_RX_IF_CLEAR);
}

#ifdef SPI_FLASH_WR_ENABLE 
void SPIEnable(void)
{	
	SET_BIT(SPI_STATUS,SPI_ENABLE); /// SPI enable	
	WRITE_REG(SPI_SHIFT,0x00); /// set bit[1:0]=00  ,data format 1 byte
}

void SetFlashSIData(BYTE bData)
{	
	WRITE_REG(SPI_DATA, bData); ///  FLASH command, Data, or Address	
	SET_BIT(SPI_STATUS,SPI_SHIFT_START);	 /// spi shift start 	
	while( (READ_REG(SPI_STATUS)	) != 0x81); /// wait for ready
	WRITE_REG(SPI_IF,0x00); 	///clear interrupt flag
}

void SetFlashAddress(BYTE bAddrH,BYTE bAddrM,BYTE bAddrL)
{   	 
	 SetFlashSIData(bAddrH);/// Second Data prepare , address 23~16 bit 
	 SetFlashSIData(bAddrM); /// 3rd		Data prepare, address 15~8 bit 	
	 SetFlashSIData(bAddrL); /// 4th		Data prepare, address 7~0 bit
}

/*
void SPI_Test(void)
{
	BYTE xdata FlashID;
	////////////  SPI configuration	
	SPIEnable();  /// SPI enable
	bDebugBuf[10]=0xDD;

	///////////  SPI communication start	
	CLR_BIT(SPI_STATUS,SPI_CSB); /// csb go low, enable slave 
	
	SetFlashSIData(FLASH_RDID); /// command data, FLASH ID read command
	SetFlashSIData(0xaa);	  /// Second Data prepare , dummy data
	FlashID=READ_REG(SPI_DATA);
	bDebugBuf[11]=FlashID;	
	SetFlashSIData(0x55);   /// 3rd    Data prepare, dummy data
	FlashID=READ_REG(SPI_DATA);
	bDebugBuf[12]=FlashID;
	SetFlashSIData(0x00); 	/// 4th    Data prepare, Dummy data
	FlashID=READ_REG(SPI_DATA);
	bDebugBuf[13]=FlashID;
	
	SET_BIT(SPI_STATUS,SPI_CSB);  /// CSB go High disable slave
}
*/

void SPIFlashUIDRead(BYTE AddrH,BYTE AddrM,BYTE AddrL, WORD wSize, BYTE volatile *pFlashBuffer)
{
	WORD i;

	////////////  SPI configuration
	
	SPIEnable();   /// SPI enable

	///////////  SPI communication start
	CLR_BIT(SPI_STATUS,SPI_CSB);	/// csb go low, enable slave 
	
	SetFlashSIData(FLASH_RDUID);	/// command data, FLASH read data command	
	SetFlashAddress(AddrH, AddrM,AddrL);
	for(i=0;i< wSize;i++)
	{
     SetFlashSIData(0xaa); //set dummy data
	   *pFlashBuffer=READ_REG(SPI_DATA);
		 pFlashBuffer++;
	}		

	SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave
}


/**
 * @brief  SPIFlashRead()
 *
 *  Read flash data and store into the pool buffer which pFlashBuffer indicate
 *
 * @return NULL
 *
 */

void SPIFlashRead(BYTE AddrH,BYTE AddrM,BYTE AddrL, WORD wSize, BYTE volatile *pFlashBuffer)
{
	WORD i;

	////////////  SPI configuration
	
	SPIEnable();   /// SPI enable

	///////////  SPI communication start
	CLR_BIT(SPI_STATUS,SPI_CSB);	/// csb go low, enable slave 
	
	SetFlashSIData(FLASH_RD);	/// command data, FLASH read data command	
	SetFlashAddress(AddrH, AddrM,AddrL);
	for(i=0;i< wSize;i++)
	{
     SetFlashSIData(0xaa); //set dummy data
	   *pFlashBuffer=READ_REG(SPI_DATA);
		 pFlashBuffer++;
	}		

	SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave

}
/**
 * @brief  SPIFlashSE()
 *
 *  Erase one sector flash data in fixed address 0x000D00, Never to change address!!!
 *  It need about 200ms to finish erase, it will not allow any write in this duration!!! 
 * 
 *  @return NULL
 */

//Section Erase, Fixed Erase Address 0x000D00 in flash
void SPIFlashSE(void)
{
///////////  SPI communication start
		CLR_BIT(SPI_STATUS,SPI_CSB);  /// csb go low, enable slave 
		SetFlashSIData(FLASH_WREN);   /// command data, FLASH  command
		SET_BIT(SPI_STATUS,SPI_CSB);  /// CSB go High disable slave

		CLR_BIT(SPI_STATUS,SPI_CSB);		/// csb go low, enable slave 
		SetFlashSIData(FLASH_SE);		/// command data, FLASH Sector Erase command
		SetFlashAddress(0x00, 0xD0,0x00);	
		SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave

		CLR_BIT(SPI_STATUS,SPI_CSB);		/// csb go low, enable slave 
		SetFlashSIData(FLASH_WRDI);		/// command data, FLASH  command
		SET_BIT(SPI_STATUS,SPI_CSB);		/// CSB go High disable slave
}

/**
 * @brief  SPIFlashWrite()
 *
 *  Write data into specified flash address from the pool buffer which pFlashBuffer indicate
 *
 * @return NULL
 *
 */

void SPIFlashWrite(BYTE AddrH,BYTE AddrM,BYTE AddrL, WORD wSize, BYTE volatile *pFlashBuffer)
{
	BYTE bBYTE;
	WORD i;

	////////////  SPI configuration
	SPIEnable();	/// SPI enable

	///////////  SPI communication start
	CLR_BIT(SPI_STATUS,SPI_CSB);	/// csb go low, enable slave 
	SetFlashSIData(FLASH_WREN);	/// command data, FLASH  command
	SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave

	CLR_BIT(SPI_STATUS,SPI_CSB);	/// csb go low, enable slave 
	SetFlashSIData(FLASH_PP);	/// command data, FLASH PP command
	SetFlashAddress(AddrH, AddrM, AddrL);
	for(i=0;i< wSize;i++)
	{
		bBYTE=*pFlashBuffer;
		SetFlashSIData(bBYTE);	
		pFlashBuffer++;
	}	
	SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave

	CLR_BIT(SPI_STATUS,SPI_CSB);	/// csb go low, enable slave 
	SetFlashSIData(FLASH_WRDI);	/// command data, FLASH  command
	SET_BIT(SPI_STATUS,SPI_CSB);	/// CSB go High disable slave

}

#endif



/**
 * @brief  I2CnSPIDispatchZetVenCmd()
 *
 *  Dispatch C0 Cmd
 *
 * @return NULL
 *
 */
void I2CnSPIDispatchZetVenCmd()
{
    BYTE  data i;
    BYTE  const code *ptr;
    BYTE  data cnt;
    BYTE  data cntshift;
    BYTE  data bLShift;
    BYTE  data bRShift;
    BYTE  data bStart;

    ZetVar.bVenCmd = ZetVar.pbI2CnSPIRxData[1];

    ///----------------------------------------------------
    /// Check Command Mode
    ///----------------------------------------------------
    if(ZetVar.bVenCmd == VENCMD_ANALOG_SET1)
    {
        ///--------------------------------------------------------------------------------------------///
        /// Change the AD CTRL0 and Non-Drive and Sense Value
        ///
        /// I2CBuf[0] : [C1]
        /// I2CBuf[1] : [0]  									/// Vender Command
        /// I2CBuf[2] : [bAdcCtrl0Setup]			/// ADC_CTRL0 : Detect Range			 : Value = 5 / 2
        /// I2CBuf[3] : [bMutDriveSenseLevel] /// Sense Level 									 : Value = 0 / 1 / 2
        /// I2CBuf[4] : [bDgdCtrl0Setup]			/// DGD_CTRL0 : Decimation Ratio   : Value = 4
        /// I2CBuf[5] : [bDgdCtrl5Setup]			/// DGD_CTRL5 : Digital Filter Freq: Value = 3
        /// I2CBuf[6] : [bAdcCtrl8Setup]			/// ADC_CTRL8 : FrontEnd Clk Div   : Value = 5
        /// I2CBuf[7] : [bOptClkSetup]				/// OPT_CLK 	: AD CLK Div				 : Value = 3
        /// I2CBuf[8] : [0000_0000]						/// [4]: ADC_CTRL11.4              : Value = 0
        /// I2CBuf[9] : [XX]
        ///
        ///--------------------------------------------------------------------------------------------///
        ZetVar.wI2CnSPIRxIdx = 0;
    }
    else if(ZetVar.bVenCmd == VENCMD_GET_TRACE) ///<  C1-01
    {
        bLShift = 0;
        bRShift = 0;
        bStart  = 0;
        cntshift= 0;
        ///--------------------------------------------------
        /// [C1][01][TRACE][XX][XX][XX][XX][XX][XX][XX]
        ///     [TRACE] = 0x00 --> DRIVE
        ///     [TRACE] = 0x01 --> SENSE
        ///--------------------------------------------------
        ///----------------------------------
        ///  C1-01-00 : Mutual Alg Drive Axis
        ///----------------------------------
        if(ZetVar.pbI2CnSPIRxData[2] == GET_TRACE_DRIVE) ///<  C1-01-00
        {
            ptr = &ZetDF.cTraceSetup.scPanelDriveAxis.bDri00;
#ifdef FEATURE_1T2R_ARCHITECTURE
            if(	((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)	== TP_TYPE_1T2R)
								#ifdef FEATURE_NINE_SQUARE_ENABLE
                || ((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)  == TP_TYPE_1T2R_REVERSE)
          			#endif      
              )
            {
                cnt = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax<<1;
                bRShift = 1;
            }
#ifdef FEATURE_1T2R_NO_EDGE
            else if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE) == TP_TYPE_1T2R_NO_EDGE)
            {
                cnt = (ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax<<1) - 2;
                bRShift = 1;
                bStart  = 1;
                cntshift= 1;
            }
#endif
            else
#endif
            {
                cnt = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax;
            }
        }
        ///----------------------------------
        ///  C1-01-01: Mutual Alg Sense Axis
        ///----------------------------------
        else if(ZetVar.pbI2CnSPIRxData[2] == GET_TRACE_SENSE)  ///<  C1-01-01
        {
            ptr = &ZetDF.cTraceSetup.scPanelSenseAxis.bSen00;
#ifdef FEATURE_1T2R_ARCHITECTURE
            if(	((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)	== TP_TYPE_1T2R)
								#ifdef FEATURE_NINE_SQUARE_ENABLE
                ||((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE)  == TP_TYPE_1T2R_REVERSE)
								#endif
							)
            {
                cnt = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax>>1;
                bLShift = 1;
            }
#ifdef FEATURE_1T2R_NO_EDGE
            else if((ZetDF.cFormatCtrl.scDataCtrl.bTPType & TP_TYPE_TYPE) == TP_TYPE_1T2R_NO_EDGE)
            {
                cnt = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax>>1;
                bLShift = 1;
            }
#endif
            else
#endif
            {
                cnt = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax;
            }
        }
        ///----------------------------------
        ///  C1-01-02: Force Alg Drive Axis
        ///----------------------------------
        else if(ZetVar.pbI2CnSPIRxData[2] == GET_FORCE_TRACE_DRIVE) ///<  C1-01-02
        {
            ptr = &ZetDF.cForceTraceSetup.scForceDriveAxis.bForceDri00;
            cnt = ZetDF.cGlobalData.scForceTouchInformation.bFDriveAxisMax;
        }
        ///----------------------------------
        ///  C1-01-03: Force Alg Sense Axis
        ///----------------------------------
        else if(ZetVar.pbI2CnSPIRxData[2] == GET_FORCE_TRACE_SENSE)  ///<  C1-01-03
        {
            ptr = &ZetDF.cForceTraceSetup.scForceSenseAxis.bForceSen00;
            cnt = ZetDF.cGlobalData.scForceTouchInformation.bFSenseAxisMax;
        }
        ///----------------------------------
        ///  C1-01-04 : Mutual HW Drive Axis
        ///----------------------------------
        else if(ZetVar.pbI2CnSPIRxData[2] == GET_TRACE_HW_DRIVE) ///<  C1-01-04
        {
            ptr = &ZetDF.cTraceSetup.scPanelDriveAxis.bDri00;
            cnt = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax;
        }
        ///----------------------------------
        ///  C1-01-05 : Mutual HW Sense Axis
        ///----------------------------------
        else if(ZetVar.pbI2CnSPIRxData[2] == GET_TRACE_HW_SENSE) ///<  C1-01-05
        {
            ptr = &ZetDF.cTraceSetup.scPanelSenseAxis.bSen00;
            cnt = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax;
        }
        else
        {
            ptr = &ZetDF.cTraceSetup.scPanelDriveAxis.bDri00;
            cnt = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax;
        }

        for(i=0; i<(cnt+cntshift); i++)
        {
            pGenBuf1[i] = *(ptr + ((i<<bLShift)>>bRShift));
        }

        ZetVar.wI2CnSPITxLen = cnt;

        //ZetVar.pbI2CnSPITxData = (BYTE *)(&pGenBuf1[bStart]);
        //ZetVar.wI2CnSPITxIdx = 0;
        //ZetVar.wI2CnSPIRxIdx = 0;
        LinkBuffer((BYTE *)(&pGenBuf1[bStart]));
        I2C_INT_LOW();
    }
    else if(ZetVar.bVenCmd == VENCMD_SET_TRAN_TYPE)  ///<  C1-02
    {
        ///---------------------------------------------------------------------///
        /// Set the transfer type
        ///---------------------------------------------------------------------///
        /// TRAN_TYPE_DYNAMIC		                (0x00)
        /// TRAN_TYPE_MUTUAL_SCAN_BASE          (0x01) ///< 16-bit
        /// TRAN_TYPE_MUTUAL_SCAN_DEV           (0x02)
        /// TRAN_TYPE_INIT_SCAN_BASE 		        (0x03)
        /// TRAN_TYPE_INIT_SCAN_DEV		      	  (0x04)
        /// TRAN_TYPE_KEY_MUTUAL_SCAN_BASE		  (0x05)
        /// TRAN_TYPE_KEY_MUTUAL_SCAN_DEV 		  (0x06)
        /// TRAN_TYPE_KEY_DATA  			          (0x07)
        /// TRAN_TYPE_MTK_TYPE  			          (0x0A)
        /// TRAN_TYPE_FOCAL_TYPE  		          (0x0B)
        /// TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV     (0x0D)
        /// TRAN_TYPE_MIX_DYNAMIC_FRH           (0x0E)
        /// TRAN_TYPE_CHECKSUM_READ		          (0x0F)
        /// TRAN_TYPE_ADV_INIT_SCAN_DEV         (0x10)

        /// TRAN_TYPE_FORCE_SCAN_BASE         	(0x11)
        /// TRAN_TYPE_FORCE_SCAN_DEV  	       	(0x12)
        /// TRAN_TYPE_FORCE_C1C2_BASE  	       	(0x13)
        ///---------------------------------------------------------------------///
        ZetVar.wI2CnSPIRxIdx = 0;
        if((ZetVar.pbI2CnSPIRxData[3] == 0x55) &&
           (ZetVar.pbI2CnSPIRxData[4] == 0xAA))
        {
            if (ZetVar.pbI2CnSPIRxData[2] < TRAN_TYPE_UNKNOWN)
            {
                ZetVar.bTranType = ZetVar.pbI2CnSPIRxData[2];
                I2C_INT_HIGH();  ///< [KV] add for Dev Mode err-Transfer in 1st Round
            }
        }
    }
    // disable mark for avoid Zetouch query fail
    //#ifdef FEATURE_FORCE_TOUCH_SCAN
    else if(ZetVar.bVenCmd == VENCMD_TP_DETIAL_LEN_QUERY)  ///<  C1-03
    {
        pGenBuf1[0] = LOBYTE(VENCMD_TP_DETIAL_LEN_QUERY_BUF_LEN);
        pGenBuf1[1] = HIBYTE(VENCMD_TP_DETIAL_LEN_QUERY_BUF_LEN);
        ZetVar.wI2CnSPITxLen = VENCMD_TP_DETIAL_LEN_QUERY_CMD_LEN;

        //ZetVar.pbI2CnSPITxData = (BYTE *)(&pGenBuf1[0]);
        //ZetVar.wI2CnSPITxIdx = 0;
        //ZetVar.wI2CnSPIRxIdx = 0;
        LinkBuffer((BYTE *)(&pGenBuf1[0]));
        I2C_INT_LOW();
    }
    else if(ZetVar.bVenCmd == VENCMD_TP_DETIAL_INFO)  ///<  C1-04
    {
        pGenBuf1[TPINFO_BYTE_0] = ZetDF.cGlobalData.scForceTouchInformation.bFSenseAxisMax; ///< Don't use ZetVar2.bSenseMaxAlg, because the ZetVar is not inited, yet
        pGenBuf1[TPINFO_BYTE_1] = ZetDF.cGlobalData.scForceTouchInformation.bFDriveAxisMax; ///< Don't use ZetVar2.bDriveMaxAlg, because the ZetVar is not inited, yet
        pGenBuf1[TPINFO_BYTE_2] = ZetDF.cGlobalData.scPanelInformation.bSenseAxisMax; ///< Don't use ZetVar2.bSenseMax, because the ZetVar is not inited, yet
        pGenBuf1[TPINFO_BYTE_3] = ZetDF.cGlobalData.scPanelInformation.bDriveAxisMax; ///< Don't use ZetVar2.bDriveMax, because the ZetVar is not inited, yet
        ZetVar.wI2CnSPITxLen = VENCMD_TP_DETIAL_LEN_QUERY_BUF_LEN;
        //ZetVar.pbI2CnSPITxData = (BYTE *)(&pGenBuf1[0]);
        //ZetVar.wI2CnSPITxIdx = 0;
        //ZetVar.wI2CnSPIRxIdx = 0;
				LinkBuffer((BYTE *)(&pGenBuf1[0]));
        I2C_INT_LOW();
    }
    else if(ZetVar.bVenCmd == VENCMD_LOCK_IN_TEST_MODE)  ///<  C1-05
    {
        ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);
        I2C_INT_HIGH();
    }
    //#endif
}

#ifdef FEATURE_CUSTOMER_PROTOCOL
#define UNCHANGE_IDX 0
#define RESET_RXIDX 1
#define RESET_TXIDX 2
#define RESET_TXRXIDX 3

void CustomerResponseArray(BYTE bTmpBuf[], WORD wI2CnSPITxLen, BYTE bResetRxTxIdx)
{
  WORD wTmpIdx;
  ZetVar.wI2CnSPITxIdx= 0;
  ZetVar.wI2CnSPITxLen = wI2CnSPITxLen;

  for(wTmpIdx=0; wTmpIdx < wI2CnSPITxLen; wTmpIdx++)
  {
      CustomerVar.bCustomerBuf[wTmpIdx]= bTmpBuf[wTmpIdx];
  }
  if(wI2CnSPITxLen>0)
  {     	
      ZetVar.pbI2CnSPITxData = (BYTE *)(&CustomerVar.bCustomerBuf[0]);
  }
  #ifdef FEATURE_SPI_TRANSFER 
  SPISramReConfigure(); 
  #endif	
}

void CustomHfunc4CmdFFF101()
{
  BYTE bTmpByte[8];
  BYTE bCnt;
  ZetVar.bWorkingState=ZetVar.pbI2CnSPIRxData[2];

  bDebugBuf[5]=ZetVar.bWorkingState;		 
  for(bCnt=0; bCnt<8; bCnt++)
  {
      bTmpByte[bCnt]=bDebugBuf[bCnt];
  }
  CustomerResponseArray(bTmpByte,8, RESET_RXIDX);
}
void CustomHfunc4CmdFFF102()
{
  CustomHfunc4CmdFFF101();
}
void CustomHfunc4CmdFFF103()
{
	CustomHfunc4CmdFFF101();
}
void CustomHfunc4CmdFFFF()
{
  BYTE bTmpByte[8];
  BYTE bCnt;
  bDebugBuf[5]=ZetVar.bWorkingState;
  for(bCnt=0; bCnt<8; bCnt++)
  {
  	bTmpByte[bCnt]=bDebugBuf[bCnt];
  }
  CustomerResponseArray(bTmpByte,8, RESET_RXIDX);
}

void CustomHfunc4Cmd07() // LCD off
{
	ZetVar.bLcdOn = 0;
	if(ZetVar.wSysMode2 & SYS_MODE2_RED_MODE)
	{
		ZetVar.wSysMode2 &= ~SYS_MODE2_RED_MODE;
	}
	else if(ZetVar.wSysMode2 & SYS_MODE2_YELLOW_MODE)
	{
		ZetVar.wSysMode2 &= ~SYS_MODE2_YELLOW_MODE;
		ZetVar2.bTimerCounter = ZetDF.cAlgoPage.scAlgoPowerSave.bGreenModeDebounce;
	}
	ZetVar2.bTimerCounter=0;
	ZetVar.wSysMode2 |= SYS_MODE2_GREEN_MODE;
	ZetVar.bTimerStatus |= TIMER_RELOAD_FLAG;
}

void CustomHfunc4Cmd08() // LCD on
{
	ZetVar.bLcdOn = 1;
	if(ZetVar.wSysMode2 & SYS_MODE2_GREEN_MODE)
	{
		ZetVar.wSysMode2 &= ~SYS_MODE2_GREEN_MODE;
	}
	ZetVar2.bTimerCounter=0;
	ZetVar.wSysMode2 |= SYS_MODE2_RED_MODE;	
	ZetVar.bTimerStatus |= TIMER_RELOAD_FLAG;
}

/**###################################
INT Control in TxIsr
##################################**/

void CustomerTxIsrINTControl()
{
	if(ZetVar.pbI2CnSPIRxData[0] == 0x05)		// 0502 0501 
	{
		if(I2C_INT() == FALSE)
		{ 					 
			I2C_INT_HIGH();
		}
	}
}

/**###################################
main parsing loop
##################################**/

void I2CDispatchCustomerCmd()
{	
	WORD j;
	BYTE k;
	void (*pTmpCmdHF)();
	BYTE bLocalNonHitKeyFlag;
	I2CCmdType *pI2CCmd;
	BYTE btmpCmdKey;
 
	pI2CCmd=(I2CCmdType *)I2CCmd;

	 for(j=0;j<wCustomCmdElementNum;j++)
   {      
       if(I2CCmd[j].Cmdkey[0]==(BYTE)(ZetVar.wI2CnSPIRxIdx)) //size matched
			 {			
			   bLocalNonHitKeyFlag=0;
			   for(k=0;k<(BYTE)ZetVar.wI2CnSPIRxIdx;k++)
			   {	
	           btmpCmdKey=I2CCmd[j].Cmdkey[k+1];						 
		         if(k==0)
		         {
	              if(btmpCmdKey!=ZetVar.pbI2CnSPIRxData[0])
	             	{
	                 bLocalNonHitKeyFlag=1;
									 break;
	             	}
		         }
						 else 
						 { 
								if(btmpCmdKey!=ZetVar.pbI2CnSPIRxData[k])
								 {										       
                   bLocalNonHitKeyFlag=1;
									 break;
								 }
						 	}
			   }

				 //Hit command
				 if(bLocalNonHitKeyFlag==0)
				 {  
							pTmpCmdHF=I2CCmd[j].CmdHF;
							(pTmpCmdHF)();
							// ZetVar.wI2CnSPIRxIdx=0;   // consider 01 02 & 01 02 07, if Rxidx set as 0, 01 02 07 will never hit 
							break;
				 }				 
			 }
       pI2CCmd++;
   }
}
#endif

void I2CnSPIDispatchZetCmd()
{
  if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A0_INIT_SCAN_AD)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_A0_CMD_LEN;
            ZetVar.wI2CnSPITxLen = 2*(ZetVar2.bSenseMax + ZetVar2.bDriveMax);

            /// Test Mode Enabled
            ZetVar.wTestMode = (TP_TEST_FPC_SHORT_EN);
            ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);
            ZetVar.wSysMode2 |= SYS_MODE2_ZET7100_FORCE_EVENODD_SCAN;
            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();
						/// Link the buffer
						LinkBuffer((BYTE *)(&pGenBuf[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A1_INIT_SCAN_DV)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_A1_CMD_LEN;
            ZetVar.wI2CnSPITxLen = ZetVar2.bSenseMax + ZetVar2.bDriveMax;

            /// Test Mode Enabled
            ZetVar.wTestMode = (TP_TEST_FPC_OPEN_EN);
            ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);
            ZetVar.wSysMode2 |= SYS_MODE2_ZET7100_FORCE_EVENODD_SCAN;
            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();
						/// Link the buffer
						LinkBuffer((BYTE *)(&pGenBuf[0]));
        }
#ifdef FEATURE_I2C_CMD_A2_RELEASE_TEST
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A2_RELEASE_TEST)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_A2_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_A2_TX_DATA_LEN;

            /// Test Mode Disabled
            ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;

						/// Link the buffer
						LinkBuffer((BYTE *)(&pGenBuf[0]));
            I2C_INT_HIGH();
        }
#endif
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A3_INIT_SCAN_DV)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_A3_CMD_LEN;
            ZetVar.wI2CnSPITxLen = (ZetVar2.bSenseMaxAlg * ZetVar2.bDriveMaxAlg)<<1;

            /// Test Mode Enabled
            if(ZetVar.wTestMode == TP_TEST_ITO_SENSOR_SWITCH)
            {
                ZetVar.wTestMode = (TP_TEST_ITO_SENSOR | TP_TEST_ITO_SENSOR_SWITCH);
            }
            else
            {
                ZetVar.wTestMode = (TP_TEST_ITO_SENSOR);
            }

            /*tVar.bScanMethod = METHOD_SCAN_FINISH;
            IntAdcDisable();*/

            ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);
            //ZetVar.wSysMode2 &= ~SYS_MODE2_ROUND_RUNNING;
            //CLR_BIT(REG32_TCON, TMR_EN);

            SYSWatchDogDisable();
            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&AlgorithmDataPtr->sDevAlloc[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A4_MUTUAL_DEV)
        {
            I2C_INT_HIGH();
            ZetVar.wI2CnSPIRxLen = TP_CMD_A4_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_A4_TX_DATA_LEN;
            ZetVar.bTranType = TRAN_TYPE_MUTUAL_SCAN_DEV;

            /// Test Mode Disabled
            ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;

						/// Link the buffer
						LinkBuffer((BYTE *)(&AlgorithmDataPtr->sDevAlloc[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_A5_MUTUTAL_PIN_SWITCH)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_A5_CMD_LEN;
            ZetVar.wI2CnSPITxLen = (ZetVar2.bSenseMaxAlg * ZetVar2.bDriveMaxAlg)<<1;

            /// Test Mode Enabled
            ZetVar.wTestMode = (TP_TEST_ITO_SENSOR_SWITCH);
            ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);

            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&AlgorithmDataPtr->sDevAlloc[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_AA_INIT_ENHANCE_SCAN_DV)
        {
            #ifdef FEATURE_SPI_TRANSFER
						/// The last command bytes
						if(ZetVar.bInterfaceType == SPI_TYPE)
						{
								ZetVar.wI2CnSPIRxLen = TP_CMD_AA_CMD_LEN;
								ZetVar.wI2CnSPIRxIdx = 0;
						}
						else
            #endif
						{
								if(ZetVar.wI2CnSPIRxIdx  == 1)
								{
									ZetVar.wI2CnSPIRxLen = TP_CMD_AA_CMD_LEN;
								}
								else if(ZetVar.wI2CnSPIRxIdx == TP_CMD_AA_CMD_LEN)
								{
									ZetVar.wI2CnSPIRxIdx = 0;
								}
						}		
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_AB_KEY_SCAN_AD)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_AB_CMD_LEN;
            ZetVar.wI2CnSPITxLen = 2*KEY_MAX_NUM; ///< For SW Easy Array Re-use;

            /// Test Mode Enabled
            ZetVar.wTestMode = (TP_TEST_KEY_AD_EN);
            ZetVar.wSysMode |= (SYS_MODE_TP_TEST_EN);

            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&pGenBuf[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_AC_RAW_DEV)
        {
            ZetVar2.bDynamicHopDataOutSel = (ZetVar2.bDynamicHopDataOutSel+1)%3;
            ///------------------------------------------///
            ///    To display mutual raw dev at mutual dev mode
            ///------------------------------------------///
            if(ZetVar.wSysMode2 & SYS_MODE2_RAW_MUTUAL_DEV)
            {
                ZetVar.wSysMode2 &= ~SYS_MODE2_RAW_MUTUAL_DEV;
            }
            else
            {
                ZetVar.wSysMode2 |= SYS_MODE2_RAW_MUTUAL_DEV;
            }
            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
        }
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B0_RESET)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_B0_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B0_TX_DATA_LEN;

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));

            /// Relese the interrupt
            CLR_BIT(REG32_I2CON, STROBE_PEND);
            CLR_BIT(REG32_I2CON, I2C_STOP);
            CLR_BIT(REG32_I2C_RX_IF, I2CRX_IF);

            /// Do the softreset
            SYSSoftResetInt();
        }
#ifdef FEATURE_DEEP_SLEEP				
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B1_DEEP_SLEEP)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_B1_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B1_TX_DATA_LEN;
                
						/// Test Mode Enabled
            ZetVar.wSysMode |= (SYS_MODE_DEEP_SLEEP);
            /// Test Mode Disabled
            ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;

            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
        }
#endif				
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B2_PANEL_INFO)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_B2_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B2_TX_DATA_LEN;

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));

            I2C_INT_LOW();
        }
#ifdef FEATURE_IDLE				
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B3_IDLE)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_B3_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B3_TX_DATA_LEN;
            /// Test Mode Enabled
            ZetVar.wSysMode |= (SYS_MODE_IDLE);
            /// Test Mode Disabled
            ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;

            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
        }
#endif				
#ifdef FEATURE_WAKE_UP				
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B4_WAKE)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_B4_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B4_TX_DATA_LEN;

            /// Wake up the system
            ZetVar.wSysMode &= ~(SYS_MODE_IDLE | SYS_MODE_DEEP_SLEEP);

            /// Test Mode Disabled
            ZetVar.wSysMode &= ~SYS_MODE_TP_TEST_EN;
            ZetVar2.wGreenModeCounter = 0;
            ZetVar.wSysMode2 &= ~SYS_MODE2_GREEN_MODE;
						ZetVar.wSysMode2 &= ~SYS_MODE2_YELLOW_MODE;
            ZetVar.bTimerStatus |= TIMER_RELOAD_FLAG;

            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();

						/// Link the buffer
						LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
        }
#endif				
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B5_CHARGER_MODE_ENABLE)
        {
					SYSChargerModeEnable();

					/// Cancel the previous interrupt, if any
					I2C_INT_HIGH();

					ZetVar.wI2CnSPIRxLen = TP_CMD_B5_CMD_LEN;
					ZetVar.wI2CnSPITxLen = TP_CMD_B5_TX_DATA_LEN;
					/// Link the buffer
					LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
				}
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B5_CHARGER_MODE_DISABLE)
        {
	        SYSChargerModeDisable();
	        /// Cancel the previous interrupt, if any
	        I2C_INT_HIGH();
	        ZetVar.wI2CnSPIRxLen = TP_CMD_B5_CMD_LEN;
	        ZetVar.wI2CnSPITxLen = TP_CMD_B5_TX_DATA_LEN;
					/// Link the buffer
					LinkBuffer((BYTE *)(&ZetVar.bB2Buf[0]));
        }
#ifdef FEATURE_CHECK_SUM
        else if(ZetVar.pbI2CnSPIRxData[0] == TPCMD_B8_CHECKSUM_READ)
        {
            /// Cancel the previous interrupt, if any
            I2C_INT_HIGH();
            ZetVar.wI2CnSPIRxLen = TP_CMD_B8_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_B8_TX_DATA_LEN;
            ZetVar.wI2CnSPIRxIdx = 0;
						ZetVar.wI2CnSPITxIdx = 0;
            CLR_BIT(REG32_I2CON, STROBE_PEND);
            CLR_BIT(REG32_I2CON, I2C_STOP);
            SYSWatchDogDisable();
            ALCheckSumProcess();
            SYSWatchDogEnable();
            DFCheckSumData();
        }
#endif ///< for FEATURE_CHECK_SUM
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_C0_ADC_CTRL0_SET)
        {
            #ifdef FEATURE_SPI_TRANSFER
						/// The last command bytes
						if(ZetVar.bInterfaceType == SPI_TYPE)
						{
								ZetVar.wI2CnSPIRxLen = TP_CMD_C0_CMD_LEN;
								ZetVar.wI2CnSPIRxIdx = 0;
						}
						else
            #endif
						{
		            /// The last command bytes
		            if(ZetVar.wI2CnSPIRxIdx  == 1)
		            {
		                ZetVar.wI2CnSPIRxLen = TP_CMD_C0_CMD_LEN;
		            }
		            else if(ZetVar.wI2CnSPIRxIdx == TP_CMD_C0_CMD_LEN)
		            {
		                ZetVar.wI2CnSPIRxIdx = 0;
		            }
            }
        }
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_C1_VEN_CMD_SET)
        {
          #ifdef FEATURE_SPI_TRANSFER
						/// The last command bytes
						if(ZetVar.bInterfaceType == SPI_TYPE)
						{
								ZetVar.wI2CnSPIRxLen = TP_CMD_C1_CMD_LEN;
								I2CnSPIDispatchZetVenCmd();
						}
						else
            #endif
						{
		            /// The last command bytes
		            if(ZetVar.wI2CnSPIRxIdx  == 1)
		            {
		                ZetVar.wI2CnSPIRxLen = TP_CMD_C1_CMD_LEN;
		            }
		            else if(ZetVar.wI2CnSPIRxIdx == TP_CMD_C1_CMD_LEN)
		            {
		                I2CnSPIDispatchZetVenCmd();
		            }
            }
        }
		#ifdef FEATURE_SPI_TRANSFER
		   else if (ZetVar.pbI2CnSPIRxData[0] == 0xD5)
       {
          if(ZetVar.bInterfaceType==SPI_TYPE)
					{
              if(ZetVar.bTranType != TRAN_TYPE_MIX_DYNAMIC_MUTUALDEV)
              {					
				         SPISramReConfigure();
							   I2C_INT_HIGH();
              }
							else
							{
                if(ZetVar.pbI2CnSPITxData==ZetVar.pCurReportData)
              	{
                   //redirect the pointer to Dev structure, after sent Coordinate data.
                  ZetVar.wI2CnSPITxLen = (ZetVar2.bDriveMaxAlg + 2) * (ZetVar2.bSenseMaxAlg + 2) * MUTUAL_DATA_TYPE;
                  if(ZetVar.wSysMode2 & SYS_MODE2_RAW_MUTUAL_DEV)
			            {
			                ZetVar.pbI2CnSPITxData = (BYTE xdata *)&AlgorithmDataPtr->sRawDev[0][0];
			            }
			            else
			            {
			                ZetVar.pbI2CnSPITxData = (BYTE xdata *)&AlgorithmDataPtr->sDev[0][0];
			            }									
                  SPISramReConfigure();
              	}
								else
								{
									SPISramReConfigure();
									I2C_INT_HIGH();
								}								
							}
					}
			 }
		#endif
#ifdef FEATURE_SRAM_READ_WRITE
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_F3_SRAM32_READ)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_F3_CMD_LEN;
            #ifdef FEATURE_SPI_TRANSFER
						if(ZetVar.wI2CnSPIRxIdx == (TP_CMD_F3_CMD_LEN)||ZetVar.bInterfaceType == SPI_TYPE)
						#else
            /// The last command bytes
            if(ZetVar.wI2CnSPIRxIdx == (TP_CMD_F3_CMD_LEN))
						#endif
            {
                MEMCPY((void*)&ZetVar.dwSramAddr, (void*)&ZetVar.pbI2CnSPIRxData[1], 4);
               // ZetVar.wI2CnSPITxLen = *(WORD *)&ZetVar.pbI2CnSPIRxData[TP_CMD_F3_CMD_LEN-2];
                ZetVar.wI2CnSPITxLen = *(WORD *)(&ZetVar.pbI2CnSPIRxData[TP_CMD_F3_CMD_LEN-2]);
							 /// Link the buffer
								LinkBuffer((BYTE xdata *)(uintptr_t)(ZetVar.dwSramAddr));
                I2C_INT_LOW();
            }
        }
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_F4_SRAM32_WRITE)
        {            
            #ifdef FEATURE_SPI_TRANSFER
						/// The last command bytes
						if(ZetVar.bInterfaceType == SPI_TYPE)
						{
								ZetVar.wI2CnSPIRxLen = TP_CMD_F4_CMD_LEN;
								MEMCPY((void*)&ZetVar.dwSramAddr, (void*)&ZetVar.pbI2CnSPIRxData[1], 4);
                /// Write Data to sram
                WRITE_XDATA((uintptr_t)ZetVar.dwSramAddr, ZetVar.pbI2CnSPIRxData[(TP_CMD_F4_CMD_LEN-1)]);
                ZetVar.dwSramAddr++;
								ZetVar.wI2CnSPIRxIdx = 0;
						}
						else
            #endif
						{							
		            /// The last command bytes
		            if(ZetVar.wI2CnSPIRxIdx  == 1)
		            {
		                ZetVar.wI2CnSPIRxLen = TP_CMD_F4_CMD_LEN;
		            }
		            if(ZetVar.wI2CnSPIRxIdx == TP_CMD_F4_CMD_LEN)
		            {
		                MEMCPY((void*)&ZetVar.dwSramAddr, (void*)&ZetVar.pbI2CnSPIRxData[1], 4);
		                /// Write Data to sram
		                WRITE_XDATA((uintptr_t)ZetVar.dwSramAddr, ZetVar.pbI2CnSPIRxData[(TP_CMD_F4_CMD_LEN-1)]);
		                ZetVar.dwSramAddr++;
		                ZetVar.wI2CnSPIRxIdx = 0;
		            }
            }
        }
#endif
#ifdef FEATURE_I2C_CMD_F7_ENABLE
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_F7_CALIBRATE)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_F7_CMD_LEN;
            ZetVar.wI2CnSPITxLen = TP_CMD_F7_TX_DATA_LEN;
            ZetVar.wSysMode &=~(SYS_MODE_CALIBRATED);
            ZetVar.bCalibrationCtrl |= BASE_RE_CALIBRATION;
            ZetVar.wI2CnSPIRxIdx = 0;
        }
#endif
#ifdef FEATURE_I2C_CMD_F8_ENABLE
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_F8_WDT)
        {
             #ifdef FEATURE_SPI_TRANSFER
						/// The last command bytes
						if(ZetVar.bInterfaceType == SPI_TYPE)
						{
								ZetVar.wI2CnSPIRxLen = TP_CMD_F8_CMD_LEN;
								if(ZetVar.pbI2CnSPIRxData[1] == TRUE)
                {
                    /// Enable WDT
                    SYSWatchDogEnable();
                }
                else
                {
                    /// Disable  WDT
                    SYSWatchDogDisable();
                }
                ZetVar.wI2CnSPIRxIdx = 0;
						}
						else
            #endif
            {
			            /// The last command bytes
			            if(ZetVar.wI2CnSPIRxIdx  == 1)
			            {
			                ZetVar.wI2CnSPIRxLen = TP_CMD_F8_CMD_LEN;
			            }
			            else if(ZetVar.wI2CnSPIRxIdx == TP_CMD_F8_CMD_LEN)
			            {
			                if(ZetVar.pbI2CnSPIRxData[1] == TRUE)
			                {
			                    /// Enable WDT
			                    SYSWatchDogEnable();
			                }
			                else
			                {
			                    /// Disable  WDT
			                    SYSWatchDogDisable();
			                }
			                ZetVar.wI2CnSPIRxIdx = 0;
			            }
            }
        }
#endif
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_F9_REVISION_READ)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_F9_CMD_LEN;

            /// The last command bytes
            //if(ZetVar.wI2CnSPIRxIdx == (TP_CMD_F9_CMD_LEN))
            {
                ZetVar.bF9Buf[0] = LOBYTE(FW_VERSION);
                ZetVar.bF9Buf[1] = HIBYTE(FW_VERSION);
                ZetVar.bF9Buf[2] = LOBYTE(ZetDF.cDevData.scDevData.wDataFlashVersion);
                ZetVar.bF9Buf[3] = HIBYTE(ZetDF.cDevData.scDevData.wDataFlashVersion);

                ZetVar.wI2CnSPITxLen = TP_CMD_F9_TX_DATA_LEN;
								/// Link the buffer
								LinkBuffer((BYTE xdata *)(& ZetVar.bF9Buf[0]));
                I2C_INT_LOW();
            }
        }
#ifdef FEATURE_I2C_CMD_FA_CALIB_BASETRACK_CTRL
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_FA_CALIB_BASETRACK_CTRL)
        {
         	#ifdef FEATURE_SPI_TRANSFER
					/// The last command bytes
					if(ZetVar.bInterfaceType == SPI_TYPE)
					{
							ZetVar.wI2CnSPIRxLen = TP_CMD_FA_CMD_LEN;
							if(ZetVar.pbI2CnSPIRxData[1] == TRUE)
              {
                  ZetVar.bCalibrationCtrl |= BASE_TRACK_RECALIBRAION_TOTAL_CTRL;
              }
              else
              {
                  ZetVar.bCalibrationCtrl &= ~BASE_TRACK_RECALIBRAION_TOTAL_CTRL;
              }
              ZetVar.wI2CnSPIRxIdx = 0;
					}
					else
          #endif
					{	
	            /// The last command bytes
	            if(ZetVar.wI2CnSPIRxIdx  == 1)
	            {
	                ZetVar.wI2CnSPIRxLen = TP_CMD_FA_CMD_LEN;
	            }
	            else if(ZetVar.wI2CnSPIRxIdx == TP_CMD_FA_CMD_LEN)
	            {
	                if(ZetVar.pbI2CnSPIRxData[1] == TRUE)
	                {
	                    ZetVar.bCalibrationCtrl |= BASE_TRACK_RECALIBRAION_TOTAL_CTRL;
	                }
	                else
	                {
	                    ZetVar.bCalibrationCtrl &= ~BASE_TRACK_RECALIBRAION_TOTAL_CTRL;
	                }
	                ZetVar.wI2CnSPIRxIdx = 0;
	            }
           }
        }
#endif
#ifdef FEATURE_I2C_CMD_FB_DEVICE_NAME
        else if (ZetVar.pbI2CnSPIRxData[0] == TPCMD_FB_DEVICE_NAME_READ)
        {
            ZetVar.wI2CnSPIRxLen = TP_CMD_FB_CMD_LEN;

            /// The last command bytes
            //if(ZetVar.wI2CnSPIRxIdx == (TP_CMD_F9_CMD_LEN))
            {
                BYTE i;
                for(i=0; i<FB_BUF_LEN; i++)
                {
                    ZetVar.bFBBuf[i] = ZetVar2.CompileDeviceName[i];//(ZetDF.cDevData.scDevData.pDeviceName[i]);
                }

                ZetVar.wI2CnSPITxLen = TP_CMD_FB_TX_DATA_LEN;
								/// Link the buffer
								LinkBuffer((BYTE xdata *)(& ZetVar.bFBBuf[0]));
                I2C_INT_LOW();
            }
        }
#endif
        else if((ZetVar.pbI2CnSPIRxData[0] & 0xF0) == TPCMD_80_SRAM_ACCESS) //0x8n cmd
        {
            //SPI type may has problem!!
            // seen as write ZetPara Cmd, 3 BYTEs
				   	#ifdef FEATURE_SPI_TRANSFER
						if(ZetVar.bInterfaceType == SPI_TYPE)
					  {
							ZetVar.dwSramAddr = BASE_SRAM_BA | ((ZetVar.pbI2CnSPIRxData[0] & 0x0F)<<8);
							ZetVar.dwSramAddr |= ZetVar.pbI2CnSPIRxData[1]; 						 
							ZetVar.bI2cStatus |= I2C_STA_SRAM_RW_MODE; ///< 1: enable SRAM Read Mode
	            WRITE_XDATA((uintptr_t)ZetVar.dwSramAddr, ZetVar.pbI2CnSPIRxData[2]);
	            //ZetVar.dwSramAddr++;
						}
						else
						#endif
						{
		            if(ZetVar.wI2CnSPIRxIdx  == 1)
		            {
		                ZetVar.dwSramAddr = BASE_SRAM_BA | ((ZetVar.pbI2CnSPIRxData[0] & 0x0F)<<8);
		            }
		            else if(ZetVar.wI2CnSPIRxIdx == 2)
		            {
		                ZetVar.dwSramAddr |= ZetVar.bI2cData;              
		                ZetVar.bI2cStatus |= I2C_STA_SRAM_RW_MODE; ///< 1: enable SRAM Read Mode

										LinkBuffer((BYTE xdata *)(uintptr_t)(ZetVar.dwSramAddr));
										ZetVar.wI2CnSPIRxIdx = 2; // set back 2
										//ZetVar.pbI2CnSPITxData = (BYTE xdata *)(uintptr_t)(ZetVar.dwSramAddr);
		                //ZetVar.wI2CnSPITxIdx = 0;
		                ////ZetVar.wI2CnSPIRxIdx = 0;
		                //I2C_INT_LOW();
		            }
		            else
		            {
		                WRITE_XDATA((uintptr_t)ZetVar.dwSramAddr, ZetVar.bI2cData);
		                ZetVar.dwSramAddr++;
		            }
						}
            I2C_INT_HIGH();
        }
				else if((ZetVar.pbI2CnSPIRxData[0]) == TPCMD_90_SRAM_READ) //0x90 cmd, LM, test 
				{ 
					ZetVar.wI2CnSPITxLen = TP_CMD_90_TX_DATA_LEN;  
					LinkBuffer((BYTE xdata *)(&ZetVarPara.bCmdPara[0])); 
					I2C_INT_LOW(); 
				}
#ifdef SPI_FLASH_WR_ENABLE				
				else if((ZetVar.pbI2CnSPIRxData[0]) == 0x9B) //0x9B cmd, LM, test
				{
						SPIFlashUIDRead(0x00,0x00,0x00,17,(BYTE volatile xdata *)(&bFlashBuffer[0]));
						ZetVar.wI2CnSPITxLen = 16; 
						LinkBuffer((BYTE xdata *)(&bFlashBuffer[1]));
						I2C_INT_LOW();
				}
#endif				
}



/**
 * @brief  I2CnSPIDispatchCmd()
 *
 *  Dispatch Cmd
 *
 * @return NULL
 *
 */
void I2CnSPIDispatchCmd()
{
#ifdef FEATURE_CUSTOMER_PROTOCOL    
		if(ZetVar.pbI2CnSPIRxData[0] == 0xFF) //new reserve cmd(switching mode): FF FF FF FF, FF FF nn FF, 
		{
				if((ZetVar.wI2CnSPIRxIdx==4||ZetVar.bInterfaceType==SPI_TYPE) && ZetVar.pbI2CnSPIRxData[3]==0xFF && ZetVar.pbI2CnSPIRxData[1]==0xFF)
				{
          if(ZetVar.pbI2CnSPIRxData[2]!=0xFF)
          {
						ZetVar.bWorkingState=ZetVar.pbI2CnSPIRxData[2];
          }

					bSwitchResponse[5]=ZetVar.bWorkingState;
					bSwitchResponse[6]=0xA5; //special pattern for detect mcu running
					bSwitchResponse[7]=0x5A; //special pattern for detect mcu running
					//CustomerResponseArray(bTmpByte,8, UNCHANGE_IDX);
					ZetVar.wI2CnSPITxLen=8;
					LinkBuffer((BYTE *)(&bSwitchResponse[0]));				
					I2C_INT_LOW();
          return;
				}	
		}

    //other cmd
    if((ZetVar.bWorkingState==WORKING_STATE_CUSTOMER_INITIAL)	||
       (ZetVar.bWorkingState==WORKING_STATE_CUSTOMER_NORMAL))
    {    
    	if(ZetVar.bInterfaceType==SPI_TYPE)
			{
		    #ifdef FEATURE_SPI_TRANSFER
		    SPIDispatchCustomerCmd();  //sample code
		    #endif
			}
			else if(ZetVar.bInterfaceType==I2C_TYPE)
      {
			  I2CDispatchCustomerCmd();	
      }
    }
    else if(ZetVar.bWorkingState==WORKING_STATE_ZET_CMD_STATE) //switch to Zetouch mode			
#endif
    {
        I2CnSPIDispatchZetCmd();
    }
}

void I2C_INT_LOW(void)
{
    CLR_BIT(BASE_GPIO_BA + INTPortReg,INTPinBit);
}
void I2C_INT_HIGH(void)
{
    SET_BIT(BASE_GPIO_BA + INTPortReg,INTPinBit);
}

#ifdef FEATURE_MCU_LIB_ENABLE
BOOL I2C_INT(void)
{
  if(READ_REG(BASE_GPIO_BA + INTPortReg) & INTPinBit)
  {
      return TRUE;
  }
  else
  {
      return FALSE;
  }
}
#endif

#ifdef FEATURE_SELF_SCAN
void MSSelfScanStart(void)
{
	U1    bk_pn_num;
	U4    bk_REG32_RX_PN_CODE_OUT0;
	U1    bk_REG32_AD_CTRL10;
	U1    bk_REG32_DAC_CTRL0;
	U1    bk_REG32_TX_CTRL;
	U1    bk_REG32_TX_BUF;
	U1    bk_REG32_VREF6V;
	U1    bk_REG32_AD_CTRL13;
	U1    bk_REG32_STR_PN_CTRL_1;
	U1    bk_REG32_PN_CTRL_2;
	U1    bk_REG32_SELF_CTRL;
	U1		bk_REG32_PN_CTRL;

	/////  set Self mode scan flag
	sSelfScan.bSelfMode= 1;

	SET_BIT(REG32_PN_CTRL, RG_AMP_FUNC_AUTO_EN);	

	/// TX manual mode,  disable TX auto config
	SET_BIT(REG32_STR_PN_CTRL,RG_MANUAL);

	//////////////////////////////////////////////
	/// Back up parameters
	//////////////////////////////////////////////
	bk_pn_num = READ_REG(REG32_TRY_PNCNT+2);
	bk_REG32_AD_CTRL10 = READ_REG(REG32_AD_CTRL10);
	bk_REG32_DAC_CTRL0= READ_REG(REG32_DAC_CTRL0);
	bk_REG32_TX_CTRL= READ_REG(REG32_TX_CTRL);
	bk_REG32_TX_BUF= READ_REG(REG32_TX_BUF);
	bk_REG32_AD_CTRL13= READ_REG(REG32_AD_CTRL13);
	bk_REG32_VREF6V =  READ_REG(REG32_VREF6V);
	bk_REG32_STR_PN_CTRL_1 =   READ_REG(REG32_STR_PN_CTRL+1);
	bk_REG32_PN_CTRL_2 = READ_REG(REG32_PN_CTRL+2);
	bk_REG32_RX_PN_CODE_OUT0 = READ_REG32(REG32_RX_PN_CODE_OUT0);
	bk_REG32_SELF_CTRL  = READ_REG(REG32_SELF_CTRL);
	bk_REG32_PN_CTRL = READ_REG(REG32_PN_CTRL);

	/////////////////////////////////
	/////   Self Scan Setup       //////////
	/////////////////////////////////

	/////////////////  PN Num = 1
	//WRITE_REG(REG32_TRY_PNCNT+2, 1); 
	WRITE_REG(REG32_TRY_PNCNT+2, 2); 
	WRITE_REG(REG32_PN_CTRL,READ_REG(REG32_PN_CTRL)&0x8f);

	// pwd  RG_RX_PWREF_EN_15V
	SET_BIT(REG32_AD_CTRL10,1<<6);   

	// RG_RX_FRONT_MODE_15V  00: self mode 
	CLR_BIT(REG32_AD_CTRL10,1<<5);   
	CLR_BIT(REG32_AD_CTRL10,1<<4);   

	// pwd  RG_DAC0_PWD_15V
	SET_BIT(REG32_DAC_CTRL0,1<<0);   

	// pwd  RG_TX_LPF0_PWD_15V
	SET_BIT(REG32_TX_CTRL,1<<0);  

	// pwd  RG_TXBUF0_PWD_15V
	SET_BIT(REG32_TX_BUF,1<<0);  

	// pwd  RG_VREF6V_PWD_15V
	SET_BIT(REG32_VREF6V,1<<3);  

	// on  RG_DAC6V_ENE_15V
	CLR_BIT(REG32_AD_CTRL13,1<<1);  

	// on RG_DACS_PWD_15V
	CLR_BIT(REG32_AD_CTRL13,1<<4);  

	//// DAC square mode
	CLR_BIT(REG32_STR_PN_CTRL+1,1<<0); 

	////  str_pn_d,   sync DAC_CLK
	SET_BIT(REG32_PN_CTRL+2,1<<4);

	//// DA_SELF_CK divider
	//SET_BIT(SELF_CTRL,1<<5); 
	SET_BIT(REG32_SELF_CTRL,1<<4); 

	/////   PN code
	WRITE_REG32(REG32_RX_PN_CODE_OUT0, 0x00000001); 
	

	//////////////////////////////////////////////
	/////   Self   PAD/Cannnel    Configration   //////////
	//////////////////////////////////////////////
/*
	///example    TR4  connect to ADC0
	PAD_SEL[4] = 0x10;
	///example    TR5  connect to ADC1
	PAD_SEL[5] = 0x11;
*/

	//PAD_SEL[48] = 0x10;
	//PAD_SEL[49] = 0x10;
	//PAD_SEL[49] =0x00; ///
	//PAD_SEL[50] = 0x12;
	PAD_SEL[ZetDF.cGlobalData.scForceTouchInformation.bFSenseAxisMax] = 0x10; //0x02;

	//SET_BIT(REG32_AD_CTRL13,1<<1);
	//SET_BIT(REG32_AD_CTRL13,1<<0);

	//////////////////////////////////////////////
	/////   Self   Scan Start                           //////////
	//////////////////////////////////////////////

	////  set interrupt flag = 0
	sSelfScan.bSelfModeINT = 0;

	//////   modem scan (self scan start)
	SET_BIT(REG_STR_PN_CTRL_2,1<<0);

	/////  while finish
	while(sSelfScan.bSelfModeINT== 0);

	sSelfScan.bSelfModeINT = 0;

	//PAD_SEL[48] = 0x00;
	//PAD_SEL[49] = 0x00;
	//PAD_SEL[50] = 0x00;
	PAD_SEL[ZetDF.cGlobalData.scForceTouchInformation.bFSenseAxisMax] = 0x00;

	//////////////////////////////////////////////
	////// restore parameters
	//////////////////////////////////////////////
	WRITE_REG(REG32_TRY_PNCNT+2, bk_pn_num); 
	/////   PN code
	WRITE_REG32(REG32_RX_PN_CODE_OUT0, bk_REG32_RX_PN_CODE_OUT0); 

	WRITE_REG(REG32_AD_CTRL10, bk_REG32_AD_CTRL10);
	WRITE_REG(REG32_DAC_CTRL0, bk_REG32_DAC_CTRL0);
	WRITE_REG(REG32_TX_CTRL, bk_REG32_TX_CTRL);
	WRITE_REG(REG32_TX_BUF, bk_REG32_TX_BUF);
	WRITE_REG(REG32_AD_CTRL13, bk_REG32_AD_CTRL13);
	WRITE_REG(REG32_VREF6V, bk_REG32_VREF6V);
	WRITE_REG(REG32_STR_PN_CTRL+1, bk_REG32_STR_PN_CTRL_1);
	WRITE_REG(REG32_PN_CTRL+2, bk_REG32_PN_CTRL_2);
	WRITE_REG(REG32_PN_CTRL, bk_REG32_PN_CTRL);

	WRITE_REG(REG32_SELF_CTRL, bk_REG32_SELF_CTRL);

	/// TX manual mode,  disable TX auto config
	CLR_BIT(REG32_STR_PN_CTRL,RG_MANUAL);

	CLR_BIT(REG32_PN_CTRL, RG_AMP_FUNC_AUTO_EN);	
}
#endif

#ifdef FEATURE_TX_RX_IR
void IRInit(void)
{	
  /// IR_EN as output High
	SET_BIT(P2,1<<0);  ///ir_en, set low TR7
	SET_BIT(IODIR_P2,1<<0);  ///ir_en, set output TR7
	SET_BIT(AD_CTRL14,1<<0); // ir_en DIO en, TR7
#ifdef FEATURE_IR_BASE
 	sIRScan.bIRBaseRound = 1;
#endif	
}

void IRSetup( unsigned char freq_multi, unsigned char sqrt_mode,
             unsigned char rx_sine_len)
{
	U1    bk_pn_num;
	U4    bk_REG32_RX_PN_CODE_OUT0;
	U4    bk_REG32_SINE_ROM_LEN_RX;
	U4    bk_REG32_TX_PN_CODE;
	U4    bk_REG32_RX_HOLD_CYCE_ADDR;
	U4    bk_REG32_LPF_COEFF_HALF_LEN;
	U1    bk_TXSIN_NUM;
	U4    bk_REG32_RX_FREQ_MULTIPLEA;
	U4    bk_REG32_SINE_FREQ_SCALE;
	U4    bk_REG32_FRONT_ADC_RST_CNT;
	U4    bk_REG16_CHIP_2_CHIP_DURA;
	U1    bk_REG32_VREF6V;
	U1    bk_REG32_TX_CTRL_1_;
	U1    bk_REG32_DAC_CTRL2_1_;
	U1    bk_REG32_VREF3V;
	U1		bk_REG32_PN_CTRL;

	sIRScan.bIR_ADmode = 1;

	SET_BIT(REG32_PN_CTRL, RG_AMP_FUNC_AUTO_EN);	

	/////   adc mode
	SET_BIT(REG32_AD_CTRL10,1<<5);   
	SET_BIT(REG32_AD_CTRL10,1<<4); 

	bk_pn_num = READ_REG(REG32_TRY_PNCNT+2);
	bk_REG32_RX_PN_CODE_OUT0 = READ_REG32(REG32_RX_PN_CODE_OUT0);
	bk_REG32_SINE_ROM_LEN_RX = READ_REG32(REG32_SINE_ROM_LEN_RX);
	bk_REG32_TX_PN_CODE = READ_REG32(REG32_TX_PN_CODE);
	bk_REG32_RX_HOLD_CYCE_ADDR = READ_REG32(REG32_RX_HOLD_CYCE_ADDR);
	bk_REG32_LPF_COEFF_HALF_LEN = READ_REG32(REG32_LPF_COEFF_HALF_LEN);
	bk_TXSIN_NUM = READ_REG(REG32_TRY_PNCNT);
	bk_REG32_RX_FREQ_MULTIPLEA = READ_REG32(REG32_RX_FREQ_MULTIPLEA);
	bk_REG32_SINE_FREQ_SCALE = READ_REG32(REG32_SINE_FREQ_SCALE);
	bk_REG32_FRONT_ADC_RST_CNT = READ_REG32(REG32_FRONT_ADC_RST_CNT);
	bk_REG16_CHIP_2_CHIP_DURA = READ_REG32(REG16_CHIP_2_CHIP_DURA);
	bk_REG32_VREF6V =  READ_REG(REG32_VREF6V);
	bk_REG32_TX_CTRL_1_ =  READ_REG(REG32_TX_CTRL+1);
	bk_REG32_DAC_CTRL2_1_ = READ_REG(REG32_DAC_CTRL2+1);
	bk_REG32_VREF3V   = READ_REG(REG32_VREF3V);
	bk_REG32_PN_CTRL  = READ_REG(REG32_PN_CTRL);

#ifdef FEATURE_IR_BASE
	if( sIRScan.bIRBaseRound == 1 )
	{
		//// set ir_out pin as No-Signal			set drive-pin to TR53(NULL-pin)
		WRITE_REG(REG32_TRNUM_OUT0, 0x35);
	}
	else
	{
		//// set ir_out pin as TX 						set drive-pin to TR6
		WRITE_REG(REG32_TRNUM_OUT0, 0x06);
	}
#else
	//// set ir_out pin as TX 							set drive-pin to TR6
	WRITE_REG(REG32_TRNUM_OUT0, 0x06);
#endif

	/// ir_in as ADC0
	PAD_SEL[14] = 0x10;

	/////////////////  PN Num = 1
	WRITE_REG(REG32_TRY_PNCNT+2, 1); 

	/*
	//// TX square  mode 0: srqt  1: dac mode
	if(sqrt_mode==0x01){
	CLR_BIT(REG_STR_PN_CTRL_1, 1<<0);
	}else{
	SET_BIT(REG_STR_PN_CTRL_1, 1<<0);
	}
	*/

	/////   PN code
	WRITE_REG32(REG32_RX_PN_CODE_OUT0, 0x00000001); 
	WRITE_REG32(REG32_TX_PN_CODE, 0x00000001); 

	////
	WRITE_REG32(REG32_RX_HOLD_CYCE_ADDR,0x03); 

	WRITE_REG32(REG32_SINE_ROM_LEN_RX,rx_sine_len);  /// RX SINE ROM Length

	WRITE_REG32(REG32_LPF_COEFF_HALF_LEN,(rx_sine_len*2)-1); //   coeff len =( 2* tx_sine_len  ) -1
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////  bit6-0: Tone1 Sine Num / Bit14-8: Tone2 Sine num / bit21-16: chip-num / bit23:dual or sigle Tone  /bit22: NA/////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	WRITE_REG(REG32_PN_CTRL,(bk_REG32_PN_CTRL & 0x8F) | 0x10);
	
	WRITE_REG(REG32_TRY_PNCNT, freq_multi+1);  ///usually TX sine num > fre_multi, maybe morethan 1 ....OK   

	WRITE_REG32(REG32_RX_FREQ_MULTIPLEA,freq_multi); // multiple frequence //Jason_0902 OSR800

	/////////////////////////////////////////////////////////////////////////////////
	//////////   SINE_FREQ_SCALE  ///////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	//  bit Tone A frequence multiple
	//  bit Tone B frequence multiple
	//  bit 27-16, TX Base frequence ROM length
	//
	//////////////////////////////////////////////////////////////////////////////


	WRITE_REG32(REG32_SINE_FREQ_SCALE,(0xffff0000 &(rx_sine_len<<19))| (0xffff&freq_multi)); //tx len = 4*rx_sine_len

	///ADC_FRONT_RST
	WRITE_REG32(REG32_FRONT_ADC_RST_CNT,0x002e002e); // adc front  rst 


	///////////////////////////////////////////////////////////////////////
	// 	      
	//   .RG_CHIP_2_CHIP_DURA( {PN_DURCNT1_OUT,PN_DURCNT0_OUT}),
	//   tx chip to chip delay, for TX vcm settle
	//
	//   .RG_FRM_2_FRM_DURA(  {PN_DURCNT3_OUT,PN_DURCNT2_OUT}),
	//   tx delay time before first chip start,  for TX vcm settle
	//
	//////////////////////////////////////////////////////////////////////////
	WRITE_REG32(REG16_CHIP_2_CHIP_DURA,0x00300030);

	SET_BIT(REG32_VREF6V, 1<<4);


	////TX gain down
	WRITE_REG(REG32_TX_CTRL+1,0x00);

	WRITE_REG(REG32_DAC_CTRL2+1,0x00);

	///vcm bias to 1.2
	CLR_BIT(REG32_VREF3V, 1<<1);
	CLR_BIT(REG32_VREF3V, 1<<0);

	sIRScan.bIR_ADmode_INT = 0;

	//////   modem scan
	SET_BIT(REG_STR_PN_CTRL_2,1<<0);


	/////  while finish
	while(sIRScan.bIR_ADmode_INT == 0){
	SET_BIT(REG32_VREF6V, 1<<4);
	}

	sIRScan.bIR_ADmode_INT = 0;

	/////////////////  PN Num = 1
	WRITE_REG(REG32_TRY_PNCNT+2, bk_pn_num); 
	/////   PN code
	WRITE_REG32(REG32_RX_PN_CODE_OUT0, bk_REG32_RX_PN_CODE_OUT0); 
	WRITE_REG32(REG32_TX_PN_CODE, bk_REG32_TX_PN_CODE); 

	////
	WRITE_REG32(REG32_RX_HOLD_CYCE_ADDR,bk_REG32_RX_HOLD_CYCE_ADDR); 

	WRITE_REG32(REG32_SINE_ROM_LEN_RX,bk_REG32_SINE_ROM_LEN_RX);  /// RX SINE ROM Length

	WRITE_REG32(REG32_LPF_COEFF_HALF_LEN,bk_REG32_LPF_COEFF_HALF_LEN); //   coeff len =( 2* tx_sine_len  ) -1

	WRITE_REG(REG32_TRY_PNCNT,bk_TXSIN_NUM);  ///usually TX sine num > fre_multi, maybe morethan 1 ....OK   

	WRITE_REG32(REG32_RX_FREQ_MULTIPLEA,bk_REG32_RX_FREQ_MULTIPLEA); // multiple frequence //Jason_0902 OSR800


	WRITE_REG32(REG32_SINE_FREQ_SCALE,bk_REG32_SINE_FREQ_SCALE); //tx len = 4*rx_sine_len

	///ADC_FRONT_RST
	WRITE_REG32(REG32_FRONT_ADC_RST_CNT,bk_REG32_FRONT_ADC_RST_CNT); // adc front  rst 

	WRITE_REG32(REG16_CHIP_2_CHIP_DURA,bk_REG16_CHIP_2_CHIP_DURA);

	WRITE_REG(REG32_VREF6V, bk_REG32_VREF6V);

	WRITE_REG(REG32_TX_CTRL+1, bk_REG32_TX_CTRL_1_);

	WRITE_REG(REG32_DAC_CTRL2+1, bk_REG32_DAC_CTRL2_1_);

	WRITE_REG(REG32_VREF3V, bk_REG32_VREF3V);

	WRITE_REG(REG32_PN_CTRL,bk_REG32_PN_CTRL);

	CLR_BIT(REG32_PN_CTRL, RG_AMP_FUNC_AUTO_EN);	

	/// ir_in as gnd
	PAD_SEL[14] = 0x00;

	/////  back to normal mode
	CLR_BIT(REG32_AD_CTRL10,1<<5);   
	SET_BIT(REG32_AD_CTRL10,1<<4); 
}

#endif


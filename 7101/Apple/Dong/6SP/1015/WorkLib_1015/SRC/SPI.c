/**
 * @file SPI.c
 *
 * SPI Related Function
 *
 * @version $Revision: 62 $
 * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @note
*/


#include"ZetDEF.h"

#ifdef FEATURE_SPI_TRANSFER	
#define SPI_BUFFER_SIZE 100
BYTE SramSPI[SPI_BUFFER_SIZE];

BYTE CustomRes4Cmd1A_CODE[] = {0x1f,0x01,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79,0x48,0x79};
BYTE CustomRes4Cmd19_CODE[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
BYTE CustomRes4Cmd1A_2[] = {0x00,0x00};
BYTE CustomRes4Cmd1d[] = {0xe1,0x0e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xef,0x00};
BYTE CustomRes4CmdEdata[50] = {0};
BYTE CustomRes4CmdEcmdbuff[20];
BYTE CustomCmdList[5][2] = {{0,0},{0,0},{0,0},{0,0},{0,0}} ;
BYTE CustomCmdListCnt=0;
BYTE CustomExDataCnt=0;


/**
 * @brief  SPIInit()
 *
 *  Init the SPI module
 *
 * @return NULL
 *
 */
void SPIInit(void)
{
	///////// set 4K sram as write buffer
	SPIBufferWriteEn();	 
	// SPISetNonIntTypeCmd();	 

	SPISetWakeupTypeCmd();				
	SPISramInit();
	SPISramReConfigure();
	// SPI2 continune command setting
	WRITE_REG(REG_SPI2_CMD0+2,0xf2);	
#if 1 //def FEATURE_DEVELOP1
	//setting all SPI cmd can wake-up mcu from deep-sleep mode
	SET_BIT(REG_SPI2_STATUS,1<<6);
#endif		
	// SPI buffer offset setting
	WRITE_REG32(SPI2_BUF_ADR,0);	
}

/*
void SPI_INT_HIGH(void)
{
   #if 0 //def FEATURE_DEVELOP1
	 //SET_MASK_BIT16(REG32_P3,(P35));
	 WRITE_REG16(REG32_P3, 0x2020);
   #else
   SET_BIT(REG32_P3,P35);
	 #endif
}

void SPI_INT_LOW(void)
{
   #if 0 //def FEATURE_DEVELOP1
	 //CLR_MASK_BIT16(REG32_P3,(P35));
	 WRITE_REG16(REG32_P3, 0x2000);
   #else
   CLR_BIT(REG32_P3,P35);
	 #endif
}

BYTE SPI_INT_STATUS(void)
{
   return (BYTE)I2C_INT();
}
*/

/**
 * @brief  SPISetNonIntTypeCmd()
 *
 *  Init the SPI module
 *
 * @return NULL
 *
 */
void SPISetNonIntTypeCmd(void)
{  
			 WRITE_REG(REG_SPI2_CMD0,0xf2);//// mask cmd, allow Max 4 cmd 
}

/**
 * @brief  SPISetWakeupTypeCmd()
 *
 *  Init the SPI module
 *
 * @return NULL
 *
 */
void SPISetWakeupTypeCmd(void)
{   
	//   WRITE_REG(REG_SPI2_CMD1,0x88);//////  wake up cmd, allow Max 4 cmd
	   WRITE_REG(REG_SPI2_CMD1,0x1C); 
}

/**
 * @brief  SPIBufferWriteEn()
 *
 *  Init the SPI module
 *
 * @return NULL
 *
 */
inline void SPIBufferWriteEn(void)
{  
    ///////// set 4K sram as write buffer
		CLR_BIT(REG_SPI2_STATUS,SPIBUF_WEN); //bit4:0:Can be written by External SPI HOST	 
}


void SPISramInit(void)
{  
	// BYTE i;
	ZetVar.bInterfaceType=SPI_TYPE;

	//SPI slave Rx buffer
	WRITE_REG32(REG_SPI2_TX_DMA_DEST_ADR,(unsigned char*)&bI2CRxBuf[0] );  
	WRITE_REG32(REG_SPI2_TX_DMA_DEST_MAX_ADR,((unsigned char*)&bI2CRxBuf[MAX_RX_BUF_LEN-1])); 	
	WRITE_REG32(REG_SPI2_TX_DMA_CNT,(MAX_RX_BUF_LEN));  

#ifdef FEATURE_SPI_CMD_REG_TYPE
	ZetVar.pbI2CnSPIRxData =(BYTE *)SPI2_DATA0; //set in I2Cinit, allocate others if I2C conflict with SPI
#endif

	//ZetVar.pbI2CnSPITxData = (BYTE *)&SramSPI[0];
	//SPI slave Tx buffer
	WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[0] );	
	WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[ ZetVar.wI2CnSPITxLen-1]));	 
	WRITE_REG32(REG_SPI2_RX_DMA_CNT,(ZetVar.wI2CnSPITxLen));	

	//WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[0] );	
	//WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[ZetVar.wI2CnSPITxLen-1]));	 
	//WRITE_REG32(REG_SPI2_RX_DMA_CNT,(ZetVar.wI2CnSPITxLen)-1);	

	CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO	 
#ifdef FEATURE_SPI_CMD_REG_TYPE
	SET_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO
#endif

	CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI	 
	SET_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI    
	///// 	RX first BUFFER / DUMMY BUFFER		
	WRITE_REG(REG_SPI2_RX_BUF, ZetVar.bROMCheckSum);	
}

#if 1

void SPISramReConfigure(void)
{ 			   
	WORD wlocalTxLen;

	if(ZetVar.bInterfaceType!=SPI_TYPE)
	{
	  return;
	}

	wlocalTxLen=ZetVar.wI2CnSPITxLen;

	if((ZetVar.bWorkingState==WORKING_STATE_ZET_CMD_STATE)
		||((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x20)==0x00))
	{
		//SPI slave Tx buffer, in Zetouch cmd state
		WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[0] );	
		if(wlocalTxLen>1)
		{			 
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[wlocalTxLen-1]));	 
			//WRITE_REG32(REG_SPI2_RX_DMA_CNT,(wlocalTxLen-1));
		}
		else
		{
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[0]));  
		}
		WRITE_REG32(REG_SPI2_RX_DMA_CNT,(wlocalTxLen));
	}
	else
	{
		//SPI slave Tx buffer, in custom state and 1st BYTE is data(no dummy BYTE) for iphone case 
		if(wlocalTxLen>1)
		{
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[1] );			 
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[wlocalTxLen-1]));	 
			WRITE_REG32(REG_SPI2_RX_DMA_CNT,65535);//(wlocalTxLen-1));
		}
		else
		{
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[0] );			 
			WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[0]));	 
			WRITE_REG32(REG_SPI2_RX_DMA_CNT,(wlocalTxLen));
		}
	}

	CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO		 
	SET_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO
	//CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI
	// SET_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI   
	if((ZetVar.bWorkingState==WORKING_STATE_ZET_CMD_STATE)
		||((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x20)==0x00))
	{
	}
	else if((ZetDF.cMiscRegister.scInterface.bRegSPIMode&0x20)==0x20)
	{
	 //in custom state and 1st BYTE is data(no dummy BYTE) for iphone case 
	 WRITE_REG(REG_SPI2_RX_BUF, ZetVar.pbI2CnSPITxData[0]); 
	}		 
}

#else
void SPISramReConfigure(void)
{ 			   
     if(ZetVar.bInterfaceType!=SPI_TYPE)
  	 {
        return;
  	 }
     //SPI slave Tx buffer
	   WRITE_REG32(REG_SPI2_RX_DMA_SRC_ADR,(unsigned char*)&ZetVar.pbI2CnSPITxData[0] );	
	   WRITE_REG32(REG_SPI2_RX_DMA_SRC_MAX_ADR,((unsigned char*)&ZetVar.pbI2CnSPITxData[ ZetVar.wI2CnSPITxLen-1]));	 
	   WRITE_REG32(REG_SPI2_RX_DMA_CNT,(ZetVar.wI2CnSPITxLen));	
	 
		 CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO
		 //CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI
		 
		 SET_BIT(REG_SPI2_FIFO_CTRL, 1<<0 );	 /// RX fifo		 MISO
	  // SET_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );	/// for Master TX fifo	 MOSI  

    
}
#endif






	
/**
 * @brief  SPIRxIsr()
 *
 *	Handle SPI RxIsr
 *
 * @return NULL
 *
 */

void SPIRxIsr(void)
{ 

  ZetVar.bInterfaceType=SPI_TYPE;
	#ifdef FEATURE_SPI_CMD_REG_TYPE
	ZetVar.pbI2CnSPIRxData =(BYTE *)SPI2_DATA0; //set in I2Cinit, allocate others if I2C conflict with SPI
  #endif
	//SPIDispatchCmd();
  I2CnSPIDispatchCmd();
	 
	#ifdef FEATURE_SPI_CMD_REG_TYPE
	#else
	//CLR_BIT(0x1A0015, 1<<0 ); /// for Master TX fifo 	MOSI
	//SET_BIT(0x1A0015, 1<<0 ); /// for Master TX fifo 	MOSI	 
	CLR_BIT(REG32_I2C_CTRL+1, 1<<0 ); /// for Master TX fifo 	MOSI
	SET_BIT(REG32_I2C_CTRL+1, 1<<0 ); /// for Master TX fifo 	MOSI	 
	
	CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<1 ); /// for Master TX fifo	 MOSI
	SET_BIT(REG_SPI2_FIFO_CTRL, 1<<1 ); /// for Master TX fifo	 MOSI 	
    
	CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<0 ); 	/// RX fifo 		MISO
				//CLR_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );  /// for Master TX fifo 	MOSI
				
	SET_BIT(REG_SPI2_FIFO_CTRL, 1<<0 ); 	/// RX fifo 		MISO
			 // SET_BIT(REG_SPI2_FIFO_CTRL, 1<<1 );  /// for Master TX fifo 	MOSI			
  #endif 
	
////////////////////////////////////////////////////////////////
/// Maybe need to clear cmd buffer after parsed!!!
////////////////////////////////////////////////////////////////
		///--------------------------------------------///
		/// Clear interrupt flag
		///--------------------------------------------///
    WRITE_REG32(SPI2_IF, 0x0);
}

#ifdef FEATURE_CUSTOMER_PROTOCOL
void SPIDispatchCustomerCmd(void)
{
#ifdef FEATURE_6SP_PROTOCOL
	BYTE i=0;
	BYTE bCustomExCksumCnt=0;
	if((ZetVar.pbI2CnSPIRxData[0] == 0xeb) && (ZetVar.pbI2CnSPIRxData[2] == 0x00))
	{
		BYTE bPacketNumber = CustomerVar.bDoubleBuffer[37];
		WORD wCheckSum = 0;
		
		CustomerVar.bDoubleBuffer[17] = ZetVar.pbI2CnSPIRxData[1];
		CustomerVar.bDoubleBuffer[20] = 0xFC - bPacketNumber*30 - CustomerVar.bDoubleBuffer[17];
		//bPacketNumber = (CustomerVar.bDoubleBuffer[1] - 0x1A)/30;
		for(i=16;i<(45+30*bPacketNumber);i++)
		{
			wCheckSum += CustomerVar.bDoubleBuffer[i];
		}
		CustomerVar.bDoubleBuffer[45+30*bPacketNumber]=LOBYTE(wCheckSum);		
		CustomerVar.bDoubleBuffer[46+30*bPacketNumber]=HIBYTE(wCheckSum)-2;
		CustomerResponseArray((BYTE *)(&CustomerVar.bDoubleBuffer[16]),31+30*bPacketNumber,0);
		I2C_INT_HIGH();
	}
	else if((ZetVar.pbI2CnSPIRxData[0] >= 0xe0)&&(ZetVar.pbI2CnSPIRxData[0] <= 0xee))
	{	
		WORD wCustomChecksum = 0;
		CustomExDataCnt=16;

		for(i=2;i<16;i++)
		{
			CustomRes4CmdEdata[i] = 0;
		}				
    /// fill command buffer history list fifo
		CustomCmdList[CustomCmdListCnt][0] = ZetVar.pbI2CnSPIRxData[0];
		CustomCmdList[CustomCmdListCnt][1] = ZetVar.pbI2CnSPIRxData[1];

		if(ZetVar.pbI2CnSPIRxData[2] == 0)
		{	
			if(ZetVar.pbI2CnSPIRxData[0] == 0xe2)
			{		
				CustomRes4CmdEdata[0] = ZetVar.pbI2CnSPIRxData[0];
				CustomRes4CmdEdata[1] = 0x5c;
				CustomRes4CmdEdata[2] = 0x01;
				CustomRes4CmdEdata[3] = 0x90;
				CustomRes4CmdEdata[4] = 0x07;
			}
			else if((ZetVar.pbI2CnSPIRxData[0] == 0xe3))
			{		
				CustomRes4CmdEdata[0] = ZetVar.pbI2CnSPIRxData[0];					
				CustomRes4CmdEdata[1] = ZetVar.pbI2CnSPIRxData[1];
				
				if(ZetVar.pbI2CnSPIRxData[1] == 0x72)
				{
					CustomRes4CmdEdata[3] = 0x20;					
					CustomRes4CmdEdata[13] = 0xe3; //0xaa;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x73)
				{
					CustomRes4CmdEdata[3] = 0x10;
					CustomRes4CmdEdata[5] = 0x01;
					CustomRes4CmdEdata[6] = 0x02;
					CustomRes4CmdEdata[7] = 0x02;
					CustomRes4CmdEdata[8] = 0x02;
					CustomRes4CmdEdata[9] = 0x01;
					CustomRes4CmdEdata[10] = 0x01;
					CustomRes4CmdEdata[11] = 0x02;
					CustomRes4CmdEdata[12] = 0x06;
					CustomRes4CmdEdata[13] = 0x07;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x7e)
				{
					CustomRes4CmdEdata[3] = 0x2c;
					CustomRes4CmdEdata[9] = 0x19;
					CustomRes4CmdEdata[10] = 0x04;				
					CustomRes4CmdEdata[13] = 0xe6 ;//0xbe;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x9d)				
				{						
					CustomRes4CmdEdata[3] = 0x08;								
					CustomRes4CmdEdata[13] = 0x20;				
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xA1)
				{
					CustomRes4CmdEdata[3] = 0x06;
					CustomRes4CmdEdata[7] = 0x0c;
					CustomRes4CmdEdata[10] = 0x03;
					CustomRes4CmdEdata[11] = 0x0f;
					CustomRes4CmdEdata[13] = 0x08;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xAF)
				{
					CustomRes4CmdEdata[3] = 0x01;
					CustomRes4CmdEdata[6] = 0x01;
					CustomRes4CmdEdata[7] = 0x09;
					CustomRes4CmdEdata[13] = 0x20;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xBF)
				{
					CustomRes4CmdEdata[3] = 0x04;
					CustomRes4CmdEdata[5] = 0x9d;
					CustomRes4CmdEdata[6] = 0x01;
					CustomRes4CmdEdata[7] = 0x09;
					CustomRes4CmdEdata[13] = 0x20;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD0)
				{
					CustomRes4CmdEdata[3] = 0x1d;
					CustomRes4CmdEdata[5] = 0x04;
					CustomRes4CmdEdata[6] = 0x01;
					CustomRes4CmdEdata[8] = 0x1c;
					CustomRes4CmdEdata[9] = 0x01;
					CustomRes4CmdEdata[11] = 0x0f;
					CustomRes4CmdEdata[13] = 0x08;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD1)
				{
					CustomRes4CmdEdata[3] = 0x01;
					CustomRes4CmdEdata[5] = 0x5c;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD3)
				{				
					CustomRes4CmdEdata[3] = 0x0c;					
					CustomRes4CmdEdata[5] = 0x01;
					CustomRes4CmdEdata[6] = 0x2d;
					CustomRes4CmdEdata[7] = 0x0f;
					CustomRes4CmdEdata[8] = 0x06;
					CustomRes4CmdEdata[9] = 0x70; //0x63; //0x64; 
					CustomRes4CmdEdata[11] = 0x1c;
					CustomRes4CmdEdata[12] = 0x0f;
					CustomRes4CmdEdata[13] = 0x90;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD9)
				{				
					CustomRes4CmdEdata[3] = 0x10;					
					CustomRes4CmdEdata[5] = 0xb4;
					CustomRes4CmdEdata[6] = 0x1a;
					CustomRes4CmdEdata[9] = 0x79;
					CustomRes4CmdEdata[10] = 0x2f;
					CustomRes4CmdEdata[13] = 0x20;					
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xe0)
				{					
					CustomRes4CmdEdata[2] = 0x02;					
					CustomRes4CmdEdata[3] = 0x01;					
					CustomRes4CmdEdata[5] = 0xb4;
					CustomRes4CmdEdata[6] = 0x1a;
					CustomRes4CmdEdata[9] = 0x79;
					CustomRes4CmdEdata[10] = 0x2f;
					CustomRes4CmdEdata[13] = 0x20;							
				}
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xe5)
			{		
				CustomRes4CmdEdata[0]  = 0xe1;
				CustomRes4CmdEdata[1]  = 0x0e;
	
				for(i=16;i<50;i++)
				{
					CustomRes4CmdEdata[i] = 0;
				}		
				CustomRes4CmdEdata[6]  = CustomCmdList[CustomCmdListCnt][0];              /// current command
				CustomRes4CmdEdata[7]  = CustomCmdList[CustomCmdListCnt][1];			
				if(ZetVar.pbI2CnSPIRxData[1] == 0x72)
				{
					ZetVar.bWorkingState = WORKING_STATE_CUSTOMER_NORMAL;
					CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+4)%5][0];   /// before of current cmd 
					CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+4)%5][1];
					CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// after of current cmd  
					CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+3)%5][1];
					//CustomRes4CmdEdata[32] = 0x10;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x7e)
				{				
					CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// before of current cmd 
					CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+3)%5][1];
					CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+2)%5][0];   /// after of current cmd  
					CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+2)%5][1];
				}
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xe6)
			{		
				CustomRes4CmdEdata[0] = ZetVar.pbI2CnSPIRxData[0];
				CustomRes4CmdEdata[1] = ZetVar.pbI2CnSPIRxData[1];
				if(ZetVar.pbI2CnSPIRxData[1] == 0xA1)
				{
					CustomRes4CmdEdata[5] = 0x0c;
					CustomRes4CmdEdata[8] = 0x03;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD1)
				{
					CustomRes4CmdEdata[3] = 0x5c;
				}
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xe7)
			{		
				CustomRes4CmdEdata[0] = ZetVar.pbI2CnSPIRxData[0];
				CustomRes4CmdEdata[1] = ZetVar.pbI2CnSPIRxData[1];

				if(ZetVar.pbI2CnSPIRxData[1] == 0x73)
				{
				  CustomExDataCnt = 21;
					CustomRes4CmdEdata[3] = 0x01;
					CustomRes4CmdEdata[4] = 0x02;
					CustomRes4CmdEdata[5] = 0x02;
					CustomRes4CmdEdata[6] = 0x02;
					CustomRes4CmdEdata[7] = 0x01;
					CustomRes4CmdEdata[8] = 0x01;
					CustomRes4CmdEdata[9] = 0x02;
					CustomRes4CmdEdata[10] = 0x06;
					CustomRes4CmdEdata[11] = 0x07;
					CustomRes4CmdEdata[12] = 0x03;
					CustomRes4CmdEdata[13] = 0x03;
					CustomRes4CmdEdata[14] = 0x01;
					CustomRes4CmdEdata[15] = 0x01;
					CustomRes4CmdEdata[16] = 0x02;
					CustomRes4CmdEdata[17] = 0x08;
					CustomRes4CmdEdata[18] = 0x09;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x7e)
				{
					for(i=16;i<35;i++)
					{
						CustomRes4CmdEdata[i] = 0;
					}	
				  CustomExDataCnt = 49;
					
					CustomRes4CmdEdata[7] = 0x19;
					CustomRes4CmdEdata[8] = 0x04;
			
					CustomRes4CmdEdata[11] = 0xe6; //0xbe;
					CustomRes4CmdEdata[12] = 0x21; //0x13;
					CustomRes4CmdEdata[35] = 0xf6;
					CustomRes4CmdEdata[36] = 0x03;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD0)
				{	
					for(i=16;i<30;i++)
					{
						CustomRes4CmdEdata[i] = 0;
					}		
					CustomExDataCnt = 34;
					CustomRes4CmdEdata[3] = 0x04;
					CustomRes4CmdEdata[4] = 0x01;
					CustomRes4CmdEdata[6] = 0x1c;
					CustomRes4CmdEdata[7] = 0x01;
					CustomRes4CmdEdata[9] = 0x0f;
					CustomRes4CmdEdata[11] = 0x08;
					CustomRes4CmdEdata[12] = 0x1c;
					CustomRes4CmdEdata[13] = 0x04;
					CustomRes4CmdEdata[14] = 0x01;
					CustomRes4CmdEdata[16] = 0x0f;
					CustomRes4CmdEdata[18] = 0x02;
					CustomRes4CmdEdata[19] = 0x20;
					CustomRes4CmdEdata[20] = 0x0c;
					CustomRes4CmdEdata[21] = 0x01;
					CustomRes4CmdEdata[23] = 0x08;
					CustomRes4CmdEdata[25] = 0x03;
					CustomRes4CmdEdata[26] = 0x2c;					
					CustomRes4CmdEdata[27] = 0x01;
					CustomRes4CmdEdata[28] = 0x01;
					CustomRes4CmdEdata[30] = 0x04;
					CustomRes4CmdEdata[31] = 0x0f;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD3)
				{
				  CustomExDataCnt = 17;
					CustomRes4CmdEdata[3] = 0x01;
					CustomRes4CmdEdata[4] = 0x2d;
					CustomRes4CmdEdata[5] = 0x0f;
					CustomRes4CmdEdata[6] = 0x06;
					CustomRes4CmdEdata[7] = 0x70; //0x63; //0x64; 
					CustomRes4CmdEdata[9] = 0x1c;
					CustomRes4CmdEdata[10] = 0x0f;
					CustomRes4CmdEdata[11] = 0x90;
					CustomRes4CmdEdata[12] = 0x07;
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0xD9)
				{
					CustomExDataCnt = 21;
					CustomRes4CmdEdata[3] = 0xb4;
					CustomRes4CmdEdata[4] = 0x1a;
					CustomRes4CmdEdata[7] = 0x79;
					CustomRes4CmdEdata[8] = 0x2f;
					CustomRes4CmdEdata[11] = 0x20;
					CustomRes4CmdEdata[12] = 0xff;
					CustomRes4CmdEdata[13] = 0x14;
					CustomRes4CmdEdata[14] = 0xff;
					CustomRes4CmdEdata[15] = 0xd0;
					CustomRes4CmdEdata[16] = 0x19;
					CustomRes4CmdEdata[17] = 0x78;
					CustomRes4CmdEdata[18] = 0x2e;					
				}
				else if(ZetVar.pbI2CnSPIRxData[1] == 0x72)
				{
					for(i=16;i<37;i++)
					{
						CustomRes4CmdEdata[i] = 0;
					}		
				  CustomExDataCnt = 37;
					CustomRes4CmdEdata[11] = 0xe3; //0xaa;
					CustomRes4CmdEdata[12] = 0x29; //0x1b;
					CustomRes4CmdEdata[19] = 0xe3; //0xaa;
					CustomRes4CmdEdata[20] = 0x29; //0x1b;
				}
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xee) // ??
			{
				ZetVar.bWorkingState = WORKING_STATE_CUSTOMER_INITIAL;
				CustomRes4CmdEdata[0]  = 0xe1;
				CustomRes4CmdEdata[1]  = 0x0e;
				CustomRes4CmdEdata[6]  = CustomCmdList[CustomCmdListCnt][0];         /// current command
				CustomRes4CmdEdata[7]  = CustomCmdList[CustomCmdListCnt][1];
				CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// before of current cmd 
				CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+3)%5][1];
				CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+2)%5][0];   /// after of current cmd  
				CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+2)%5][1];				
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xe1)
			{
				if(CustomerVar.bCmdStatus == 1)
				{
					ZetVar.bWorkingState = WORKING_STATE_CUSTOMER_NORMAL;
					CustomerVar.bCmdStatus = 0;
				}
				CustomRes4CmdEdata[0]  = 0xe1;
				CustomRes4CmdEdata[1]  = 0x0e;
				CustomRes4CmdEdata[6]  = CustomCmdList[CustomCmdListCnt][0];         /// current command
				CustomRes4CmdEdata[7]  = CustomCmdList[CustomCmdListCnt][1];
				CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+4)%5][0];   /// before of current cmd 
				CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+4)%5][1];
				CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// after of current cmd  
				CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+3)%5][1];
			}
		}
		else // (ZetVar.pbI2CnSPIRxData[2] == 0)
		{
			CustomRes4CmdEdata[0]  = 0xe1;
			CustomRes4CmdEdata[1]  = 0x0e;		
			CustomRes4CmdEdata[6]  = CustomCmdList[CustomCmdListCnt][0];              /// current command
			CustomRes4CmdEdata[7]  = CustomCmdList[CustomCmdListCnt][1];
			if((ZetVar.pbI2CnSPIRxData[0] == 0xe4) || (ZetVar.pbI2CnSPIRxData[0] == 0xe5) || (ZetVar.pbI2CnSPIRxData[0] == 0xe6))
			{
				CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+4)%5][0];   /// before of current cmd 
				CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+4)%5][1];
				CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// after of current cmd  
				CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+3)%5][1];	
				if(ZetVar.pbI2CnSPIRxData[0] == 0xe6)
				{
					CustomRes4CmdEdata[8]  = 0x11;
				}

				if((ZetVar.bWorkingState == WORKING_STATE_CUSTOMER_INITIAL) && (ZetVar.pbI2CnSPIRxData[0] == 0xe4) && (ZetVar.pbI2CnSPIRxData[1] == 0xaf))
				{
					CustomerVar.bCmdStatus = 1;
				}
#ifdef FEATURE_TX_RX_IR			
				else if((ZetVar.pbI2CnSPIRxData[0] == 0xe4) && (ZetVar.pbI2CnSPIRxData[1] == 0xbf) && (ZetVar.pbI2CnSPIRxData[3] == 0xdd))
				{
					// IR on
					CustomRes4CmdEdata[1]  = 0x38;	
					CustomerVar.bIREnable = 1;
					CustomerVar.bIRFirstDetect = 1;
					CustomerVar.bIRKeepStatus = 0;
				}
				else if((ZetVar.pbI2CnSPIRxData[0] == 0xe4) && (ZetVar.pbI2CnSPIRxData[1] == 0xbf) && (ZetVar.pbI2CnSPIRxData[3] == 0x9d))
				{
					// IR off
					CustomRes4CmdEdata[1]  = 0x38;
					CustomerVar.bIREnable = 0;
					CustomerVar.bIRFirstDetect = 0;
				}
#endif
			}
			else if(ZetVar.pbI2CnSPIRxData[0] == 0xeb) // eb ?? 01
			{
				CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];	 /// before of current cmd 
				CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+3)%5][1];
				CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+1)%5][0];	 /// after of current cmd  
				CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+1)%5][1];	
			}
			else
			{
				CustomRes4CmdEdata[3]  = CustomCmdList[(CustomCmdListCnt+3)%5][0];   /// before of current cmd 
				CustomRes4CmdEdata[4]  = CustomCmdList[(CustomCmdListCnt+3)%5][1];
				CustomRes4CmdEdata[9]  = CustomCmdList[(CustomCmdListCnt+2)%5][0];   /// after of current cmd  
				CustomRes4CmdEdata[10] = CustomCmdList[(CustomCmdListCnt+2)%5][1];
			}
		}

		bCustomExCksumCnt = CustomExDataCnt-2;
		for(i=0;i<bCustomExCksumCnt;i++)
		{
			wCustomChecksum += CustomRes4CmdEdata[i];
		}
		CustomRes4CmdEdata[bCustomExCksumCnt] = LOBYTE(wCustomChecksum);
		CustomRes4CmdEdata[bCustomExCksumCnt+1] = HIBYTE(wCustomChecksum);
		if(ZetVar.pbI2CnSPIRxData[0] != 0xe5)
		{
			CustomerResponseArray((BYTE *)CustomRes4CmdEdata,CustomExDataCnt,0);
		}
		else
		{
			CustomerResponseArray((BYTE *)CustomRes4CmdEdata,37,0);
		}
	}	
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x19)
	{
		CustomerResponseArray((BYTE *)CustomRes4Cmd19_CODE,11, 0);
	}
	else if((ZetVar.pbI2CnSPIRxData[0] == 0x1a)&&(ZetVar.pbI2CnSPIRxData[1] == 0xa1))
	{	
		CustomerVar.bCustom1ACnt++;
		if(ZetVar.bWorkingState == WORKING_STATE_CUSTOMER_NORMAL)
		{
			ZetVar.bWorkingState = WORKING_STATE_CUSTOMER_INITIAL;
		}
		if(CustomerVar.bCustom1ACnt == 1)
		{
			CustomerResponseArray((BYTE *)CustomRes4Cmd1A_CODE,16, 0);
		}
		if(CustomerVar.bCustom1ACnt == 2)
		{
			CustomerResponseArray((BYTE *)CustomRes4Cmd1A_2,2,0);
		}
		else if(CustomerVar.bCustom1ACnt == 3)
		{
			CustomerResponseArray((BYTE *)CustomRes4Cmd19_CODE,11, 0);
		}
		else if(CustomerVar.bCustom1ACnt == 4)
		{			
			CustomerResponseArray((BYTE *)CustomRes4Cmd1A_CODE,16, 0);			
		}
		else if(CustomerVar.bCustom1ACnt == 5)
		{
			CustomerVar.bCustom1ACnt = 4;
			BYTE bTempbuff[] = {0x48,0x79,0x48};			
			CustomerResponseArray(bTempbuff,3,0);  
		}
	}
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x18)
	{		
		BYTE bTempbuff[] = {0x4b,0xc1};
		CustomerResponseArray(bTempbuff,2,0);  
	}
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x1c)
	{		
		if(ZetVar.pbI2CnSPIRxData[2] == 0x8f)
		{
			BYTE bTempbuff[] = {0x4C,0x39,0x15,0xC1,0x43,0x4D,0x01,0x66};
			CustomerResponseArray((BYTE *)bTempbuff,8,0);
		}
		else
		{
			BYTE bTempbuff[] = {0x4c,0x39,0,0,0,0,0,0};
			CustomerResponseArray((BYTE *)bTempbuff,8,0);
		}
	}
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x1d)
	{	
		BYTE a,b;
		for(a=0;a<5;a++)
		{
			for(b=0;b<2;b++)
			{				
				CustomCmdList[a][b] = 0;
			}
		}

		for(i=0;i<50;i++)
		{
			CustomRes4CmdEdata[i] = 0;
		}	
		CustomerResponseArray((BYTE *)CustomRes4Cmd1d,16,0);
		CustomerVar.bTimeToReport = 0;
	}
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x1e)
	{		
		BYTE bTempbuff[] = {0x4a,0xd1};			
		CustomerResponseArray(bTempbuff,2,0);  
	}
	else if(ZetVar.pbI2CnSPIRxData[0] == 0x1f)
	{		
		BYTE bTempbuff[] = {0x49,0x69};			
		CustomerResponseArray(bTempbuff,2,0);  
	}

	else if(ZetVar.pbI2CnSPIRxData[0] == 0xff)
	{
	}
	ZetVar.pbI2CnSPIRxData[0] = 0xff;	 	 
	
	/// history command fifo count increase
	CustomCmdListCnt = (CustomCmdListCnt+1)%5;
#else

#endif
}

#endif //FEATURE_CUSTOMER_PROTOCOL
#endif


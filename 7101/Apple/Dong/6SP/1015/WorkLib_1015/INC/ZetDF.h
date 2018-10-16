///-------------------------------------------------------------------------------///
///  Data flash SubClass #0 => RomProjectData
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRomProjectDataSt
{
	///  1. Abbrev: PCODE1; Default: 0x11(-); DF Addr: 0000h; SRAM Offset: 0000h ; SubClass Offset : 0
	///     Project Code 1
	U1 bProjectCode1;                                          
	///  2. Abbrev: PCODE2; Default: 0x01(-); DF Addr: 0001h; SRAM Offset: 0001h ; SubClass Offset : 1
	///     Project Code 2 :  : Primary version
	U1 bProjectCode2;                                          
	///  3. Abbrev: PCODE3; Default: 0x02(-); DF Addr: 0002h; SRAM Offset: 0002h ; SubClass Offset : 2
	///     Project Code 3 : Sub version
	U1 bProjectCode3;                                          
	///  4. Abbrev: PCODE4; Default: 0x08(-); DF Addr: 0003h; SRAM Offset: 0003h ; SubClass Offset : 3
	///     Project Code 4
	U1 bProjectCode4;                                          
	///  5. Abbrev: PCODE5; Default: 0x11(-); DF Addr: 0004h; SRAM Offset: 0004h ; SubClass Offset : 4
	///     Project Code 5
	U1 bProjectCode5;                                          
	///  6. Abbrev: PCODE6; Default: 0x05(-); DF Addr: 0005h; SRAM Offset: 0005h ; SubClass Offset : 5
	///     Project Code 6
	U1 bProjectCode6;                                          
	///  7. Abbrev: PCODE7; Default: 0x04(-); DF Addr: 0006h; SRAM Offset: 0006h ; SubClass Offset : 6
	///     Project Code 7
	U1 bProjectCode7;                                          
	///  8. Abbrev: PCODE8; Default: 0x0C(-); DF Addr: 0007h; SRAM Offset: 0007h ; SubClass Offset : 7
	///     Project Code 8
	U1 bProjectCode8;                                          
} SubClassRomProjectDataType;	///< Total = 8 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #1 => PanelInformation
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassPanelInformationSt
{
	///  1. Abbrev: SENSE_AXIS_MAX; Default: 0x0C(-); DF Addr: 0008h; SRAM Offset: 0008h ; SubClass Offset : 0
	///     Max sense axis number
	U1 bSenseAxisMax;                                          
	///  2. Abbrev: DRIVE_AXIS_MAX; Default: 0x14(-); DF Addr: 0009h; SRAM Offset: 0009h ; SubClass Offset : 1
	///     Max sense axis number
	U1 bDriveAxisMax;                                          
	///  3. Abbrev: IC_PACKAGE_TYPE; Default: 0x01(-); DF Addr: 000Ah; SRAM Offset: 000Ah ; SubClass Offset : 2
	///     [FIX] IC Package Type , don't remove or move the item!!!
	U1 bICPackageType;                                         
	///  4. Abbrev: SEN_AXIS_INTERP; Default: 0x01E0(-); DF Addr: 000Bh; SRAM Offset: 000Bh ; SubClass Offset : 3
	///     [!!!] Sense Axis Interpolation
	U2 wSenAxisInterp;                                         
	///  5. Abbrev: INIT_SCAN_EN; Default: 0x00(-); DF Addr: 000Dh; SRAM Offset: 000Dh ; SubClass Offset : 5
	///     [FIX]Init Scan Enable
	U1 bEnableInitScan;                                        
	///  6. Abbrev: SEN_REPORT_SHIFT; Default: 0x0000(-); DF Addr: 000Eh; SRAM Offset: 000Eh ; SubClass Offset : 6
	///     [!!!]Sense Axis Report Offset Shift value
	U2 iSenAxisReportShift;                                    
	///  7. Abbrev: DRI_REPORT_SHIFT; Default: 0x0000(-); DF Addr: 0010h; SRAM Offset: 0010h ; SubClass Offset : 8
	///     [!!!]Drive Axis Report Offset Shift value
	U2 iDriAxisReportShift;                                    
	///  8. Abbrev: SEN_REPORT_GAIN; Default: 0x0000(-); DF Addr: 0012h; SRAM Offset: 0012h ; SubClass Offset : 10
	///     [!!!]Sense Axis Report Gain Shift value
	U2 iSenAxisReportGain;                                     
	///  9. Abbrev: DRI_REPORT_GAIN; Default: 0x0000(-); DF Addr: 0014h; SRAM Offset: 0014h ; SubClass Offset : 12
	///     [!!!]Drive Axis Report Gain Shift value
	U2 iDriAxisReportGain;                                     
	/// 10. Abbrev: DRI_AXIS_INTERP; Default: 0x01C9(-); DF Addr: 0016h; SRAM Offset: 0016h ; SubClass Offset : 14
	///     [!!!] Drive Axis Interpolation
	U2 wDriAxisInterp;                                         
	/// 11. Abbrev: SEN_AXIS_RES; Default: 0x02D0(-); DF Addr: 0018h; SRAM Offset: 0018h ; SubClass Offset : 16
	///     Sense Axis Resolution
	U2 wSenAxisRes;                                            
	/// 12. Abbrev: DRI_AXIS_RES; Default: 0x0500(-); DF Addr: 001Ah; SRAM Offset: 001Ah ; SubClass Offset : 18
	///     Drive Axis Resolution
	U2 wDriAxisRes;                                            
	/// 13. Abbrev: LCM_SEN_AXIS_RES; Default: 0x0300(-); DF Addr: 001Ch; SRAM Offset: 001Ch ; SubClass Offset : 20
	///     LCM Sense Axis Resolution
	U2 wLcmSenAxisRes;                                         
	/// 14. Abbrev: LCM_DRI_AXIS_RES; Default: 0x0400(-); DF Addr: 001Eh; SRAM Offset: 001Eh ; SubClass Offset : 22
	///     LCM Drive Axis Resolution
	U2 wLcmDriAxisRes;                                         
} SubClassPanelInformationType;	///< Total = 24 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #2 => ForceTouchInformation
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassForceTouchInformationSt
{
	///  1. Abbrev: FORCE_SENSE_AXIS_MAX; Default: 0x00(-); DF Addr: 0020h; SRAM Offset: 0020h ; SubClass Offset : 0
	///     Max sense axis number
	U1 bFSenseAxisMax;                                         
	///  2. Abbrev: FORCE_DRIVE_AXIS_MAX; Default: 0x00(-); DF Addr: 0021h; SRAM Offset: 0021h ; SubClass Offset : 1
	///     Max sense axis number
	U1 bFDriveAxisMax;                                         
} SubClassForceTouchInformationType;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #0 => GlobalData
/// =============================================================///
typedef struct PACK ClassGlobalDataSt
{
	SubClassRomProjectDataType scRomProjectData;
	SubClassPanelInformationType scPanelInformation;
	SubClassForceTouchInformationType scForceTouchInformation;
}ClassGlobalDataType;	///< Total = 34 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #3 => InitDevRatio
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassInitDevRatioSt
{
	///  1. Abbrev: INIT_DEV_RATIO_EN; Default: 0x00(); DF Addr: 0022h; SRAM Offset: 0022h ; SubClass Offset : 0
	///     
	U1 bInitialDevRatioEn;                                     
} SubClassInitDevRatioType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #4 => InitSenseAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassInitSenseAxisSt
{
	///  1. Abbrev: INIT_SEN_RATIO_00; Default: 0x40(); DF Addr: 0023h; SRAM Offset: 0023h ; SubClass Offset : 0
	///     Dev=Dev*N/64
	U1 bSenDevRatio00;                                         
	///  2. Abbrev: INIT_SEN_RATIO_01; Default: 0x40(); DF Addr: 0024h; SRAM Offset: 0024h ; SubClass Offset : 1
	///     Dev=Dev*N/64
	U1 bSenDevRatio01;                                         
	///  3. Abbrev: INIT_SEN_RATIO_02; Default: 0x40(); DF Addr: 0025h; SRAM Offset: 0025h ; SubClass Offset : 2
	///     Dev=Dev*N/64
	U1 bSenDevRatio02;                                         
	///  4. Abbrev: INIT_SEN_RATIO_03; Default: 0x40(); DF Addr: 0026h; SRAM Offset: 0026h ; SubClass Offset : 3
	///     Dev=Dev*N/64
	U1 bSenDevRatio03;                                         
	///  5. Abbrev: INIT_SEN_RATIO_04; Default: 0x40(); DF Addr: 0027h; SRAM Offset: 0027h ; SubClass Offset : 4
	///     Dev=Dev*N/64
	U1 bSenDevRatio04;                                         
	///  6. Abbrev: INIT_SEN_RATIO_05; Default: 0x40(); DF Addr: 0028h; SRAM Offset: 0028h ; SubClass Offset : 5
	///     Dev=Dev*N/64
	U1 bSenDevRatio05;                                         
	///  7. Abbrev: INIT_SEN_RATIO_06; Default: 0x40(); DF Addr: 0029h; SRAM Offset: 0029h ; SubClass Offset : 6
	///     Dev=Dev*N/64
	U1 bSenDevRatio06;                                         
	///  8. Abbrev: INIT_SEN_RATIO_07; Default: 0x40(); DF Addr: 002Ah; SRAM Offset: 002Ah ; SubClass Offset : 7
	///     Dev=Dev*N/64
	U1 bSenDevRatio07;                                         
	///  9. Abbrev: INIT_SEN_RATIO_08; Default: 0x40(); DF Addr: 002Bh; SRAM Offset: 002Bh ; SubClass Offset : 8
	///     Dev=Dev*N/64
	U1 bSenDevRatio08;                                         
	/// 10. Abbrev: INIT_SEN_RATIO_09; Default: 0x40(); DF Addr: 002Ch; SRAM Offset: 002Ch ; SubClass Offset : 9
	///     Dev=Dev*N/64
	U1 bSenDevRatio09;                                         
	/// 11. Abbrev: INIT_SEN_RATIO_10; Default: 0x40(); DF Addr: 002Dh; SRAM Offset: 002Dh ; SubClass Offset : 10
	///     Dev=Dev*N/64
	U1 bSenDevRatio10;                                         
	/// 12. Abbrev: INIT_SEN_RATIO_11; Default: 0x40(); DF Addr: 002Eh; SRAM Offset: 002Eh ; SubClass Offset : 11
	///     Dev=Dev*N/64
	U1 bSenDevRatio11;                                         
	/// 13. Abbrev: INIT_SEN_RATIO_12; Default: 0x40(); DF Addr: 002Fh; SRAM Offset: 002Fh ; SubClass Offset : 12
	///     Dev=Dev*N/64
	U1 bSenDevRatio12;                                         
	/// 14. Abbrev: INIT_SEN_RATIO_13; Default: 0x40(); DF Addr: 0030h; SRAM Offset: 0030h ; SubClass Offset : 13
	///     Dev=Dev*N/64
	U1 bSenDevRatio13;                                         
	/// 15. Abbrev: INIT_SEN_RATIO_14; Default: 0x40(); DF Addr: 0031h; SRAM Offset: 0031h ; SubClass Offset : 14
	///     Dev=Dev*N/64
	U1 bSenDevRatio14;                                         
	/// 16. Abbrev: INIT_SEN_RATIO_15; Default: 0x40(); DF Addr: 0032h; SRAM Offset: 0032h ; SubClass Offset : 15
	///     Dev=Dev*N/64
	U1 bSenDevRatio15;                                         
	/// 17. Abbrev: INIT_SEN_RATIO_16; Default: 0x40(); DF Addr: 0033h; SRAM Offset: 0033h ; SubClass Offset : 16
	///     Dev=Dev*N/64
	U1 bSenDevRatio16;                                         
	/// 18. Abbrev: INIT_SEN_RATIO_17; Default: 0x40(); DF Addr: 0034h; SRAM Offset: 0034h ; SubClass Offset : 17
	///     Dev=Dev*N/64
	U1 bSenDevRatio17;                                         
	/// 19. Abbrev: INIT_SEN_RATIO_18; Default: 0x40(); DF Addr: 0035h; SRAM Offset: 0035h ; SubClass Offset : 18
	///     Dev=Dev*N/64
	U1 bSenDevRatio18;                                         
	/// 20. Abbrev: INIT_SEN_RATIO_19; Default: 0x40(); DF Addr: 0036h; SRAM Offset: 0036h ; SubClass Offset : 19
	///     Dev=Dev*N/64
	U1 bSenDevRatio19;                                         
	/// 21. Abbrev: INIT_SEN_RATIO_20; Default: 0x40(); DF Addr: 0037h; SRAM Offset: 0037h ; SubClass Offset : 20
	///     Dev=Dev*N/64
	U1 bSenDevRatio20;                                         
	/// 22. Abbrev: INIT_SEN_RATIO_21; Default: 0x40(); DF Addr: 0038h; SRAM Offset: 0038h ; SubClass Offset : 21
	///     Dev=Dev*N/64
	U1 bSenDevRatio21;                                         
	/// 23. Abbrev: INIT_SEN_RATIO_22; Default: 0x40(); DF Addr: 0039h; SRAM Offset: 0039h ; SubClass Offset : 22
	///     Dev=Dev*N/64
	U1 bSenDevRatio22;                                         
	/// 24. Abbrev: INIT_SEN_RATIO_23; Default: 0x40(); DF Addr: 003Ah; SRAM Offset: 003Ah ; SubClass Offset : 23
	///     Dev=Dev*N/64
	U1 bSenDevRatio23;                                         
	/// 25. Abbrev: INIT_SEN_RATIO_24; Default: 0x40(); DF Addr: 003Bh; SRAM Offset: 003Bh ; SubClass Offset : 24
	///     Dev=Dev*N/64
	U1 bSenDevRatio24;                                         
	/// 26. Abbrev: INIT_SEN_RATIO_25; Default: 0x40(); DF Addr: 003Ch; SRAM Offset: 003Ch ; SubClass Offset : 25
	///     Dev=Dev*N/64
	U1 bSenDevRatio25;                                         
	/// 27. Abbrev: INIT_SEN_RATIO_26; Default: 0x40(); DF Addr: 003Dh; SRAM Offset: 003Dh ; SubClass Offset : 26
	///     Dev=Dev*N/64
	U1 bSenDevRatio26;                                         
	/// 28. Abbrev: INIT_SEN_RATIO_27; Default: 0x40(); DF Addr: 003Eh; SRAM Offset: 003Eh ; SubClass Offset : 27
	///     Dev=Dev*N/64
	U1 bSenDevRatio27;                                         
	/// 29. Abbrev: INIT_SEN_RATIO_28; Default: 0x40(); DF Addr: 003Fh; SRAM Offset: 003Fh ; SubClass Offset : 28
	///     Dev=Dev*N/64
	U1 bSenDevRatio28;                                         
	/// 30. Abbrev: INIT_SEN_RATIO_29; Default: 0x40(); DF Addr: 0040h; SRAM Offset: 0040h ; SubClass Offset : 29
	///     Dev=Dev*N/64
	U1 bSenDevRatio29;                                         
	/// 31. Abbrev: INIT_SEN_RATIO_30; Default: 0x40(); DF Addr: 0041h; SRAM Offset: 0041h ; SubClass Offset : 30
	///     Dev=Dev*N/64
	U1 bSenDevRatio30;                                         
	/// 32. Abbrev: INIT_SEN_RATIO_31; Default: 0x40(); DF Addr: 0042h; SRAM Offset: 0042h ; SubClass Offset : 31
	///     Dev=Dev*N/64
	U1 bSenDevRatio31;                                         
} SubClassInitSenseAxisType;	///< Total = 32 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #5 => InitDriveAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassInitDriveAxisSt
{
	///  1. Abbrev: INIT_DRI_RATIO_00; Default: 0x40(); DF Addr: 0043h; SRAM Offset: 0043h ; SubClass Offset : 0
	///     Dev=Dev*N/64
	U1 bDriDevRatio00;                                         
	///  2. Abbrev: INIT_DRI_RATIO_01; Default: 0x40(); DF Addr: 0044h; SRAM Offset: 0044h ; SubClass Offset : 1
	///     Dev=Dev*N/64
	U1 bDriDevRatio01;                                         
	///  3. Abbrev: INIT_DRI_RATIO_02; Default: 0x40(); DF Addr: 0045h; SRAM Offset: 0045h ; SubClass Offset : 2
	///     Dev=Dev*N/64
	U1 bDriDevRatio02;                                         
	///  4. Abbrev: INIT_DRI_RATIO_03; Default: 0x40(); DF Addr: 0046h; SRAM Offset: 0046h ; SubClass Offset : 3
	///     Dev=Dev*N/64
	U1 bDriDevRatio03;                                         
	///  5. Abbrev: INIT_DRI_RATIO_04; Default: 0x40(); DF Addr: 0047h; SRAM Offset: 0047h ; SubClass Offset : 4
	///     Dev=Dev*N/64
	U1 bDriDevRatio04;                                         
	///  6. Abbrev: INIT_DRI_RATIO_05; Default: 0x40(); DF Addr: 0048h; SRAM Offset: 0048h ; SubClass Offset : 5
	///     Dev=Dev*N/64
	U1 bDriDevRatio05;                                         
	///  7. Abbrev: INIT_DRI_RATIO_06; Default: 0x40(); DF Addr: 0049h; SRAM Offset: 0049h ; SubClass Offset : 6
	///     Dev=Dev*N/64
	U1 bDriDevRatio06;                                         
	///  8. Abbrev: INIT_DRI_RATIO_07; Default: 0x40(); DF Addr: 004Ah; SRAM Offset: 004Ah ; SubClass Offset : 7
	///     Dev=Dev*N/64
	U1 bDriDevRatio07;                                         
	///  9. Abbrev: INIT_DRI_RATIO_08; Default: 0x40(); DF Addr: 004Bh; SRAM Offset: 004Bh ; SubClass Offset : 8
	///     Dev=Dev*N/64
	U1 bDriDevRatio08;                                         
	/// 10. Abbrev: INIT_DRI_RATIO_09; Default: 0x40(); DF Addr: 004Ch; SRAM Offset: 004Ch ; SubClass Offset : 9
	///     Dev=Dev*N/64
	U1 bDriDevRatio09;                                         
	/// 11. Abbrev: INIT_DRI_RATIO_10; Default: 0x40(); DF Addr: 004Dh; SRAM Offset: 004Dh ; SubClass Offset : 10
	///     Dev=Dev*N/64
	U1 bDriDevRatio10;                                         
	/// 12. Abbrev: INIT_DRI_RATIO_11; Default: 0x40(); DF Addr: 004Eh; SRAM Offset: 004Eh ; SubClass Offset : 11
	///     Dev=Dev*N/64
	U1 bDriDevRatio11;                                         
	/// 13. Abbrev: INIT_DRI_RATIO_12; Default: 0x40(); DF Addr: 004Fh; SRAM Offset: 004Fh ; SubClass Offset : 12
	///     Dev=Dev*N/64
	U1 bDriDevRatio12;                                         
	/// 14. Abbrev: INIT_DRI_RATIO_13; Default: 0x40(); DF Addr: 0050h; SRAM Offset: 0050h ; SubClass Offset : 13
	///     Dev=Dev*N/64
	U1 bDriDevRatio13;                                         
	/// 15. Abbrev: INIT_DRI_RATIO_14; Default: 0x40(); DF Addr: 0051h; SRAM Offset: 0051h ; SubClass Offset : 14
	///     Dev=Dev*N/64
	U1 bDriDevRatio14;                                         
	/// 16. Abbrev: INIT_DRI_RATIO_15; Default: 0x40(); DF Addr: 0052h; SRAM Offset: 0052h ; SubClass Offset : 15
	///     Dev=Dev*N/64
	U1 bDriDevRatio15;                                         
	/// 17. Abbrev: INIT_DRI_RATIO_16; Default: 0x40(); DF Addr: 0053h; SRAM Offset: 0053h ; SubClass Offset : 16
	///     Dev=Dev*N/64
	U1 bDriDevRatio16;                                         
	/// 18. Abbrev: INIT_DRI_RATIO_17; Default: 0x40(); DF Addr: 0054h; SRAM Offset: 0054h ; SubClass Offset : 17
	///     Dev=Dev*N/64
	U1 bDriDevRatio17;                                         
	/// 19. Abbrev: INIT_DRI_RATIO_18; Default: 0x40(); DF Addr: 0055h; SRAM Offset: 0055h ; SubClass Offset : 18
	///     Dev=Dev*N/64
	U1 bDriDevRatio18;                                         
	/// 20. Abbrev: INIT_DRI_RATIO_19; Default: 0x40(); DF Addr: 0056h; SRAM Offset: 0056h ; SubClass Offset : 19
	///     Dev=Dev*N/64
	U1 bDriDevRatio19;                                         
	/// 21. Abbrev: INIT_DRI_RATIO_20; Default: 0x40(); DF Addr: 0057h; SRAM Offset: 0057h ; SubClass Offset : 20
	///     Dev=Dev*N/64
	U1 bDriDevRatio20;                                         
	/// 22. Abbrev: INIT_DRI_RATIO_21; Default: 0x40(); DF Addr: 0058h; SRAM Offset: 0058h ; SubClass Offset : 21
	///     Dev=Dev*N/64
	U1 bDriDevRatio21;                                         
	/// 23. Abbrev: INIT_DRI_RATIO_22; Default: 0x40(); DF Addr: 0059h; SRAM Offset: 0059h ; SubClass Offset : 22
	///     Dev=Dev*N/64
	U1 bDriDevRatio22;                                         
	/// 24. Abbrev: INIT_DRI_RATIO_23; Default: 0x40(); DF Addr: 005Ah; SRAM Offset: 005Ah ; SubClass Offset : 23
	///     Dev=Dev*N/64
	U1 bDriDevRatio23;                                         
	/// 25. Abbrev: INIT_DRI_RATIO_24; Default: 0x40(); DF Addr: 005Bh; SRAM Offset: 005Bh ; SubClass Offset : 24
	///     Dev=Dev*N/64
	U1 bDriDevRatio24;                                         
	/// 26. Abbrev: INIT_DRI_RATIO_25; Default: 0x40(); DF Addr: 005Ch; SRAM Offset: 005Ch ; SubClass Offset : 25
	///     Dev=Dev*N/64
	U1 bDriDevRatio25;                                         
	/// 27. Abbrev: INIT_DRI_RATIO_26; Default: 0x40(); DF Addr: 005Dh; SRAM Offset: 005Dh ; SubClass Offset : 26
	///     Dev=Dev*N/64
	U1 bDriDevRatio26;                                         
	/// 28. Abbrev: INIT_DRI_RATIO_27; Default: 0x40(); DF Addr: 005Eh; SRAM Offset: 005Eh ; SubClass Offset : 27
	///     Dev=Dev*N/64
	U1 bDriDevRatio27;                                         
	/// 29. Abbrev: INIT_DRI_RATIO_28; Default: 0x40(); DF Addr: 005Fh; SRAM Offset: 005Fh ; SubClass Offset : 28
	///     Dev=Dev*N/64
	U1 bDriDevRatio28;                                         
	/// 30. Abbrev: INIT_DRI_RATIO_29; Default: 0x40(); DF Addr: 0060h; SRAM Offset: 0060h ; SubClass Offset : 29
	///     Dev=Dev*N/64
	U1 bDriDevRatio29;                                         
	/// 31. Abbrev: INIT_DRI_RATIO_30; Default: 0x40(); DF Addr: 0061h; SRAM Offset: 0061h ; SubClass Offset : 30
	///     Dev=Dev*N/64
	U1 bDriDevRatio30;                                         
	/// 32. Abbrev: INIT_DRI_RATIO_31; Default: 0x40(); DF Addr: 0062h; SRAM Offset: 0062h ; SubClass Offset : 31
	///     Dev=Dev*N/64
	U1 bDriDevRatio31;                                         
} SubClassInitDriveAxisType;	///< Total = 32 Bytes

/// =============================================================///
/// Data flash Class #1 => InitialDev
/// =============================================================///
typedef struct PACK ClassInitialDevSt
{
	SubClassInitDevRatioType scInitDevRatio;
	SubClassInitSenseAxisType scInitSenseAxis;
	SubClassInitDriveAxisType scInitDriveAxis;
}ClassInitialDevType;	///< Total = 65 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #6 => DevData
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDevDataSt
{
	///  1. Abbrev: DEVICE_NAME; Default: UNKNOWN(-); DF Addr: 0080h; SRAM Offset: 0063h ; SubClass Offset : 0
	///     [FIX] Device name[12] for Software, don't remove or move the item!!!
	BYTE pDeviceName[12];                                      
	///  2. Abbrev: DATA_FLASH_VERSION; Default: 63(-); DF Addr: 008Ch; SRAM Offset: 006Fh ; SubClass Offset : 12
	///     [FIX] Data Flash Version for Software, don't remove or move the item!!!
	U2 wDataFlashVersion;                                      
	///  3. Abbrev: DATA_FLASH_SUB_VERSION; Default: 86(-); DF Addr: 008Eh; SRAM Offset: 0071h ; SubClass Offset : 14
	///     [FIX] Data Flash Version for Software, don't remove or move the item!!!
	U2 wDataFlashSubVersion;                                   
} SubClassDevDataType;	///< Total = 16 Bytes

/// =============================================================///
/// Data flash Class #2 => DevData
/// =============================================================///
typedef struct PACK ClassDevDataSt
{
	SubClassDevDataType scDevData;
}ClassDevDataType;	///< Total = 16 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #7 => PanelSenseAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassPanelSenseAxisSt
{
	///  1. Abbrev: SEN00; Default: 0x84(-); DF Addr: 0090h; SRAM Offset: 0073h ; SubClass Offset : 0
	///     Sense Trace #0
	U1 bSen00;                                                 
	///  2. Abbrev: SEN01; Default: 0x85(-); DF Addr: 0091h; SRAM Offset: 0074h ; SubClass Offset : 1
	///     Sense Trace #1
	U1 bSen01;                                                 
	///  3. Abbrev: SEN02; Default: 0x86(-); DF Addr: 0092h; SRAM Offset: 0075h ; SubClass Offset : 2
	///     Sense Trace #2
	U1 bSen02;                                                 
	///  4. Abbrev: SEN03; Default: 0x87(-); DF Addr: 0093h; SRAM Offset: 0076h ; SubClass Offset : 3
	///     Sense Trace #3
	U1 bSen03;                                                 
	///  5. Abbrev: SEN04; Default: 0x88(-); DF Addr: 0094h; SRAM Offset: 0077h ; SubClass Offset : 4
	///     Sense Trace #4
	U1 bSen04;                                                 
	///  6. Abbrev: SEN05; Default: 0x89(-); DF Addr: 0095h; SRAM Offset: 0078h ; SubClass Offset : 5
	///     Sense Trace #5
	U1 bSen05;                                                 
	///  7. Abbrev: SEN06; Default: 0x8A(-); DF Addr: 0096h; SRAM Offset: 0079h ; SubClass Offset : 6
	///     Sense Trace #6
	U1 bSen06;                                                 
	///  8. Abbrev: SEN07; Default: 0x8B(-); DF Addr: 0097h; SRAM Offset: 007Ah ; SubClass Offset : 7
	///     Sense Trace #7
	U1 bSen07;                                                 
	///  9. Abbrev: SEN08; Default: 0x8C(-); DF Addr: 0098h; SRAM Offset: 007Bh ; SubClass Offset : 8
	///     Sense Trace #8
	U1 bSen08;                                                 
	/// 10. Abbrev: SEN09; Default: 0x8D(-); DF Addr: 0099h; SRAM Offset: 007Ch ; SubClass Offset : 9
	///     Sense Trace #9
	U1 bSen09;                                                 
	/// 11. Abbrev: SEN10; Default: 0x8E(-); DF Addr: 009Ah; SRAM Offset: 007Dh ; SubClass Offset : 10
	///     Sense Trace #10
	U1 bSen10;                                                 
	/// 12. Abbrev: SEN11; Default: 0x8F(-); DF Addr: 009Bh; SRAM Offset: 007Eh ; SubClass Offset : 11
	///     Sense Trace #11
	U1 bSen11;                                                 
	/// 13. Abbrev: SEN12; Default: 0xFF(-); DF Addr: 009Ch; SRAM Offset: 007Fh ; SubClass Offset : 12
	///     Sense Trace #12
	U1 bSen12;                                                 
	/// 14. Abbrev: SEN13; Default: 0xFF(-); DF Addr: 009Dh; SRAM Offset: 0080h ; SubClass Offset : 13
	///     Sense Trace #13
	U1 bSen13;                                                 
	/// 15. Abbrev: SEN14; Default: 0xFF(-); DF Addr: 009Eh; SRAM Offset: 0081h ; SubClass Offset : 14
	///     Sense Trace #14
	U1 bSen14;                                                 
	/// 16. Abbrev: SEN15; Default: 0xFF(-); DF Addr: 009Fh; SRAM Offset: 0082h ; SubClass Offset : 15
	///     Sense Trace #15
	U1 bSen15;                                                 
	/// 17. Abbrev: SEN16; Default: 0xFF(-); DF Addr: 00A0h; SRAM Offset: 0083h ; SubClass Offset : 16
	///     Sense Trace #16
	U1 bSen16;                                                 
	/// 18. Abbrev: SEN17; Default: 0xFF(-); DF Addr: 00A1h; SRAM Offset: 0084h ; SubClass Offset : 17
	///     Sense Trace #17
	U1 bSen17;                                                 
	/// 19. Abbrev: SEN18; Default: 0xFF(-); DF Addr: 00A2h; SRAM Offset: 0085h ; SubClass Offset : 18
	///     Sense Trace #18
	U1 bSen18;                                                 
	/// 20. Abbrev: SEN19; Default: 0xFF(-); DF Addr: 00A3h; SRAM Offset: 0086h ; SubClass Offset : 19
	///     Sense Trace #19
	U1 bSen19;                                                 
	/// 21. Abbrev: SEN20; Default: 0xFF(-); DF Addr: 00A4h; SRAM Offset: 0087h ; SubClass Offset : 20
	///     Sense Trace #20
	U1 bSen20;                                                 
	/// 22. Abbrev: SEN21; Default: 0xFF(-); DF Addr: 00A5h; SRAM Offset: 0088h ; SubClass Offset : 21
	///     Sense Trace #21
	U1 bSen21;                                                 
	/// 23. Abbrev: SEN22; Default: 0xFF(-); DF Addr: 00A6h; SRAM Offset: 0089h ; SubClass Offset : 22
	///     Sense Trace #22
	U1 bSen22;                                                 
	/// 24. Abbrev: SEN23; Default: 0xFF(-); DF Addr: 00A7h; SRAM Offset: 008Ah ; SubClass Offset : 23
	///     Sense Trace #23
	U1 bSen23;                                                 
	/// 25. Abbrev: SEN24; Default: 0xFF(-); DF Addr: 00A8h; SRAM Offset: 008Bh ; SubClass Offset : 24
	///     Sense Trace #24
	U1 bSen24;                                                 
	/// 26. Abbrev: SEN25; Default: 0xFF(-); DF Addr: 00A9h; SRAM Offset: 008Ch ; SubClass Offset : 25
	///     Sense Trace #25
	U1 bSen25;                                                 
	/// 27. Abbrev: SEN26; Default: 0xFF(-); DF Addr: 00AAh; SRAM Offset: 008Dh ; SubClass Offset : 26
	///     Sense Trace #26
	U1 bSen26;                                                 
	/// 28. Abbrev: SEN27; Default: 0xFF(-); DF Addr: 00ABh; SRAM Offset: 008Eh ; SubClass Offset : 27
	///     Sense Trace #27
	U1 bSen27;                                                 
	/// 29. Abbrev: SEN28; Default: 0xFF(-); DF Addr: 00ACh; SRAM Offset: 008Fh ; SubClass Offset : 28
	///     Sense Trace #28
	U1 bSen28;                                                 
	/// 30. Abbrev: SEN29; Default: 0xFF(-); DF Addr: 00ADh; SRAM Offset: 0090h ; SubClass Offset : 29
	///     Sense Trace #29
	U1 bSen29;                                                 
	/// 31. Abbrev: SEN30; Default: 0xFF(-); DF Addr: 00AEh; SRAM Offset: 0091h ; SubClass Offset : 30
	///     Sense Trace #30
	U1 bSen30;                                                 
	/// 32. Abbrev: SEN31; Default: 0xFF(-); DF Addr: 00AFh; SRAM Offset: 0092h ; SubClass Offset : 31
	///     Sense Trace #31
	U1 bSen31;                                                 
	/// 33. Abbrev: SEN32; Default: 0xFF(-); DF Addr: 00B0h; SRAM Offset: 0093h ; SubClass Offset : 32
	///     Sense Trace #32
	U1 bSen32;                                                 
	/// 34. Abbrev: SEN33; Default: 0xFF(-); DF Addr: 00B1h; SRAM Offset: 0094h ; SubClass Offset : 33
	///     Sense Trace #33
	U1 bSen33;                                                 
	/// 35. Abbrev: SEN34; Default: 0xFF(-); DF Addr: 00B2h; SRAM Offset: 0095h ; SubClass Offset : 34
	///     Sense Trace #34
	U1 bSen34;                                                 
	/// 36. Abbrev: SEN35; Default: 0xFF(-); DF Addr: 00B3h; SRAM Offset: 0096h ; SubClass Offset : 35
	///     Sense Trace #35
	U1 bSen35;                                                 
	/// 37. Abbrev: SEN36; Default: 0xFF(-); DF Addr: 00B4h; SRAM Offset: 0097h ; SubClass Offset : 36
	///     Sense Trace #36
	U1 bSen36;                                                 
	/// 38. Abbrev: SEN37; Default: 0xFF(-); DF Addr: 00B5h; SRAM Offset: 0098h ; SubClass Offset : 37
	///     Sense Trace #37
	U1 bSen37;                                                 
	/// 39. Abbrev: SEN38; Default: 0xFF(-); DF Addr: 00B6h; SRAM Offset: 0099h ; SubClass Offset : 38
	///     Sense Trace #38
	U1 bSen38;                                                 
	/// 40. Abbrev: SEN39; Default: 0xFF(-); DF Addr: 00B7h; SRAM Offset: 009Ah ; SubClass Offset : 39
	///     Sense Trace #39
	U1 bSen39;                                                 
	/// 41. Abbrev: SEN40; Default: 0xFF(-); DF Addr: 00B8h; SRAM Offset: 009Bh ; SubClass Offset : 40
	///     Sense Trace #40
	U1 bSen40;                                                 
	/// 42. Abbrev: SEN41; Default: 0xFF(-); DF Addr: 00B9h; SRAM Offset: 009Ch ; SubClass Offset : 41
	///     Sense Trace #41
	U1 bSen41;                                                 
	/// 43. Abbrev: SEN42; Default: 0xFF(-); DF Addr: 00BAh; SRAM Offset: 009Dh ; SubClass Offset : 42
	///     Sense Trace #42
	U1 bSen42;                                                 
	/// 44. Abbrev: SEN43; Default: 0xFF(-); DF Addr: 00BBh; SRAM Offset: 009Eh ; SubClass Offset : 43
	///     Sense Trace #43
	U1 bSen43;                                                 
	/// 45. Abbrev: SEN44; Default: 0xFF(-); DF Addr: 00BCh; SRAM Offset: 009Fh ; SubClass Offset : 44
	///     Sense Trace #44
	U1 bSen44;                                                 
	/// 46. Abbrev: SEN45; Default: 0xFF(-); DF Addr: 00BDh; SRAM Offset: 00A0h ; SubClass Offset : 45
	///     Sense Trace #45
	U1 bSen45;                                                 
	/// 47. Abbrev: SEN46; Default: 0xFF(-); DF Addr: 00BEh; SRAM Offset: 00A1h ; SubClass Offset : 46
	///     Sense Trace #46
	U1 bSen46;                                                 
	/// 48. Abbrev: SEN47; Default: 0xFF(-); DF Addr: 00BFh; SRAM Offset: 00A2h ; SubClass Offset : 47
	///     Sense Trace #47
	U1 bSen47;                                                 
	/// 49. Abbrev: SEN48; Default: 0xFF(-); DF Addr: 00C0h; SRAM Offset: 00A3h ; SubClass Offset : 48
	///     Sense Trace #48
	U1 bSen48;                                                 
	/// 50. Abbrev: SEN49; Default: 0xFF(-); DF Addr: 00C1h; SRAM Offset: 00A4h ; SubClass Offset : 49
	///     Sense Trace #49
	U1 bSen49;                                                 
	/// 51. Abbrev: SEN50; Default: 0xFF(-); DF Addr: 00C2h; SRAM Offset: 00A5h ; SubClass Offset : 50
	///     Sense Trace #50
	U1 bSen50;                                                 
	/// 52. Abbrev: SEN51; Default: 0xFF(-); DF Addr: 00C3h; SRAM Offset: 00A6h ; SubClass Offset : 51
	///     Sense Trace #51
	U1 bSen51;                                                 
} SubClassPanelSenseAxisType;	///< Total = 52 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #8 => PanelDriveAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassPanelDriveAxisSt
{
	///  1. Abbrev: DRI00; Default: 0x92(-); DF Addr: 00C4h; SRAM Offset: 00A7h ; SubClass Offset : 0
	///     Drive Trace#0
	U1 bDri00;                                                 
	///  2. Abbrev: DRI01; Default: 0x93(-); DF Addr: 00C5h; SRAM Offset: 00A8h ; SubClass Offset : 1
	///     Drive Trace#1
	U1 bDri01;                                                 
	///  3. Abbrev: DRI02; Default: 0x94(-); DF Addr: 00C6h; SRAM Offset: 00A9h ; SubClass Offset : 2
	///     Drive Trace#2
	U1 bDri02;                                                 
	///  4. Abbrev: DRI03; Default: 0x95(-); DF Addr: 00C7h; SRAM Offset: 00AAh ; SubClass Offset : 3
	///     Drive Trace#3
	U1 bDri03;                                                 
	///  5. Abbrev: DRI04; Default: 0x96(-); DF Addr: 00C8h; SRAM Offset: 00ABh ; SubClass Offset : 4
	///     Drive Trace#4
	U1 bDri04;                                                 
	///  6. Abbrev: DRI05; Default: 0x97(-); DF Addr: 00C9h; SRAM Offset: 00ACh ; SubClass Offset : 5
	///     Drive Trace#5
	U1 bDri05;                                                 
	///  7. Abbrev: DRI06; Default: 0x98(-); DF Addr: 00CAh; SRAM Offset: 00ADh ; SubClass Offset : 6
	///     Drive Trace#6
	U1 bDri06;                                                 
	///  8. Abbrev: DRI07; Default: 0x99(-); DF Addr: 00CBh; SRAM Offset: 00AEh ; SubClass Offset : 7
	///     Drive Trace#6
	U1 bDri07;                                                 
	///  9. Abbrev: DRI08; Default: 0x9A(-); DF Addr: 00CCh; SRAM Offset: 00AFh ; SubClass Offset : 8
	///     Drive Trace#8
	U1 bDri08;                                                 
	/// 10. Abbrev: DRI09; Default: 0x9B(-); DF Addr: 00CDh; SRAM Offset: 00B0h ; SubClass Offset : 9
	///     Drive Trace#9
	U1 bDri09;                                                 
	/// 11. Abbrev: DRI10; Default: 0x9C(-); DF Addr: 00CEh; SRAM Offset: 00B1h ; SubClass Offset : 10
	///     Drive Trace#10
	U1 bDri10;                                                 
	/// 12. Abbrev: DRI11; Default: 0x9D(-); DF Addr: 00CFh; SRAM Offset: 00B2h ; SubClass Offset : 11
	///     Drive Trace#11
	U1 bDri11;                                                 
	/// 13. Abbrev: DRI12; Default: 0x9E(-); DF Addr: 00D0h; SRAM Offset: 00B3h ; SubClass Offset : 12
	///     Drive Trace#12
	U1 bDri12;                                                 
	/// 14. Abbrev: DRI13; Default: 0x9F(-); DF Addr: 00D1h; SRAM Offset: 00B4h ; SubClass Offset : 13
	///     Drive Trace#13
	U1 bDri13;                                                 
	/// 15. Abbrev: DRI14; Default: 0xA0(-); DF Addr: 00D2h; SRAM Offset: 00B5h ; SubClass Offset : 14
	///     Drive Trace#14
	U1 bDri14;                                                 
	/// 16. Abbrev: DRI15; Default: 0xA1(-); DF Addr: 00D3h; SRAM Offset: 00B6h ; SubClass Offset : 15
	///     Drive Trace#15
	U1 bDri15;                                                 
	/// 17. Abbrev: DRI16; Default: 0xA2(-); DF Addr: 00D4h; SRAM Offset: 00B7h ; SubClass Offset : 16
	///     Drive Trace#16
	U1 bDri16;                                                 
	/// 18. Abbrev: DRI17; Default: 0xA3(-); DF Addr: 00D5h; SRAM Offset: 00B8h ; SubClass Offset : 17
	///     Drive Trace#17
	U1 bDri17;                                                 
	/// 19. Abbrev: DRI18; Default: 0xA4(-); DF Addr: 00D6h; SRAM Offset: 00B9h ; SubClass Offset : 18
	///     Drive Trace#18
	U1 bDri18;                                                 
	/// 20. Abbrev: DRI19; Default: 0xA5(-); DF Addr: 00D7h; SRAM Offset: 00BAh ; SubClass Offset : 19
	///     Drive Trace#19
	U1 bDri19;                                                 
	/// 21. Abbrev: DRI20; Default: 0xFF(-); DF Addr: 00D8h; SRAM Offset: 00BBh ; SubClass Offset : 20
	///     Drive Trace#20
	U1 bDri20;                                                 
	/// 22. Abbrev: DRI21; Default: 0xFF(-); DF Addr: 00D9h; SRAM Offset: 00BCh ; SubClass Offset : 21
	///     Drive Trace#21
	U1 bDri21;                                                 
	/// 23. Abbrev: DRI22; Default: 0xFF(-); DF Addr: 00DAh; SRAM Offset: 00BDh ; SubClass Offset : 22
	///     Drive Trace#22
	U1 bDri22;                                                 
	/// 24. Abbrev: DRI23; Default: 0xFF(-); DF Addr: 00DBh; SRAM Offset: 00BEh ; SubClass Offset : 23
	///     Drive Trace#23
	U1 bDri23;                                                 
	/// 25. Abbrev: DRI24; Default: 0xFF(-); DF Addr: 00DCh; SRAM Offset: 00BFh ; SubClass Offset : 24
	///     Drive Trace#24
	U1 bDri24;                                                 
	/// 26. Abbrev: DRI25; Default: 0xFF(-); DF Addr: 00DDh; SRAM Offset: 00C0h ; SubClass Offset : 25
	///     Drive Trace#25
	U1 bDri25;                                                 
	/// 27. Abbrev: DRI26; Default: 0xFF(-); DF Addr: 00DEh; SRAM Offset: 00C1h ; SubClass Offset : 26
	///     Drive Trace#26
	U1 bDri26;                                                 
	/// 28. Abbrev: DRI27; Default: 0xFF(-); DF Addr: 00DFh; SRAM Offset: 00C2h ; SubClass Offset : 27
	///     Drive Trace#27
	U1 bDri27;                                                 
	/// 29. Abbrev: DRI28; Default: 0xFF(-); DF Addr: 00E0h; SRAM Offset: 00C3h ; SubClass Offset : 28
	///     Drive Trace#28
	U1 bDri28;                                                 
	/// 30. Abbrev: DRI29; Default: 0xFF(-); DF Addr: 00E1h; SRAM Offset: 00C4h ; SubClass Offset : 29
	///     Drive Trace#29
	U1 bDri29;                                                 
	/// 31. Abbrev: DRI30; Default: 0xFF(-); DF Addr: 00E2h; SRAM Offset: 00C5h ; SubClass Offset : 30
	///     Drive Trace#30
	U1 bDri30;                                                 
	/// 32. Abbrev: DRI31; Default: 0xFF(-); DF Addr: 00E3h; SRAM Offset: 00C6h ; SubClass Offset : 31
	///     Drive Trace#31
	U1 bDri31;                                                 
	/// 33. Abbrev: DRI32; Default: 0xFF(-); DF Addr: 00E4h; SRAM Offset: 00C7h ; SubClass Offset : 32
	///     Drive Trace#32
	U1 bDri32;                                                 
	/// 34. Abbrev: DRI33; Default: 0xFF(-); DF Addr: 00E5h; SRAM Offset: 00C8h ; SubClass Offset : 33
	///     Drive Trace#33
	U1 bDri33;                                                 
	/// 35. Abbrev: DRI34; Default: 0xFF(-); DF Addr: 00E6h; SRAM Offset: 00C9h ; SubClass Offset : 34
	///     Drive Trace#34
	U1 bDri34;                                                 
	/// 36. Abbrev: DRI35; Default: 0xFF(-); DF Addr: 00E7h; SRAM Offset: 00CAh ; SubClass Offset : 35
	///     Drive Trace#35
	U1 bDri35;                                                 
	/// 37. Abbrev: DRI36; Default: 0xFF(-); DF Addr: 00E8h; SRAM Offset: 00CBh ; SubClass Offset : 36
	///     Drive Trace#36
	U1 bDri36;                                                 
	/// 38. Abbrev: DRI37; Default: 0xFF(-); DF Addr: 00E9h; SRAM Offset: 00CCh ; SubClass Offset : 37
	///     Drive Trace#37
	U1 bDri37;                                                 
	/// 39. Abbrev: DRI38; Default: 0xFF(-); DF Addr: 00EAh; SRAM Offset: 00CDh ; SubClass Offset : 38
	///     Drive Trace#38
	U1 bDri38;                                                 
	/// 40. Abbrev: DRI39; Default: 0xFF(-); DF Addr: 00EBh; SRAM Offset: 00CEh ; SubClass Offset : 39
	///     Drive Trace#39
	U1 bDri39;                                                 
	/// 41. Abbrev: DRI40; Default: 0xFF(-); DF Addr: 00ECh; SRAM Offset: 00CFh ; SubClass Offset : 40
	///     Drive Trace#40
	U1 bDri40;                                                 
	/// 42. Abbrev: DRI41; Default: 0xFF(-); DF Addr: 00EDh; SRAM Offset: 00D0h ; SubClass Offset : 41
	///     Drive Trace#41
	U1 bDri41;                                                 
	/// 43. Abbrev: DRI42; Default: 0xFF(-); DF Addr: 00EEh; SRAM Offset: 00D1h ; SubClass Offset : 42
	///     Drive Trace#42
	U1 bDri42;                                                 
	/// 44. Abbrev: DRI43; Default: 0xFF(-); DF Addr: 00EFh; SRAM Offset: 00D2h ; SubClass Offset : 43
	///     Drive Trace#43
	U1 bDri43;                                                 
	/// 45. Abbrev: DRI44; Default: 0xFF(-); DF Addr: 00F0h; SRAM Offset: 00D3h ; SubClass Offset : 44
	///     Drive Trace#44
	U1 bDri44;                                                 
	/// 46. Abbrev: DRI45; Default: 0xFF(-); DF Addr: 00F1h; SRAM Offset: 00D4h ; SubClass Offset : 45
	///     Drive Trace#45
	U1 bDri45;                                                 
	/// 47. Abbrev: DRI46; Default: 0xFF(-); DF Addr: 00F2h; SRAM Offset: 00D5h ; SubClass Offset : 46
	///     Drive Trace#46
	U1 bDri46;                                                 
	/// 48. Abbrev: DRI47; Default: 0xFF(-); DF Addr: 00F3h; SRAM Offset: 00D6h ; SubClass Offset : 47
	///     Drive Trace#47
	U1 bDri47;                                                 
	/// 49. Abbrev: DRI48; Default: 0xFF(-); DF Addr: 00F4h; SRAM Offset: 00D7h ; SubClass Offset : 48
	///     Drive Trace#48
	U1 bDri48;                                                 
	/// 50. Abbrev: DRI49; Default: 0xFF(-); DF Addr: 00F5h; SRAM Offset: 00D8h ; SubClass Offset : 49
	///     Drive Trace#49
	U1 bDri49;                                                 
	/// 51. Abbrev: DRI50; Default: 0xFF(-); DF Addr: 00F6h; SRAM Offset: 00D9h ; SubClass Offset : 50
	///     Drive Trace#50
	U1 bDri50;                                                 
	/// 52. Abbrev: DRI51; Default: 0xFF(-); DF Addr: 00F7h; SRAM Offset: 00DAh ; SubClass Offset : 51
	///     Drive Trace#51
	U1 bDri51;                                                 
} SubClassPanelDriveAxisType;	///< Total = 52 Bytes

/// =============================================================///
/// Data flash Class #3 => TraceSetup
/// =============================================================///
typedef struct PACK ClassTraceSetupSt
{
	SubClassPanelSenseAxisType scPanelSenseAxis;
	SubClassPanelDriveAxisType scPanelDriveAxis;
}ClassTraceSetupType;	///< Total = 104 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #9 => NonActiveTrace
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNonActiveTraceSt
{
	///  1. Abbrev: MUT_SEN_INACT; Default: 0x00(-); DF Addr: 00F8h; SRAM Offset: 00DBh ; SubClass Offset : 0
	///     [!!!] Mutual Scan Sense In-active :When even sense ,what odd doing : 0:+Wave 1:-Wave 2:VDD 3:GND 4:FLOAT 5:ADC 6:VCOM 7:HiZ
	U1 bMutSenInact;                                           
	///  2. Abbrev: MUT_DRI_ACT; Default: 0x00(-); DF Addr: 00F9h; SRAM Offset: 00DCh ; SubClass Offset : 1
	///     [!!!] Mutual Scan Sense Axis in-active : 0:+Wave 1:-Wave 2:VDD 3:GND 4:FLOAT 5:ADC 6:VCOM 7:HiZ
	U1 bMutDriAct;                                             
	///  3. Abbrev: MUT_DRI_INACT; Default: 0x00(-); DF Addr: 00FAh; SRAM Offset: 00DDh ; SubClass Offset : 2
	///     [!!!] Mutual Scan Drive Axis Active : When Trace N Driveing , What the other Drive-Trace doing : 0:+Wave 1:-Wave 2:VDD 3:GND 4:FLOAT 5:ADC 6:VCOM 7:HiZ
	U1 bMutDriInact;                                           
	///  4. Abbrev: INIT_SEN_INACT; Default: 0x03(-); DF Addr: 00FBh; SRAM Offset: 00DEh ; SubClass Offset : 3
	///     [!!!] Init Scan Sense Axis In-active
	U1 bInitSenInact;                                          
	///  5. Abbrev: INIT_DRI_ACT; Default: 0x00(-); DF Addr: 00FCh; SRAM Offset: 00DFh ; SubClass Offset : 4
	///     [!!!] Drive Axis type
	U1 bInitDriAct;                                            
	///  6. Abbrev: INIT_DRI_INACT; Default: 0x03(-); DF Addr: 00FDh; SRAM Offset: 00E0h ; SubClass Offset : 5
	///     [!!!] Init Scan Drive Axis In-active
	U1 bInitDriInact;                                          
	///  7. Abbrev: INIT_SEN_ACT; Default: 0x00(-); DF Addr: 00FEh; SRAM Offset: 00E1h ; SubClass Offset : 6
	///     [!!!] Init Scan Sense Axis active : 0：Pwave,  1: Nwave,  2:  VDD, 3: GND, 4: FLOATING, 5: ADC, 6: VCOM, 7:HI-Z
	U1 bInitSenact;                                            
} SubClassNonActiveTraceType;	///< Total = 7 Bytes

/// =============================================================///
/// Data flash Class #4 => AxisCfg
/// =============================================================///
typedef struct PACK ClassAxisCfgSt
{
	SubClassNonActiveTraceType scNonActiveTrace;
}ClassAxisCfgType;	///< Total = 7 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #10 => InitialNormal
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassInitialNormalSt
{
	///  1. Abbrev: ADC_CTRL0_INIT; Default: 0x00(-); DF Addr: 0100h; SRAM Offset: 00E2h ; SubClass Offset : 0
	///     ADC_CTRL_0 Initial
	U1 bInitADCCtrl0;                                          
	///  2. Abbrev: REG_AD_CTRL1_INIT; Default: 0x20(-); DF Addr: 0101h; SRAM Offset: 00E3h ; SubClass Offset : 1
	///     ADC_CTRL_1 Reg[0x001B8004]
	U1 bInitADCCtrl1;                                          
	///  3. Abbrev: REG32_TX_CTRL_INIT; Default: 0x1100(-); DF Addr: 0102h; SRAM Offset: 00E4h ; SubClass Offset : 2
	///     \n1.TX_CTRL[14:12]=TX_CTRL[10:8] \n2.charger pump off 時,只能設成0~1 \n3.charger pump on, 0~7
	U2 wInitTxCtrl;                                            
	///  4. Abbrev: REG_PN_CTRL_INIT; Default: 0x04(-); DF Addr: 0104h; SRAM Offset: 00E6h ; SubClass Offset : 4
	///     Reg[0x00192000]
	U1 bInitPnCtrl;                                            
	///  5. Abbrev: INIT_TH_WATER; Default: 0x0032(-); DF Addr: 0105h; SRAM Offset: 00E7h ; SubClass Offset : 5
	///     
	U2 wInitWaterTH;                                           
	///  6. Abbrev: MUT_TH_WATER; Default: 0x0032(-); DF Addr: 0107h; SRAM Offset: 00E9h ; SubClass Offset : 7
	///     
	U2 wMutWaterTH;                                            
	///  7. Abbrev: MUT_TH_WATER_CNT_ON; Default: 0x14(-); DF Addr: 0109h; SRAM Offset: 00EBh ; SubClass Offset : 9
	///     
	U1 bMutWaterTHCntOn;                                       
	///  8. Abbrev: MUT_TH_WATER_CNT_OFF; Default: 0x0A(-); DF Addr: 010Ah; SRAM Offset: 00ECh ; SubClass Offset : 10
	///     
	U1 bMutWaterTHCntOff;                                      
	///  9. Abbrev: MUT_FINGER_WATER_CNT_ON; Default: 0x32(-); DF Addr: 010Bh; SRAM Offset: 00EDh ; SubClass Offset : 11
	///     
	U1 bMutFingerWaterTHCntOn;                                 
	/// 10. Abbrev: MUT_FINGER_WATER_CNT_OFF; Default: 0x32(-); DF Addr: 010Ch; SRAM Offset: 00EEh ; SubClass Offset : 12
	///     
	U1 bMutFingerWaterTHCntOff;                                
	/// 11. Abbrev: INIT_WATER_REPORT_DEBOUNCE; Default: 0x05(-); DF Addr: 010Dh; SRAM Offset: 00EFh ; SubClass Offset : 13
	///     
	U1 bInitReportDebounce;                                    
	/// 12. Abbrev: INIT_WATER_BASE_TRACK_DEB; Default: 0x0064(-); DF Addr: 010Eh; SRAM Offset: 00F0h ; SubClass Offset : 14
	///     
	U2 wWaterModeBaseTrackDeb;                                 
} SubClassInitialNormalType;	///< Total = 16 Bytes

/// =============================================================///
/// Data flash Class #5 => InitialAnalog
/// =============================================================///
typedef struct PACK ClassInitialAnalogSt
{
	SubClassInitialNormalType scInitialNormal;
}ClassInitialAnalogType;	///< Total = 16 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #11 => MutualNormal
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassMutualNormalSt
{
	///  1. Abbrev: TX_FREQ; Default: 0x00001DE5(-); DF Addr: 0110h; SRAM Offset: 00F2h ; SubClass Offset : 0
	///     
	U4 wTXFreq;                                                
	///  2. Abbrev: TX_PERIOD; Default: 0x0083(-); DF Addr: 0114h; SRAM Offset: 00F6h ; SubClass Offset : 4
	///     
	U2 wTXPeriod;                                              
	///  3. Abbrev: TONE_A_MUL; Default: 0x19(-); DF Addr: 0116h; SRAM Offset: 00F8h ; SubClass Offset : 6
	///     
	U1 bToneAMultiple;                                         
	///  4. Abbrev: TONE_B_MUL; Default: 0x1B(-); DF Addr: 0117h; SRAM Offset: 00F9h ; SubClass Offset : 7
	///     
	U1 bToneBMultiple;                                         
	///  5. Abbrev: TONE_A_FREQ; Default: 0x00BF(-); DF Addr: 0118h; SRAM Offset: 00FAh ; SubClass Offset : 8
	///     
	U2 wTonAFequency;                                          
	///  6. Abbrev: TONE_B_FREQ; Default: 0x00CE(-); DF Addr: 011Ah; SRAM Offset: 00FCh ; SubClass Offset : 10
	///     
	U2 wTonBFequency;                                          
	///  7. Abbrev: MIN_TONE_NUM; Default: 0x15(-); DF Addr: 011Ch; SRAM Offset: 00FEh ; SubClass Offset : 12
	///     
	U1 bMinToneNum;                                            
	///  8. Abbrev: REG_SEL18; Default: 0xFFFF(-); DF Addr: 011Dh; SRAM Offset: 00FFh ; SubClass Offset : 13
	///     Interface IO voltage,0x0000 -> 3.3V ; 0xFFFF -> 1.8V
	U2 wRegSel18;                                              
	///  9. Abbrev: REG_RX_HOLD_CYCE; Default: 0x0005(-); DF Addr: 011Fh; SRAM Offset: 0101h ; SubClass Offset : 15
	///     Reg[0x00192048]
	U2 wRXHoldCycle;                                           
	/// 10. Abbrev: REG_PN_CTRL; Default: 0x15(-); DF Addr: 0121h; SRAM Offset: 0103h ; SubClass Offset : 17
	///     Reg[0x00192000]
	U1 bPnCtrl;                                                
	/// 11. Abbrev: REG_PN_CHIP_NUM; Default: 0x0F(-); DF Addr: 0122h; SRAM Offset: 0104h ; SubClass Offset : 18
	///     Reg[0x00190002]
	U1 bPnChipNum;                                             
	/// 12. Abbrev: SRM_ADC_CTRL0_MUT; Default: 0x50(-); DF Addr: 0123h; SRAM Offset: 0105h ; SubClass Offset : 19
	///     ADC_CTRL_0 Mutual
	U1 bADC0Gain;                                              
	/// 13. Abbrev: REG_AD_CTRL1; Default: 0x60(-); DF Addr: 0124h; SRAM Offset: 0106h ; SubClass Offset : 20
	///     ADC_CTRL_1 Reg[0x001B8004]
	U1 bADCCtrl1;                                              
	/// 14. Abbrev: REG_AD_CTRL5; Default: 0x33(-); DF Addr: 0125h; SRAM Offset: 0107h ; SubClass Offset : 21
	///     Reg[0x001B8014]
	U1 bADCCtrl5;                                              
	/// 15. Abbrev: REG_AD_CTRL6; Default: 0x33(-); DF Addr: 0126h; SRAM Offset: 0108h ; SubClass Offset : 22
	///     Reg[0x001B8018]
	U1 bADCCtrl6;                                              
	/// 16. Abbrev: REG_AD_CTRL8; Default: 0x11(-); DF Addr: 0127h; SRAM Offset: 0109h ; SubClass Offset : 23
	///     Reg[0x001B8020]
	U1 bADCCtrl8;                                              
	/// 17. Abbrev: REG_AD_CTRL9; Default: 0x0000(-); DF Addr: 0128h; SRAM Offset: 010Ah ; SubClass Offset : 24
	///     Reg[0x001B8024]
	U2 wADCCtrl9;                                              
	/// 18. Abbrev: REG_AD_CTRL10; Default: 0x90(-); DF Addr: 012Ah; SRAM Offset: 010Ch ; SubClass Offset : 26
	///     ADC_CTRL_10 Reg[0x001B8028]
	U1 bADCCtrl10;                                             
	/// 19. Abbrev: REG32_CHG_PUMP; Default: 0x05(-); DF Addr: 012Bh; SRAM Offset: 010Dh ; SubClass Offset : 27
	///     
	U1 bRegCHGPUMP;                                            
	/// 20. Abbrev: REG_VREF6VN; Default: 0x02(-); DF Addr: 012Ch; SRAM Offset: 010Eh ; SubClass Offset : 28
	///     REG_VREF6V Reg[0x001B8060]
	U1 bVref6v;                                                
	/// 21. Abbrev: REG_VREF3VN; Default: 0x01(-); DF Addr: 012Dh; SRAM Offset: 010Fh ; SubClass Offset : 29
	///     REG_VREF3V Reg[0x001B8064]
	U1 bVref3v;                                                
	/// 22. Abbrev: REG_TEST_CTRL; Default: 0x00(-); DF Addr: 012Eh; SRAM Offset: 0110h ; SubClass Offset : 30
	///     Reg[0x001B8090]
	U1 bRegTESTCTRL;                                           
	/// 23. Abbrev: REG_DAC_CTRL0; Default: 0x00(-); DF Addr: 012Fh; SRAM Offset: 0111h ; SubClass Offset : 31
	///     Reg[0x001BA000]
	U1 bDACCtrl0;                                              
	/// 24. Abbrev: REG_DAC_CTRL1; Default: 0x33(-); DF Addr: 0130h; SRAM Offset: 0112h ; SubClass Offset : 32
	///     Reg[0x001BA004]
	U1 bDACCtrl1;                                              
	/// 25. Abbrev: REG_DAC_CTRL2; Default: 0x4052(-); DF Addr: 0131h; SRAM Offset: 0113h ; SubClass Offset : 33
	///     Reg[0x001BA008]
	U2 wDACCtrl2;                                              
	/// 26. Abbrev: REG32_TX_CTRL; Default: 0x7700(-); DF Addr: 0133h; SRAM Offset: 0115h ; SubClass Offset : 35
	///     \n1.TX_CTRL[14:12]=TX_CTRL[10:8] \n2.charger pump off 時,只能設成0~1 \n3.charger pump on, 0~7
	U2 wTxCtrl;                                                
	/// 27. Abbrev: REG32_TX_BUF; Default: 0x00(-); DF Addr: 0135h; SRAM Offset: 0117h ; SubClass Offset : 37
	///     
	U1 bTxBuf;                                                 
	/// 28. Abbrev: REG_TX_OP_CTRL0; Default: 0x33(-); DF Addr: 0136h; SRAM Offset: 0118h ; SubClass Offset : 38
	///     Reg[0x001BA018]
	U1 bTxOpCtrl0;                                             
	/// 29. Abbrev: REG_TX_OP_CTRL1; Default: 0x33(-); DF Addr: 0137h; SRAM Offset: 0119h ; SubClass Offset : 39
	///     Reg[0x001BA01C]
	U1 bTxOpCtrl1;                                             
	/// 30. Abbrev: REG_TX_OP_CTRL2; Default: 0x03(-); DF Addr: 0138h; SRAM Offset: 011Ah ; SubClass Offset : 40
	///     Reg[0x001BA020]
	U1 bTxOpCtrl2;                                             
	/// 31. Abbrev: REG_VOLT_CTRL; Default: 0x80(-); DF Addr: 0139h; SRAM Offset: 011Bh ; SubClass Offset : 41
	///     Reg[0x001C0010]
	U1 bRegVoltCtrl;                                           
	/// 32. Abbrev: REG32_BIAS; Default: 0xBB(-); DF Addr: 013Ah; SRAM Offset: 011Ch ; SubClass Offset : 42
	///     
	U1 bBias;                                                  
	/// 33. Abbrev: REG_PN_DURA_LO; Default: 0x0064(-); DF Addr: 013Bh; SRAM Offset: 011Dh ; SubClass Offset : 43
	///     Reg[0x00190004]
	U2 wPNDuraCntLo;                                           
	/// 34. Abbrev: REG_PN_DURA_HI; Default: 0x0064(-); DF Addr: 013Dh; SRAM Offset: 011Fh ; SubClass Offset : 45
	///     Reg[0x00190004]
	U2 wPNDuraCntHi;                                           
	/// 35. Abbrev: REG_ADC_RST_CNT_LO; Default: 0x0050(-); DF Addr: 013Fh; SRAM Offset: 0121h ; SubClass Offset : 47
	///     Reg[0x00190004]
	U2 wAdcRstCntLo;                                           
	/// 36. Abbrev: REG_ADC_RST_CNT_HI; Default: 0x0050(-); DF Addr: 0141h; SRAM Offset: 0123h ; SubClass Offset : 49
	///     Reg[0x00190004]
	U2 wAdcRstCntHi;                                           
	/// 37. Abbrev: SINE_TABLE_ID; Default: 0x00(-); DF Addr: 0143h; SRAM Offset: 0125h ; SubClass Offset : 51
	///     
	U1 bSineParaTableID;                                       
	/// 38. Abbrev: REG32_VBG; Default: 0x00(-); DF Addr: 0144h; SRAM Offset: 0126h ; SubClass Offset : 52
	///     
	U1 bVBG;                                                   
	/// 39. Abbrev: NOISE_REJECTION_ENABLE; Default: 0x00(-); DF Addr: 0145h; SRAM Offset: 0127h ; SubClass Offset : 53
	///     
	U1 bNoiseRejectEn;                                         
} SubClassMutualNormalType;	///< Total = 54 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #12 => MutualCharge
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassMutualChargeSt
{
	///  1. Abbrev: REG_SYS_MODE_CTRL; Default: 0x00(-); DF Addr: 0146h; SRAM Offset: 0128h ; SubClass Offset : 0
	///     S
	U1 bSysModeCtrl;                                           
} SubClassMutualChargeType;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #6 => MutualAnalog
/// =============================================================///
typedef struct PACK ClassMutualAnalogSt
{
	SubClassMutualNormalType scMutualNormal;
	SubClassMutualChargeType scMutualCharge;
}ClassMutualAnalogType;	///< Total = 55 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #13 => ForceNormal
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassForceNormalSt
{
	///  1. Abbrev: REG_FORCE_TOUCH_TYPE; Default: 0x04(-); DF Addr: 0147h; SRAM Offset: 0129h ; SubClass Offset : 0
	///     
	U1 bForceType;                                             
	///  2. Abbrev: REG_PN_CTRL_FORCE; Default: 0x04(-); DF Addr: 0148h; SRAM Offset: 012Ah ; SubClass Offset : 1
	///     Reg[0x00192000]
	U1 bForcePnCtrl;                                           
	///  3. Abbrev: REG_PN_CHIP_NUM_FORCE; Default: 0x07(-); DF Addr: 0149h; SRAM Offset: 012Bh ; SubClass Offset : 2
	///     Reg[0x00190002]
	U1 bForcePnChipNum;                                        
	///  4. Abbrev: SRM_ADC_CTRL0_MUT_FORCE; Default: 0x00(-); DF Addr: 014Ah; SRAM Offset: 012Ch ; SubClass Offset : 3
	///     ADC_CTRL_0 Force
	U1 bForceADC0Gain;                                         
	///  5. Abbrev: REG_AD_CTRL1_FORCE; Default: 0x20(-); DF Addr: 014Bh; SRAM Offset: 012Dh ; SubClass Offset : 4
	///     ADC_CTRL_1 Reg[0x001B8004]
	U1 bForceADCCtrl1;                                         
	///  6. Abbrev: REG_AD_CTRL10_FORCE; Default: 0x80(-); DF Addr: 014Ch; SRAM Offset: 012Eh ; SubClass Offset : 5
	///     ADC_CTRL_10 Reg[0x001B8028]
	U1 bForceADCCtrl10;                                        
	///  7. Abbrev: REG32_CHG_PUMP_FORCE; Default: 0x04(-); DF Addr: 014Dh; SRAM Offset: 012Fh ; SubClass Offset : 6
	///     
	U1 bForceRegCHGPUMP;                                       
	///  8. Abbrev: REG32_TX_CTRL_FORCE; Default: 0x1100(-); DF Addr: 014Eh; SRAM Offset: 0130h ; SubClass Offset : 7
	///     \n1.TX_CTRL[14:12]=TX_CTRL[10:8] \n2.charger pump off 時,只能設成0~1 \n3.charger pump on, 0~7
	U2 wForceTxCtrl;                                           
	///  9. Abbrev: FORCE_DEV_SENSITIVITY; Default: 0x00(-); DF Addr: 0150h; SRAM Offset: 0132h ; SubClass Offset : 9
	///     
	U1 bForceSensirivity;                                      
} SubClassForceNormalType;	///< Total = 10 Bytes

/// =============================================================///
/// Data flash Class #7 => ForceAnalog
/// =============================================================///
typedef struct PACK ClassForceAnalogSt
{
	SubClassForceNormalType scForceNormal;
}ClassForceAnalogType;	///< Total = 10 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #14 => Interface
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassInterfaceSt
{
	///  1. Abbrev: REG_I2CD_ADDR; Default: 0x20(-); DF Addr: 0151h; SRAM Offset: 0133h ; SubClass Offset : 0
	///     SFR default value : I2CDADDR
	U1 bRegI2CDADDR;                                           
	///  2. Abbrev: REG_I2C_CTRL; Default: 0x00(-); DF Addr: 0152h; SRAM Offset: 0134h ; SubClass Offset : 1
	///     
	U1 bI2Ctrl;                                                
	///  3. Abbrev: REG_I2C_CON2; Default: 0x00(-); DF Addr: 0153h; SRAM Offset: 0135h ; SubClass Offset : 2
	///     
	U1 bI2CCon2;                                               
	///  4. Abbrev: REG_FW_ADDR; Default: 0x77(-); DF Addr: 0154h; SRAM Offset: 0136h ; SubClass Offset : 3
	///     SFR default value : FWADR
	U1 bRegFWADR;                                              
	///  5. Abbrev: REG_SPI_MODE; Default: 0xE0(-); DF Addr: 0155h; SRAM Offset: 0137h ; SubClass Offset : 4
	///     SPI mode[7]: 0:disable,1:enable;SPI mode[6]: 0:SPI mode 0 and 3, 1:SPI mode 1 and 2.
	U1 bRegSPIMode;                                            
	///  6. Abbrev: REG_PORT_CFG; Default: 0x00(-); DF Addr: 0156h; SRAM Offset: 0138h ; SubClass Offset : 5
	///     SFR default value : Port Pull High Resister Control
	U1 bRegPortCfg;                                            
	///  7. Abbrev: REG_PORT1_PH; Default: 0x00(-); DF Addr: 0157h; SRAM Offset: 0139h ; SubClass Offset : 6
	///     SFR default value : PORT1_PH , Pull high function enable register of PORT 1.
	U1 bRegPort1PH;                                            
	///  8. Abbrev: REG_PORT3_PH; Default: 0x00(-); DF Addr: 0158h; SRAM Offset: 013Ah ; SubClass Offset : 7
	///     SFR default value : PORT3_PH , Pull high function enable register of PORT 3.
	U1 bRegPort3PH;                                            
} SubClassInterfaceType;	///< Total = 8 Bytes

/// =============================================================///
/// Data flash Class #8 => MiscRegister
/// =============================================================///
typedef struct PACK ClassMiscRegisterSt
{
	SubClassInterfaceType scInterface;
}ClassMiscRegisterType;	///< Total = 8 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #15 => AxisDevRatio
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAxisDevRatioSt
{
	///  1. Abbrev: AXIS_CHANNEL_DEV_RATIO_EN; Default: 0x02(-); DF Addr: 0159h; SRAM Offset: 013Bh ; SubClass Offset : 0
	///     Sense Axis Dev[n] Ratio : Raw Dev * N/128 enable
	U1 bAxisRatioDevEnable;                                    
	///  2. Abbrev: AXIS_CHANNEL_DEV_RATIO_00; Default: 0x80(-); DF Addr: 015Ah; SRAM Offset: 013Ch ; SubClass Offset : 1
	///     Sense Axis Dev[0] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev00;                                        
	///  3. Abbrev: AXIS_CHANNEL_DEV_RATIO_01; Default: 0x80(-); DF Addr: 015Bh; SRAM Offset: 013Dh ; SubClass Offset : 2
	///     Sense Axis Dev[1] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev01;                                        
	///  4. Abbrev: AXIS_CHANNEL_DEV_RATIO_02; Default: 0x80(-); DF Addr: 015Ch; SRAM Offset: 013Eh ; SubClass Offset : 3
	///     Sense Axis Dev[2] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev02;                                        
	///  5. Abbrev: AXIS_CHANNEL_DEV_RATIO_03; Default: 0x80(-); DF Addr: 015Dh; SRAM Offset: 013Fh ; SubClass Offset : 4
	///     Sense Axis Dev[3] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev03;                                        
	///  6. Abbrev: AXIS_CHANNEL_DEV_RATIO_04; Default: 0x80(-); DF Addr: 015Eh; SRAM Offset: 0140h ; SubClass Offset : 5
	///     Sense Axis Dev[4] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev04;                                        
	///  7. Abbrev: AXIS_CHANNEL_DEV_RATIO_05; Default: 0x80(-); DF Addr: 015Fh; SRAM Offset: 0141h ; SubClass Offset : 6
	///     Sense Axis Dev[5] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev05;                                        
	///  8. Abbrev: AXIS_CHANNEL_DEV_RATIO_06; Default: 0x80(-); DF Addr: 0160h; SRAM Offset: 0142h ; SubClass Offset : 7
	///     Sense Axis Dev[6] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev06;                                        
	///  9. Abbrev: AXIS_CHANNEL_DEV_RATIO_07; Default: 0x80(-); DF Addr: 0161h; SRAM Offset: 0143h ; SubClass Offset : 8
	///     Sense Axis Dev[7] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev07;                                        
	/// 10. Abbrev: AXIS_CHANNEL_DEV_RATIO_08; Default: 0x80(-); DF Addr: 0162h; SRAM Offset: 0144h ; SubClass Offset : 9
	///     Sense Axis Dev[8] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev08;                                        
	/// 11. Abbrev: AXIS_CHANNEL_DEV_RATIO_09; Default: 0x80(-); DF Addr: 0163h; SRAM Offset: 0145h ; SubClass Offset : 10
	///     Sense Axis Dev[9] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev09;                                        
	/// 12. Abbrev: AXIS_CHANNEL_DEV_RATIO_10; Default: 0x80(-); DF Addr: 0164h; SRAM Offset: 0146h ; SubClass Offset : 11
	///     Sense Axis Dev[10] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev10;                                        
	/// 13. Abbrev: AXIS_CHANNEL_DEV_RATIO_11; Default: 0x80(-); DF Addr: 0165h; SRAM Offset: 0147h ; SubClass Offset : 12
	///     Sense Axis Dev[11] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev11;                                        
	/// 14. Abbrev: AXIS_CHANNEL_DEV_RATIO_12; Default: 0x80(-); DF Addr: 0166h; SRAM Offset: 0148h ; SubClass Offset : 13
	///     Sense Axis Dev[12] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev12;                                        
	/// 15. Abbrev: AXIS_CHANNEL_DEV_RATIO_13; Default: 0x80(-); DF Addr: 0167h; SRAM Offset: 0149h ; SubClass Offset : 14
	///     Sense Axis Dev[13] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev13;                                        
	/// 16. Abbrev: AXIS_CHANNEL_DEV_RATIO_14; Default: 0x80(-); DF Addr: 0168h; SRAM Offset: 014Ah ; SubClass Offset : 15
	///     Sense Axis Dev[14] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev14;                                        
	/// 17. Abbrev: AXIS_CHANNEL_DEV_RATIO_15; Default: 0x80(-); DF Addr: 0169h; SRAM Offset: 014Bh ; SubClass Offset : 16
	///     Sense Axis Dev[15] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev15;                                        
	/// 18. Abbrev: AXIS_CHANNEL_DEV_RATIO_16; Default: 0x80(-); DF Addr: 016Ah; SRAM Offset: 014Ch ; SubClass Offset : 17
	///     Sense Axis Dev[16] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev16;                                        
	/// 19. Abbrev: AXIS_CHANNEL_DEV_RATIO_17; Default: 0x80(-); DF Addr: 016Bh; SRAM Offset: 014Dh ; SubClass Offset : 18
	///     Sense Axis Dev[17] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev17;                                        
	/// 20. Abbrev: AXIS_CHANNEL_DEV_RATIO_18; Default: 0x80(-); DF Addr: 016Ch; SRAM Offset: 014Eh ; SubClass Offset : 19
	///     Sense Axis Dev[18] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev18;                                        
	/// 21. Abbrev: AXIS_CHANNEL_DEV_RATIO_19; Default: 0x80(-); DF Addr: 016Dh; SRAM Offset: 014Fh ; SubClass Offset : 20
	///     Sense Axis Dev[19] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev19;                                        
	/// 22. Abbrev: AXIS_CHANNEL_DEV_RATIO_20; Default: 0x80(-); DF Addr: 016Eh; SRAM Offset: 0150h ; SubClass Offset : 21
	///     Sense Axis Dev[20] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev20;                                        
	/// 23. Abbrev: AXIS_CHANNEL_DEV_RATIO_21; Default: 0x80(-); DF Addr: 016Fh; SRAM Offset: 0151h ; SubClass Offset : 22
	///     Sense Axis Dev[21] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev21;                                        
	/// 24. Abbrev: AXIS_CHANNEL_DEV_RATIO_22; Default: 0x80(-); DF Addr: 0170h; SRAM Offset: 0152h ; SubClass Offset : 23
	///     Sense Axis Dev[22] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev22;                                        
	/// 25. Abbrev: AXIS_CHANNEL_DEV_RATIO_23; Default: 0x80(-); DF Addr: 0171h; SRAM Offset: 0153h ; SubClass Offset : 24
	///     Sense Axis Dev[23] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev23;                                        
	/// 26. Abbrev: AXIS_CHANNEL_DEV_RATIO_24; Default: 0x80(-); DF Addr: 0172h; SRAM Offset: 0154h ; SubClass Offset : 25
	///     Sense Axis Dev[24] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev24;                                        
	/// 27. Abbrev: AXIS_CHANNEL_DEV_RATIO_25; Default: 0x80(-); DF Addr: 0173h; SRAM Offset: 0155h ; SubClass Offset : 26
	///     Sense Axis Dev[25] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev25;                                        
	/// 28. Abbrev: AXIS_CHANNEL_DEV_RATIO_26; Default: 0x80(-); DF Addr: 0174h; SRAM Offset: 0156h ; SubClass Offset : 27
	///     Sense Axis Dev[26] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev26;                                        
	/// 29. Abbrev: AXIS_CHANNEL_DEV_RATIO_27; Default: 0x80(-); DF Addr: 0175h; SRAM Offset: 0157h ; SubClass Offset : 28
	///     Sense Axis Dev[27] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev27;                                        
	/// 30. Abbrev: AXIS_CHANNEL_DEV_RATIO_28; Default: 0xA0(-); DF Addr: 0176h; SRAM Offset: 0158h ; SubClass Offset : 29
	///     Sense Axis Dev[28] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev28;                                        
	/// 31. Abbrev: AXIS_CHANNEL_DEV_RATIO_29; Default: 0xB4(-); DF Addr: 0177h; SRAM Offset: 0159h ; SubClass Offset : 30
	///     Sense Axis Dev[29] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev29;                                        
	/// 32. Abbrev: AXIS_CHANNEL_DEV_RATIO_30; Default: 0x80(-); DF Addr: 0178h; SRAM Offset: 015Ah ; SubClass Offset : 31
	///     Sense Axis Dev[30] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev30;                                        
	/// 33. Abbrev: AXIS_CHANNEL_DEV_RATIO_31; Default: 0x80(-); DF Addr: 0179h; SRAM Offset: 015Bh ; SubClass Offset : 32
	///     Sense Axis Dev[31] Ratio : Raw Dev * N/128
	U1 bAxisRatioDev31;                                        
} SubClassAxisDevRatioType;	///< Total = 33 Bytes

/// =============================================================///
/// Data flash Class #9 => AlogAxisDevRatio
/// =============================================================///
typedef struct PACK ClassAlogAxisDevRatioSt
{
	SubClassAxisDevRatioType scAxisDevRatio;
}ClassAlogAxisDevRatioType;	///< Total = 33 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #16 => FingerEnhancement
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassFingerEnhancementSt
{
	///  1. Abbrev: FINGER_ENHANCE_EN; Default: 0x00(-); DF Addr: 017Ah; SRAM Offset: 015Ch ; SubClass Offset : 0
	///     [DYN]enable finger enhancement algorithm
	U1 bFingerEnhanceEn;                                       
} SubClassFingerEnhancementType;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #10 => FloatingIssue
/// =============================================================///
typedef struct PACK ClassFloatingIssueSt
{
	SubClassFingerEnhancementType scFingerEnhancement;
}ClassFloatingIssueType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #17 => ForceSenseAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassForceSenseAxisSt
{
	///  1. Abbrev: FORCE_SEN00; Default: 0x80(-); DF Addr: 0180h; SRAM Offset: 015Dh ; SubClass Offset : 0
	///     Force Touch Sense Trace #0
	U1 bForceSen00;                                            
	///  2. Abbrev: FORCE_SEN01; Default: 0x81(-); DF Addr: 0181h; SRAM Offset: 015Eh ; SubClass Offset : 1
	///     Force Touch Sense Trace #1
	U1 bForceSen01;                                            
	///  3. Abbrev: FORCE_SEN02; Default: 0x82(-); DF Addr: 0182h; SRAM Offset: 015Fh ; SubClass Offset : 2
	///     Force Touch Sense Trace #2
	U1 bForceSen02;                                            
	///  4. Abbrev: FORCE_SEN03; Default: 0x83(-); DF Addr: 0183h; SRAM Offset: 0160h ; SubClass Offset : 3
	///     Force Touch Sense Trace #3
	U1 bForceSen03;                                            
	///  5. Abbrev: FORCE_SEN04; Default: 0x84(-); DF Addr: 0184h; SRAM Offset: 0161h ; SubClass Offset : 4
	///     Force Touch Sense Trace #4
	U1 bForceSen04;                                            
	///  6. Abbrev: FORCE_SEN05; Default: 0x85(-); DF Addr: 0185h; SRAM Offset: 0162h ; SubClass Offset : 5
	///     Force Touch Sense Trace #5
	U1 bForceSen05;                                            
	///  7. Abbrev: FORCE_SEN06; Default: 0x86(-); DF Addr: 0186h; SRAM Offset: 0163h ; SubClass Offset : 6
	///     Force Touch Sense Trace #6
	U1 bForceSen06;                                            
	///  8. Abbrev: FORCE_SEN07; Default: 0x87(-); DF Addr: 0187h; SRAM Offset: 0164h ; SubClass Offset : 7
	///     Force Touch Sense Trace #7
	U1 bForceSen07;                                            
	///  9. Abbrev: FORCE_SEN08; Default: 0x88(-); DF Addr: 0188h; SRAM Offset: 0165h ; SubClass Offset : 8
	///     Force Touch Sense Trace #8
	U1 bForceSen08;                                            
	/// 10. Abbrev: FORCE_SEN09; Default: 0x89(-); DF Addr: 0189h; SRAM Offset: 0166h ; SubClass Offset : 9
	///     Force Touch Sense Trace #9
	U1 bForceSen09;                                            
	/// 11. Abbrev: FORCE_SEN10; Default: 0x89(-); DF Addr: 018Ah; SRAM Offset: 0167h ; SubClass Offset : 10
	///     Force Touch Sense Trace #10
	U1 bForceSen10;                                            
	/// 12. Abbrev: FORCE_SEN11; Default: 0x8A(-); DF Addr: 018Bh; SRAM Offset: 0168h ; SubClass Offset : 11
	///     Force Touch Sense Trace #11
	U1 bForceSen11;                                            
	/// 13. Abbrev: FORCE_SEN12; Default: 0x8B(-); DF Addr: 018Ch; SRAM Offset: 0169h ; SubClass Offset : 12
	///     Force Touch Sense Trace #12
	U1 bForceSen12;                                            
	/// 14. Abbrev: FORCE_SEN13; Default: 0x8B(-); DF Addr: 018Dh; SRAM Offset: 016Ah ; SubClass Offset : 13
	///     Force Touch Sense Trace #13
	U1 bForceSen13;                                            
	/// 15. Abbrev: FORCE_SEN14; Default: 0x8C(-); DF Addr: 018Eh; SRAM Offset: 016Bh ; SubClass Offset : 14
	///     Force Touch Sense Trace #14
	U1 bForceSen14;                                            
	/// 16. Abbrev: FORCE_SEN15; Default: 0x8D(-); DF Addr: 018Fh; SRAM Offset: 016Ch ; SubClass Offset : 15
	///     Force Touch Sense Trace #15
	U1 bForceSen15;                                            
	/// 17. Abbrev: FORCE_SEN16; Default: 0x8E(-); DF Addr: 0190h; SRAM Offset: 016Dh ; SubClass Offset : 16
	///     Force Touch Sense Trace #16
	U1 bForceSen16;                                            
	/// 18. Abbrev: FORCE_SEN17; Default: 0x8F(-); DF Addr: 0191h; SRAM Offset: 016Eh ; SubClass Offset : 17
	///     Force Touch Sense Trace #17
	U1 bForceSen17;                                            
	/// 19. Abbrev: FORCE_SEN18; Default: 0x90(-); DF Addr: 0192h; SRAM Offset: 016Fh ; SubClass Offset : 18
	///     Force Touch Sense Trace #18
	U1 bForceSen18;                                            
	/// 20. Abbrev: FORCE_SEN19; Default: 0x90(-); DF Addr: 0193h; SRAM Offset: 0170h ; SubClass Offset : 19
	///     Force Touch Sense Trace #19
	U1 bForceSen19;                                            
	/// 21. Abbrev: FORCE_SEN20; Default: 0x90(-); DF Addr: 0194h; SRAM Offset: 0171h ; SubClass Offset : 20
	///     Force Touch Sense Trace #20
	U1 bForceSen20;                                            
	/// 22. Abbrev: FORCE_SEN21; Default: 0x90(-); DF Addr: 0195h; SRAM Offset: 0172h ; SubClass Offset : 21
	///     Force Touch Sense Trace #21
	U1 bForceSen21;                                            
	/// 23. Abbrev: FORCE_SEN22; Default: 0x90(-); DF Addr: 0196h; SRAM Offset: 0173h ; SubClass Offset : 22
	///     Force Touch Sense Trace #22
	U1 bForceSen22;                                            
	/// 24. Abbrev: FORCE_SEN23; Default: 0x90(-); DF Addr: 0197h; SRAM Offset: 0174h ; SubClass Offset : 23
	///     Force Touch Sense Trace #23
	U1 bForceSen23;                                            
	/// 25. Abbrev: FORCE_SEN24; Default: 0x90(-); DF Addr: 0198h; SRAM Offset: 0175h ; SubClass Offset : 24
	///     Force Touch Sense Trace #24
	U1 bForceSen24;                                            
	/// 26. Abbrev: FORCE_SEN25; Default: 0x90(-); DF Addr: 0199h; SRAM Offset: 0176h ; SubClass Offset : 25
	///     Force Touch Sense Trace #25
	U1 bForceSen25;                                            
	/// 27. Abbrev: FORCE_SEN26; Default: 0x90(-); DF Addr: 019Ah; SRAM Offset: 0177h ; SubClass Offset : 26
	///     Force Touch Sense Trace #26
	U1 bForceSen26;                                            
	/// 28. Abbrev: FORCE_SEN27; Default: 0x90(-); DF Addr: 019Bh; SRAM Offset: 0178h ; SubClass Offset : 27
	///     Force Touch Sense Trace #27
	U1 bForceSen27;                                            
	/// 29. Abbrev: FORCE_SEN28; Default: 0xFF(-); DF Addr: 019Ch; SRAM Offset: 0179h ; SubClass Offset : 28
	///     Force Touch Sense Trace #28
	U1 bForceSen28;                                            
	/// 30. Abbrev: FORCE_SEN29; Default: 0xFF(-); DF Addr: 019Dh; SRAM Offset: 017Ah ; SubClass Offset : 29
	///     Force Touch Sense Trace #29
	U1 bForceSen29;                                            
	/// 31. Abbrev: FORCE_SEN30; Default: 0xFF(-); DF Addr: 019Eh; SRAM Offset: 017Bh ; SubClass Offset : 30
	///     Force Touch Sense Trace #30
	U1 bForceSen30;                                            
	/// 32. Abbrev: FORCE_SEN31; Default: 0xFF(-); DF Addr: 019Fh; SRAM Offset: 017Ch ; SubClass Offset : 31
	///     Force Touch Sense Trace #31
	U1 bForceSen31;                                            
	/// 33. Abbrev: FORCE_SEN32; Default: 0xFF(-); DF Addr: 01A0h; SRAM Offset: 017Dh ; SubClass Offset : 32
	///     Force Touch Sense Trace #32
	U1 bForceSen32;                                            
	/// 34. Abbrev: FORCE_SEN33; Default: 0xFF(-); DF Addr: 01A1h; SRAM Offset: 017Eh ; SubClass Offset : 33
	///     Force Touch Sense Trace #33
	U1 bForceSen33;                                            
	/// 35. Abbrev: FORCE_SEN34; Default: 0xFF(-); DF Addr: 01A2h; SRAM Offset: 017Fh ; SubClass Offset : 34
	///     Force Touch Sense Trace #34
	U1 bForceSen34;                                            
	/// 36. Abbrev: FORCE_SEN35; Default: 0xFF(-); DF Addr: 01A3h; SRAM Offset: 0180h ; SubClass Offset : 35
	///     Force Touch Sense Trace #35
	U1 bForceSen35;                                            
	/// 37. Abbrev: FORCE_SEN36; Default: 0xFF(-); DF Addr: 01A4h; SRAM Offset: 0181h ; SubClass Offset : 36
	///     Force Touch Sense Trace #36
	U1 bForceSen36;                                            
	/// 38. Abbrev: FORCE_SEN37; Default: 0xFF(-); DF Addr: 01A5h; SRAM Offset: 0182h ; SubClass Offset : 37
	///     Force Touch Sense Trace #37
	U1 bForceSen37;                                            
	/// 39. Abbrev: FORCE_SEN38; Default: 0xFF(-); DF Addr: 01A6h; SRAM Offset: 0183h ; SubClass Offset : 38
	///     Force Touch Sense Trace #38
	U1 bForceSen38;                                            
	/// 40. Abbrev: FORCE_SEN39; Default: 0xFF(-); DF Addr: 01A7h; SRAM Offset: 0184h ; SubClass Offset : 39
	///     Force Touch Sense Trace #39
	U1 bForceSen39;                                            
	/// 41. Abbrev: FORCE_SEN40; Default: 0xFF(-); DF Addr: 01A8h; SRAM Offset: 0185h ; SubClass Offset : 40
	///     Force Touch Sense Trace #40
	U1 bForceSen40;                                            
	/// 42. Abbrev: FORCE_SEN41; Default: 0xFF(-); DF Addr: 01A9h; SRAM Offset: 0186h ; SubClass Offset : 41
	///     Force Touch Sense Trace #41
	U1 bForceSen41;                                            
	/// 43. Abbrev: FORCE_SEN42; Default: 0xFF(-); DF Addr: 01AAh; SRAM Offset: 0187h ; SubClass Offset : 42
	///     Force Touch Sense Trace #42
	U1 bForceSen42;                                            
	/// 44. Abbrev: FORCE_SEN43; Default: 0xFF(-); DF Addr: 01ABh; SRAM Offset: 0188h ; SubClass Offset : 43
	///     Force Touch Sense Trace #43
	U1 bForceSen43;                                            
	/// 45. Abbrev: FORCE_SEN44; Default: 0xFF(-); DF Addr: 01ACh; SRAM Offset: 0189h ; SubClass Offset : 44
	///     Force Touch Sense Trace #44
	U1 bForceSen44;                                            
	/// 46. Abbrev: FORCE_SEN45; Default: 0xFF(-); DF Addr: 01ADh; SRAM Offset: 018Ah ; SubClass Offset : 45
	///     Force Touch Sense Trace #45
	U1 bForceSen45;                                            
	/// 47. Abbrev: FORCE_SEN46; Default: 0xFF(-); DF Addr: 01AEh; SRAM Offset: 018Bh ; SubClass Offset : 46
	///     Force Touch Sense Trace #46
	U1 bForceSen46;                                            
	/// 48. Abbrev: FORCE_SEN47; Default: 0xFF(-); DF Addr: 01AFh; SRAM Offset: 018Ch ; SubClass Offset : 47
	///     Force Touch Sense Trace #47
	U1 bForceSen47;                                            
	/// 49. Abbrev: FORCE_SEN48; Default: 0xFF(-); DF Addr: 01B0h; SRAM Offset: 018Dh ; SubClass Offset : 48
	///     Force Touch Sense Trace #48
	U1 bForceSen48;                                            
	/// 50. Abbrev: FORCE_SEN49; Default: 0xFF(-); DF Addr: 01B1h; SRAM Offset: 018Eh ; SubClass Offset : 49
	///     Force Touch Sense Trace #49
	U1 bForceSen49;                                            
	/// 51. Abbrev: FORCE_SEN50; Default: 0xFF(-); DF Addr: 01B2h; SRAM Offset: 018Fh ; SubClass Offset : 50
	///     Force Touch Sense Trace #50
	U1 bForceSen50;                                            
	/// 52. Abbrev: FORCE_SEN51; Default: 0xFF(-); DF Addr: 01B3h; SRAM Offset: 0190h ; SubClass Offset : 51
	///     Force Touch Sense Trace #51
	U1 bForceSen51;                                            
} SubClassForceSenseAxisType;	///< Total = 52 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #18 => ForceDriveAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassForceDriveAxisSt
{
	///  1. Abbrev: FORCE_DRI00; Default: 0x9F(-); DF Addr: 01B4h; SRAM Offset: 0191h ; SubClass Offset : 0
	///     Force Touch Drive Trace#0
	U1 bForceDri00;                                            
	///  2. Abbrev: FORCE_DRI01; Default: 0xA0(-); DF Addr: 01B5h; SRAM Offset: 0192h ; SubClass Offset : 1
	///     Force Touch Drive Trace#1
	U1 bForceDri01;                                            
	///  3. Abbrev: FORCE_DRI02; Default: 0xA1(-); DF Addr: 01B6h; SRAM Offset: 0193h ; SubClass Offset : 2
	///     Force Touch Drive Trace#2
	U1 bForceDri02;                                            
	///  4. Abbrev: FORCE_DRI03; Default: 0xA2(-); DF Addr: 01B7h; SRAM Offset: 0194h ; SubClass Offset : 3
	///     Force Touch Drive Trace#3
	U1 bForceDri03;                                            
	///  5. Abbrev: FORCE_DRI04; Default: 0xA3(-); DF Addr: 01B8h; SRAM Offset: 0195h ; SubClass Offset : 4
	///     Force Touch Drive Trace#4
	U1 bForceDri04;                                            
	///  6. Abbrev: FORCE_DRI05; Default: 0xA4(-); DF Addr: 01B9h; SRAM Offset: 0196h ; SubClass Offset : 5
	///     Force Touch Drive Trace#5
	U1 bForceDri05;                                            
	///  7. Abbrev: FORCE_DRI06; Default: 0xA5(-); DF Addr: 01BAh; SRAM Offset: 0197h ; SubClass Offset : 6
	///     Force Touch Drive Trace#6
	U1 bForceDri06;                                            
	///  8. Abbrev: FORCE_DRI07; Default: 0xA6(-); DF Addr: 01BBh; SRAM Offset: 0198h ; SubClass Offset : 7
	///     Force Touch Drive Trace#6
	U1 bForceDri07;                                            
	///  9. Abbrev: FORCE_DRI08; Default: 0xA7(-); DF Addr: 01BCh; SRAM Offset: 0199h ; SubClass Offset : 8
	///     Force Touch Drive Trace#8
	U1 bForceDri08;                                            
	/// 10. Abbrev: FORCE_DRI09; Default: 0xA8(-); DF Addr: 01BDh; SRAM Offset: 019Ah ; SubClass Offset : 9
	///     Force Touch Drive Trace#9
	U1 bForceDri09;                                            
	/// 11. Abbrev: FORCE_DRI10; Default: 0xA9(-); DF Addr: 01BEh; SRAM Offset: 019Bh ; SubClass Offset : 10
	///     Force Touch Drive Trace#10
	U1 bForceDri10;                                            
	/// 12. Abbrev: FORCE_DRI11; Default: 0xAA(-); DF Addr: 01BFh; SRAM Offset: 019Ch ; SubClass Offset : 11
	///     Force Touch Drive Trace#11
	U1 bForceDri11;                                            
	/// 13. Abbrev: FORCE_DRI12; Default: 0xAB(-); DF Addr: 01C0h; SRAM Offset: 019Dh ; SubClass Offset : 12
	///     Force Touch Drive Trace#12
	U1 bForceDri12;                                            
	/// 14. Abbrev: FORCE_DRI13; Default: 0xAC(-); DF Addr: 01C1h; SRAM Offset: 019Eh ; SubClass Offset : 13
	///     Force Touch Drive Trace#13
	U1 bForceDri13;                                            
	/// 15. Abbrev: FORCE_DRI14; Default: 0xAD(-); DF Addr: 01C2h; SRAM Offset: 019Fh ; SubClass Offset : 14
	///     Force Touch Drive Trace#14
	U1 bForceDri14;                                            
	/// 16. Abbrev: FORCE_DRI15; Default: 0xAE(-); DF Addr: 01C3h; SRAM Offset: 01A0h ; SubClass Offset : 15
	///     Force Touch Drive Trace#15
	U1 bForceDri15;                                            
	/// 17. Abbrev: FORCE_DRI16; Default: 0xAF(-); DF Addr: 01C4h; SRAM Offset: 01A1h ; SubClass Offset : 16
	///     Force Touch Drive Trace#16
	U1 bForceDri16;                                            
	/// 18. Abbrev: FORCE_DRI17; Default: 0xFF(-); DF Addr: 01C5h; SRAM Offset: 01A2h ; SubClass Offset : 17
	///     Force Touch Drive Trace#17
	U1 bForceDri17;                                            
	/// 19. Abbrev: FORCE_DRI18; Default: 0xFF(-); DF Addr: 01C6h; SRAM Offset: 01A3h ; SubClass Offset : 18
	///     Force Touch Drive Trace#18
	U1 bForceDri18;                                            
	/// 20. Abbrev: FORCE_DRI19; Default: 0xFF(-); DF Addr: 01C7h; SRAM Offset: 01A4h ; SubClass Offset : 19
	///     Force Touch Drive Trace#19
	U1 bForceDri19;                                            
	/// 21. Abbrev: FORCE_DRI20; Default: 0xFF(-); DF Addr: 01C8h; SRAM Offset: 01A5h ; SubClass Offset : 20
	///     Force Touch Drive Trace#20
	U1 bForceDri20;                                            
	/// 22. Abbrev: FORCE_DRI21; Default: 0xFF(-); DF Addr: 01C9h; SRAM Offset: 01A6h ; SubClass Offset : 21
	///     Force Touch Drive Trace#21
	U1 bForceDri21;                                            
	/// 23. Abbrev: FORCE_DRI22; Default: 0xFF(-); DF Addr: 01CAh; SRAM Offset: 01A7h ; SubClass Offset : 22
	///     Force Touch Drive Trace#22
	U1 bForceDri22;                                            
	/// 24. Abbrev: FORCE_DRI23; Default: 0xFF(-); DF Addr: 01CBh; SRAM Offset: 01A8h ; SubClass Offset : 23
	///     Force Touch Drive Trace#23
	U1 bForceDri23;                                            
	/// 25. Abbrev: FORCE_DRI24; Default: 0xFF(-); DF Addr: 01CCh; SRAM Offset: 01A9h ; SubClass Offset : 24
	///     Force Touch Drive Trace#24
	U1 bForceDri24;                                            
	/// 26. Abbrev: FORCE_DRI25; Default: 0xFF(-); DF Addr: 01CDh; SRAM Offset: 01AAh ; SubClass Offset : 25
	///     Force Touch Drive Trace#25
	U1 bForceDri25;                                            
	/// 27. Abbrev: FORCE_DRI26; Default: 0xFF(-); DF Addr: 01CEh; SRAM Offset: 01ABh ; SubClass Offset : 26
	///     Force Touch Drive Trace#26
	U1 bForceDri26;                                            
	/// 28. Abbrev: FORCE_DRI27; Default: 0xFF(-); DF Addr: 01CFh; SRAM Offset: 01ACh ; SubClass Offset : 27
	///     Force Touch Drive Trace#27
	U1 bForceDri27;                                            
	/// 29. Abbrev: FORCE_DRI28; Default: 0xFF(-); DF Addr: 01D0h; SRAM Offset: 01ADh ; SubClass Offset : 28
	///     Force Touch Drive Trace#28
	U1 bForceDri28;                                            
	/// 30. Abbrev: FORCE_DRI29; Default: 0xFF(-); DF Addr: 01D1h; SRAM Offset: 01AEh ; SubClass Offset : 29
	///     Force Touch Drive Trace#29
	U1 bForceDri29;                                            
	/// 31. Abbrev: FORCE_DRI30; Default: 0xFF(-); DF Addr: 01D2h; SRAM Offset: 01AFh ; SubClass Offset : 30
	///     Force Touch Drive Trace#30
	U1 bForceDri30;                                            
	/// 32. Abbrev: FORCE_DRI31; Default: 0xFF(-); DF Addr: 01D3h; SRAM Offset: 01B0h ; SubClass Offset : 31
	///     Force Touch Drive Trace#31
	U1 bForceDri31;                                            
	/// 33. Abbrev: FORCE_DRI32; Default: 0xFF(-); DF Addr: 01D4h; SRAM Offset: 01B1h ; SubClass Offset : 32
	///     Force Touch Drive Trace#32
	U1 bForceDri32;                                            
	/// 34. Abbrev: FORCE_DRI33; Default: 0xFF(-); DF Addr: 01D5h; SRAM Offset: 01B2h ; SubClass Offset : 33
	///     Force Touch Drive Trace#33
	U1 bForceDri33;                                            
	/// 35. Abbrev: FORCE_DRI34; Default: 0xFF(-); DF Addr: 01D6h; SRAM Offset: 01B3h ; SubClass Offset : 34
	///     Force Touch Drive Trace#34
	U1 bForceDri34;                                            
	/// 36. Abbrev: FORCE_DRI35; Default: 0xFF(-); DF Addr: 01D7h; SRAM Offset: 01B4h ; SubClass Offset : 35
	///     Force Touch Drive Trace#35
	U1 bForceDri35;                                            
	/// 37. Abbrev: FORCE_DRI36; Default: 0xFF(-); DF Addr: 01D8h; SRAM Offset: 01B5h ; SubClass Offset : 36
	///     Force Touch Drive Trace#36
	U1 bForceDri36;                                            
	/// 38. Abbrev: FORCE_DRI37; Default: 0xFF(-); DF Addr: 01D9h; SRAM Offset: 01B6h ; SubClass Offset : 37
	///     Force Touch Drive Trace#37
	U1 bForceDri37;                                            
	/// 39. Abbrev: FORCE_DRI38; Default: 0xFF(-); DF Addr: 01DAh; SRAM Offset: 01B7h ; SubClass Offset : 38
	///     Force Touch Drive Trace#38
	U1 bForceDri38;                                            
	/// 40. Abbrev: FORCE_DRI39; Default: 0xFF(-); DF Addr: 01DBh; SRAM Offset: 01B8h ; SubClass Offset : 39
	///     Force Touch Drive Trace#39
	U1 bForceDri39;                                            
	/// 41. Abbrev: FORCE_DRI40; Default: 0xFF(-); DF Addr: 01DCh; SRAM Offset: 01B9h ; SubClass Offset : 40
	///     Force Touch Drive Trace#40
	U1 bForceDri40;                                            
	/// 42. Abbrev: FORCE_DRI41; Default: 0xFF(-); DF Addr: 01DDh; SRAM Offset: 01BAh ; SubClass Offset : 41
	///     Force Touch Drive Trace#41
	U1 bForceDri41;                                            
	/// 43. Abbrev: FORCE_DRI42; Default: 0xFF(-); DF Addr: 01DEh; SRAM Offset: 01BBh ; SubClass Offset : 42
	///     Force Touch Drive Trace#42
	U1 bForceDri42;                                            
	/// 44. Abbrev: FORCE_DRI43; Default: 0xFF(-); DF Addr: 01DFh; SRAM Offset: 01BCh ; SubClass Offset : 43
	///     Force Touch Drive Trace#43
	U1 bForceDri43;                                            
	/// 45. Abbrev: FORCE_DRI44; Default: 0xFF(-); DF Addr: 01E0h; SRAM Offset: 01BDh ; SubClass Offset : 44
	///     Force Touch Drive Trace#44
	U1 bForceDri44;                                            
	/// 46. Abbrev: FORCE_DRI45; Default: 0xFF(-); DF Addr: 01E1h; SRAM Offset: 01BEh ; SubClass Offset : 45
	///     Force Touch Drive Trace#45
	U1 bForceDri45;                                            
	/// 47. Abbrev: FORCE_DRI46; Default: 0xFF(-); DF Addr: 01E2h; SRAM Offset: 01BFh ; SubClass Offset : 46
	///     Force Touch Drive Trace#46
	U1 bForceDri46;                                            
	/// 48. Abbrev: FORCE_DRI47; Default: 0xFF(-); DF Addr: 01E3h; SRAM Offset: 01C0h ; SubClass Offset : 47
	///     Force Touch Drive Trace#47
	U1 bForceDri47;                                            
	/// 49. Abbrev: FORCE_DRI48; Default: 0xFF(-); DF Addr: 01E4h; SRAM Offset: 01C1h ; SubClass Offset : 48
	///     Force Touch Drive Trace#48
	U1 bForceDri48;                                            
	/// 50. Abbrev: FORCE_DRI49; Default: 0xFF(-); DF Addr: 01E5h; SRAM Offset: 01C2h ; SubClass Offset : 49
	///     Force Touch Drive Trace#49
	U1 bForceDri49;                                            
	/// 51. Abbrev: FORCE_DRI50; Default: 0xFF(-); DF Addr: 01E6h; SRAM Offset: 01C3h ; SubClass Offset : 50
	///     Force Touch Drive Trace#50
	U1 bForceDri50;                                            
	/// 52. Abbrev: FORCE_DRI51; Default: 0xFF(-); DF Addr: 01E7h; SRAM Offset: 01C4h ; SubClass Offset : 51
	///     Force Touch Drive Trace#51
	U1 bForceDri51;                                            
} SubClassForceDriveAxisType;	///< Total = 52 Bytes

/// =============================================================///
/// Data flash Class #11 => ForceTraceSetup
/// =============================================================///
typedef struct PACK ClassForceTraceSetupSt
{
	SubClassForceSenseAxisType scForceSenseAxis;
	SubClassForceDriveAxisType scForceDriveAxis;
}ClassForceTraceSetupType;	///< Total = 104 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #19 => DataCtrl
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDataCtrlSt
{
	///  1. Abbrev: TRAN_TYPE; Default: 0x00(-); DF Addr: 01E8h; SRAM Offset: 01C5h ; SubClass Offset : 0
	///     Transfer type
	U1 bTranType;                                              
	///  2. Abbrev: FINGER_NUM; Default: 0x05(-); DF Addr: 01E9h; SRAM Offset: 01C6h ; SubClass Offset : 1
	///     Finger Numbers
	U1 bFingerNum;                                             
	///  3. Abbrev: I2C_PIPE_EN; Default: 0x00(-); DF Addr: 01EAh; SRAM Offset: 01C7h ; SubClass Offset : 2
	///     I2C pipeline enable
	U1 bI2CPipeEn;                                             
	///  4. Abbrev: I2C_INT_LOW_TIMEOUT_CNT; Default: 0x64(-); DF Addr: 01EBh; SRAM Offset: 01C8h ; SubClass Offset : 3
	///     INT low wait timing Counter
	U1 bI2CIntLowTimeOutCnt;                                   
	///  5. Abbrev: COOR_EXCHANGE; Default: 0x00(-); DF Addr: 01ECh; SRAM Offset: 01C9h ; SubClass Offset : 4
	///     [FIX]X axis and Y axis exchange enable/disable
	U1 bXYCoorExchange;                                        
	///  6. Abbrev: COOR_XY_ORGIN; Default: 0x01(-); DF Addr: 01EDh; SRAM Offset: 01CAh ; SubClass Offset : 5
	///     [FIX]X axis and Y init coordination
	U1 bXYCoorOrigin;                                          
	///  7. Abbrev: TOUCH_PANEL_TYPE; Default: 0x00(-); DF Addr: 01EEh; SRAM Offset: 01CBh ; SubClass Offset : 6
	///     [FIX]X axis and Y init coordination
	U1 bTPType;                                                
	///  8. Abbrev: TOUCH_PANEL_TYPE_SUB; Default: 0x00(-); DF Addr: 01EFh; SRAM Offset: 01CCh ; SubClass Offset : 7
	///     
	U1 bTPTypeSub;                                             
	///  9. Abbrev: RESET_DEFAULT_PROTOCOL_VERDOR; Default: 0x02(-); DF Addr: 01F0h; SRAM Offset: 01CDh ; SubClass Offset : 8
	///     
	U1 bDefaultProtocolVendor;                                 
	/// 10. Abbrev: HTYPE_MID_CHANNEL; Default: 0x0E(-); DF Addr: 01F1h; SRAM Offset: 01CEh ; SubClass Offset : 9
	///     Htype middle channel
	U1 bHTypeMidChannel;                                       
	/// 11. Abbrev: MTK_FORMAT_EN; Default: 0x00(-); DF Addr: 01F2h; SRAM Offset: 01CFh ; SubClass Offset : 10
	///     MTK format enable (Abnormal)
	U1 bMTKFormatEnable;                                       
	/// 12. Abbrev: FINGER_UP_RP_CNT; Default: 0x01(-); DF Addr: 01F3h; SRAM Offset: 01D0h ; SubClass Offset : 11
	///     Finger Up Report Zero Count. If No Finger ,keep report N zero finger
	U1 bFingerUpReportCnt;                                     
	/// 13. Abbrev: RESET_RPT_CNT; Default: 0x01(-); DF Addr: 01F4h; SRAM Offset: 01D1h ; SubClass Offset : 12
	///     Dummy INT Low counts after reset to avoid entering Dataflash Mode
	U1 bResetReportCnt;                                        
	/// 14. Abbrev: RESET_RPT_LOW_PERIOD; Default: 0xC8(-); DF Addr: 01F5h; SRAM Offset: 01D2h ; SubClass Offset : 13
	///     Dummy INT Low period after reset to avoid entering Dataflash Mode
	U1 bResetReportLowPeriod;                                  
} SubClassDataCtrlType;	///< Total = 14 Bytes

/// =============================================================///
/// Data flash Class #12 => FormatCtrl
/// =============================================================///
typedef struct PACK ClassFormatCtrlSt
{
	SubClassDataCtrlType scDataCtrl;
}ClassFormatCtrlType;	///< Total = 14 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #20 => KeyAnalog
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassKeyAnalogSt
{
	///  1. Abbrev: ADC_CTRL0_KEY; Default: 0x00(-); DF Addr: 0200h; SRAM Offset: 01D3h ; SubClass Offset : 0
	///     ADC_CTRL_0 Key
	U1 bKeyADCCtrl0;                                           
	///  2. Abbrev: REG_AD_CTRL1_KEY; Default: 0x20(-); DF Addr: 0201h; SRAM Offset: 01D4h ; SubClass Offset : 1
	///     ADC_CTRL_1 Reg[0x001B8004]
	U1 bKeyADCCtrl1;                                           
	///  3. Abbrev: REG32_TX_CTRL_KEY; Default: 0x1100(-); DF Addr: 0202h; SRAM Offset: 01D5h ; SubClass Offset : 2
	///     \n1.TX_CTRL[14:12]=TX_CTRL[10:8] \n2.charger pump off 時,只能設成0~1 \n3.charger pump on, 0~7
	U2 wKeyTxCtrl;                                             
	///  4. Abbrev: REG_PN_CTRL_KEY; Default: 0x04(-); DF Addr: 0204h; SRAM Offset: 01D7h ; SubClass Offset : 4
	///     Reg[0x00192000]
	U1 bKeyPnCtrl;                                             
	///  5. Abbrev: REG_REK_TH_KEY; Default: 0x96(-); DF Addr: 0205h; SRAM Offset: 01D8h ; SubClass Offset : 5
	///     Key rek threshold
	U1 bKeyRekTH;                                              
} SubClassKeyAnalogType;	///< Total = 6 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #21 => KeyDriveTrace
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassKeyDriveTraceSt
{
	///  1. Abbrev: KEY_SUPPORT; Default: 0x00(-); DF Addr: 0206h; SRAM Offset: 01D9h ; SubClass Offset : 0
	///     Key function support, 0: Disable, 1 : Enable
	U1 bKeySupport;                                            
	///  2. Abbrev: KEY_DRI_TRACE; Default: 0xFF(-); DF Addr: 0207h; SRAM Offset: 01DAh ; SubClass Offset : 1
	///     Key drive trace
	U1 bKeyDriTrace;                                           
} SubClassKeyDriveTraceType;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #22 => KeySenseTrace
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassKeySenseTraceSt
{
	///  1. Abbrev: KEY_SEN00; Default: 0xFF(-); DF Addr: 0208h; SRAM Offset: 01DBh ; SubClass Offset : 0
	///     Key drive Sense Trace 0
	U1 bKeySen0;                                               
	///  2. Abbrev: KEY_SEN01; Default: 0xFF(-); DF Addr: 0209h; SRAM Offset: 01DCh ; SubClass Offset : 1
	///     Key drive Sense Trace 1
	U1 bKeySen1;                                               
	///  3. Abbrev: KEY_SEN02; Default: 0xFF(-); DF Addr: 020Ah; SRAM Offset: 01DDh ; SubClass Offset : 2
	///     Key drive Sense Trace 2
	U1 bKeySen2;                                               
	///  4. Abbrev: KEY_SEN03; Default: 0xFF(-); DF Addr: 020Bh; SRAM Offset: 01DEh ; SubClass Offset : 3
	///     Key drive Sense Trace 3
	U1 bKeySen3;                                               
	///  5. Abbrev: KEY_SEN04; Default: 0xFF(-); DF Addr: 020Ch; SRAM Offset: 01DFh ; SubClass Offset : 4
	///     Key drive Sense Trace 4
	U1 bKeySen4;                                               
	///  6. Abbrev: KEY_SEN05; Default: 0xFF(-); DF Addr: 020Dh; SRAM Offset: 01E0h ; SubClass Offset : 5
	///     Key drive Sense Trace 5
	U1 bKeySen5;                                               
	///  7. Abbrev: KEY_SEN06; Default: 0xFF(-); DF Addr: 020Eh; SRAM Offset: 01E1h ; SubClass Offset : 6
	///     Key drive Sense Trace 6
	U1 bKeySen6;                                               
	///  8. Abbrev: KEY_SEN07; Default: 0xFF(-); DF Addr: 020Fh; SRAM Offset: 01E2h ; SubClass Offset : 7
	///     Key drive Sense Trace 7
	U1 bKeySen7;                                               
} SubClassKeySenseTraceType;	///< Total = 8 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #23 => AlgoKeyTh
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoKeyThSt
{
	///  1. Abbrev: KEY_TH1; Default: 0x50(-); DF Addr: 0210h; SRAM Offset: 01E3h ; SubClass Offset : 0
	///     [DYN] Key  Threshold #1
	U1 bKeyTh1;                                                
	///  2. Abbrev: KEY_TH2; Default: 0x50(-); DF Addr: 0211h; SRAM Offset: 01E4h ; SubClass Offset : 1
	///     [DYN] Key  Threshold #2
	U1 bKeyTh2;                                                
	///  3. Abbrev: KEY_TH3; Default: 0x50(-); DF Addr: 0212h; SRAM Offset: 01E5h ; SubClass Offset : 2
	///     [DYN] Key  Threshold #3
	U1 bKeyTh3;                                                
	///  4. Abbrev: KEY_TH4; Default: 0x50(-); DF Addr: 0213h; SRAM Offset: 01E6h ; SubClass Offset : 3
	///     [DYN] Key  Threshold #4
	U1 bKeyTh4;                                                
	///  5. Abbrev: KEY_TH5; Default: 0x50(-); DF Addr: 0214h; SRAM Offset: 01E7h ; SubClass Offset : 4
	///     [DYN] Key  Threshold #5
	U1 bKeyTh5;                                                
	///  6. Abbrev: KEY_TH6; Default: 0x50(-); DF Addr: 0215h; SRAM Offset: 01E8h ; SubClass Offset : 5
	///     [DYN] Key  Threshold #6
	U1 bKeyTh6;                                                
	///  7. Abbrev: KEY_TH7; Default: 0x50(-); DF Addr: 0216h; SRAM Offset: 01E9h ; SubClass Offset : 6
	///     [DYN] Key  Threshold #7
	U1 bKeyTh7;                                                
	///  8. Abbrev: KEY_TH8; Default: 0x50(-); DF Addr: 0217h; SRAM Offset: 01EAh ; SubClass Offset : 7
	///     [DYN] Key  Threshold #8
	U1 bKeyTh8;                                                
	///  9. Abbrev: KEY_DEBOOUNCE_CNT; Default: 0x00(-); DF Addr: 0218h; SRAM Offset: 01EBh ; SubClass Offset : 8
	///     [DYN] Key  debounce
	U1 bKeyDebounceCnt;                                        
	/// 10. Abbrev: KEY_RELEASE_CNT; Default: 0x00(-); DF Addr: 0219h; SRAM Offset: 01ECh ; SubClass Offset : 9
	///     [DYN] Key Release Lock Status Cnt
	U1 bKeyReleaseCnt;                                         
	/// 11. Abbrev: KEY_MIN_DELTA_DEV; Default: 0x00(-); DF Addr: 021Ah; SRAM Offset: 01EDh ; SubClass Offset : 10
	///     The delta value of 1st max key dev and the 2nd max key shall be bigger than this value then the key is recognized
	U1 bKeyMinDeltaDev;                                        
} SubClassAlgoKeyThType;	///< Total = 11 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #24 => VirtualKey
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassVirtualKeySt
{
	///  1. Abbrev: VKEY_EN; Default: 0x00(-); DF Addr: 021Bh; SRAM Offset: 01EEh ; SubClass Offset : 0
	///     Virtual Key Enable : 0, disable ; 1: Enable
	U1 bVKeyEnable;                                            
	///  2. Abbrev: VKEY0_X; Default: 0x0000(-); DF Addr: 021Ch; SRAM Offset: 01EFh ; SubClass Offset : 1
	///     Virtual Key0 (Search) : X (Left)
	U2 wKey0X;                                                 
	///  3. Abbrev: VKEY0_Y; Default: 0x0000(-); DF Addr: 021Eh; SRAM Offset: 01F1h ; SubClass Offset : 3
	///     Virtual Key0 (Search) : Y (Top)
	U2 wKey0Y;                                                 
	///  4. Abbrev: VKEY0_W; Default: 0x00(-); DF Addr: 0220h; SRAM Offset: 01F3h ; SubClass Offset : 5
	///     Virtual Key0 (Search) : W (Width)
	U1 bKey0W;                                                 
	///  5. Abbrev: VKEY0_H; Default: 0x00(-); DF Addr: 0221h; SRAM Offset: 01F4h ; SubClass Offset : 6
	///     Virtual Key0 (Search) : H (Height)
	U1 bKey0H;                                                 
	///  6. Abbrev: VKEY1_X; Default: 0x0000(-); DF Addr: 0222h; SRAM Offset: 01F5h ; SubClass Offset : 7
	///     Virtual Key1 (Back) : X (Left)
	U2 wKey1X;                                                 
	///  7. Abbrev: VKEY1_Y; Default: 0x0000(-); DF Addr: 0224h; SRAM Offset: 01F7h ; SubClass Offset : 9
	///     Virtual Key1 (Back) : Y (Top)
	U2 wKey1Y;                                                 
	///  8. Abbrev: VKEY1_W; Default: 0x00(-); DF Addr: 0226h; SRAM Offset: 01F9h ; SubClass Offset : 11
	///     Virtual Key1 (Back) : W (Width)
	U1 bKey1W;                                                 
	///  9. Abbrev: VKEY1_H; Default: 0x00(-); DF Addr: 0227h; SRAM Offset: 01FAh ; SubClass Offset : 12
	///     Virtual Key1 (Back) : H (Height)
	U1 bKey1H;                                                 
	/// 10. Abbrev: VKEY2_X; Default: 0x0000(-); DF Addr: 0228h; SRAM Offset: 01FBh ; SubClass Offset : 13
	///     Virtual Key2 (Home) : X (Left)
	U2 wKey2X;                                                 
	/// 11. Abbrev: VKEY2_Y; Default: 0x0000(-); DF Addr: 022Ah; SRAM Offset: 01FDh ; SubClass Offset : 15
	///     Virtual Key2 (Home) : Y (Top)
	U2 wKey2Y;                                                 
	/// 12. Abbrev: VKEY2_W; Default: 0x00(-); DF Addr: 022Ch; SRAM Offset: 01FFh ; SubClass Offset : 17
	///     Virtual Key2 (Home) : W (Width)
	U1 bKey2W;                                                 
	/// 13. Abbrev: VKEY2_H; Default: 0x00(-); DF Addr: 022Dh; SRAM Offset: 0200h ; SubClass Offset : 18
	///     Virtual Key2 (Home) : H (Height)
	U1 bKey2H;                                                 
	/// 14. Abbrev: VKEY3_X; Default: 0x0000(-); DF Addr: 022Eh; SRAM Offset: 0201h ; SubClass Offset : 19
	///     Virtual Key3 (Menu) : X (Lef)
	U2 wKey3X;                                                 
	/// 15. Abbrev: VKEY3_Y; Default: 0x0000(-); DF Addr: 0230h; SRAM Offset: 0203h ; SubClass Offset : 21
	///     Virtual Key3 (Menu) : Y (Top)
	U2 wKey3Y;                                                 
	/// 16. Abbrev: VKEY3_W; Default: 0x00(-); DF Addr: 0232h; SRAM Offset: 0205h ; SubClass Offset : 23
	///     Virtual Key3 (Menu) : W (Width)
	U1 bKey3W;                                                 
	/// 17. Abbrev: VKEY3_H; Default: 0x00(-); DF Addr: 0233h; SRAM Offset: 0206h ; SubClass Offset : 24
	///     Virtual Key3 (Menu) : H (Height)
	U1 bKey3H;                                                 
	/// 18. Abbrev: VKEY_BORDER; Default: 0x00(-); DF Addr: 0234h; SRAM Offset: 0207h ; SubClass Offset : 25
	///     Virtual Key Border : If the y point close to the border, treat it as border
	U1 bVKeyBorder;                                            
	/// 19. Abbrev: VKEY_DEBOUNCE_CNT; Default: 0x00(-); DF Addr: 0235h; SRAM Offset: 0208h ; SubClass Offset : 26
	///     Virtual Key: Deboucned Count
	U1 bVKeyDebounce;                                          
} SubClassVirtualKeyType;	///< Total = 27 Bytes

/// =============================================================///
/// Data flash Class #13 => KeyCtrl
/// =============================================================///
typedef struct PACK ClassKeyCtrlSt
{
	SubClassKeyAnalogType scKeyAnalog;
	SubClassKeyDriveTraceType scKeyDriveTrace;
	SubClassKeySenseTraceType scKeySenseTrace;
	SubClassAlgoKeyThType scAlgoKeyTh;
	SubClassVirtualKeyType scVirtualKey;
}ClassKeyCtrlType;	///< Total = 54 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #25 => SystenCtel
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassSystenCtelSt
{
	///  1. Abbrev: WDT_EN; Default: 0x01(-); DF Addr: 0236h; SRAM Offset: 0209h ; SubClass Offset : 0
	///     Enable Watch Dog Timer
	U1 bWatchDogTimerEnable;                                   
} SubClassSystenCtelType;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #14 => FuncCtrl
/// =============================================================///
typedef struct PACK ClassFuncCtrlSt
{
	SubClassSystenCtelType scSystemCtrl;
}ClassFuncCtrlType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #26 => FreqSetup
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassFreqSetupSt
{
	///  1. Abbrev: FREQ_HOP_REBOOT_NUM; Default: 0x40(-); DF Addr: 0237h; SRAM Offset: 020Ah ; SubClass Offset : 0
	///     Freq Hop times while Reset  0:Disable ; 1:1-time , 2:2-time , ... N-N-time
	U1 bFreqHopTimesWhenReset;                                 
	///  2. Abbrev: FREQ_HOP_FREQ_MAX; Default: 0x0158(-); DF Addr: 0238h; SRAM Offset: 020Bh ; SubClass Offset : 1
	///     
	U2 bFreqHopMax;                                            
	///  3. Abbrev: FREQ_HOP_FREQ_MIN; Default: 0x0072(-); DF Addr: 023Ah; SRAM Offset: 020Dh ; SubClass Offset : 3
	///     
	U2 bFreqHopMin;                                            
} SubClassFreqSetupType;	///< Total = 5 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #27 => DynamicHop
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDynamicHopSt
{
	///  1. Abbrev: DYNAMIC_HOP_SCAN_NUM; Default: 0x40(-); DF Addr: 023Ch; SRAM Offset: 020Fh ; SubClass Offset : 0
	///     
	U1 bDynamicScanSampleNum;                                  
	///  2. Abbrev: DYNAMIC_HOP_RESET_ROUND; Default: 0x08(-); DF Addr: 023Dh; SRAM Offset: 0210h ; SubClass Offset : 1
	///     
	U1 bDynamicHopResetRound;                                  
	///  3. Abbrev: DYNAMIC_HOP_DEV_SUM_TH; Default: 0x0032(-); DF Addr: 023Eh; SRAM Offset: 0211h ; SubClass Offset : 2
	///     
	U2 wDynamicHopDevSumTH;                                    
	///  4. Abbrev: DYNAMIC_HOP_DEV_ERROR; Default: 0x05(-); DF Addr: 0240h; SRAM Offset: 0213h ; SubClass Offset : 4
	///     
	U1 bDynamicHopDevErr;                                      
	///  5. Abbrev: DYNAMIC_HOP_DEV_TH_QUICK; Default: 0x0064(-); DF Addr: 0241h; SRAM Offset: 0214h ; SubClass Offset : 5
	///     
	U2 wDynamicHopDevQuickTH;                                  
	///  6. Abbrev: DYNAMIC_HOP_DEV_TH_QUICK_CNT; Default: 0x01(-); DF Addr: 0243h; SRAM Offset: 0216h ; SubClass Offset : 7
	///     
	U1 bDynamicHopDevQuickCnt;                                 
} SubClassDynamicHopType;	///< Total = 8 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #28 => FreqHop
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassFreqHopSt
{
	///  1. Abbrev: FREQ_HOP_TONE_ROUND; Default: 0x01(-); DF Addr: 0244h; SRAM Offset: 0217h ; SubClass Offset : 0
	///     the ND scan round(Single Tone=1, Dual Tone=1/2)
	U1 bFreqHopToneRound;                                      
	///  2. Abbrev: FREQ_HOP_CURRENT_ID; Default: 0x05(-); DF Addr: 0245h; SRAM Offset: 0218h ; SubClass Offset : 1
	///     
	U1 bFreqHopCurrentID;                                      
	///  3. Abbrev: FREQ_HOP_SET_NUM; Default: 0x0A(-); DF Addr: 0246h; SRAM Offset: 0219h ; SubClass Offset : 2
	///     
	U1 bFreqHopSetNum;                                         
	///  4. Abbrev: FREQ_HOP_MUL_ND1; Default: 0x02(-); DF Addr: 0247h; SRAM Offset: 021Ah ; SubClass Offset : 3
	///     
	U1 bFreqMulND1;                                            
	///  5. Abbrev: FREQ_HOP_MUL_DIFF1; Default: 0x01(-); DF Addr: 0248h; SRAM Offset: 021Bh ; SubClass Offset : 4
	///     
	U1 bFreqMulDiff1;                                          
	///  6. Abbrev: FREQ_HOP_NUM1; Default: 0x0A(-); DF Addr: 0249h; SRAM Offset: 021Ch ; SubClass Offset : 5
	///     
	U1 bFreqNum1;                                              
	///  7. Abbrev: FREQ_HOP_MUL_ND2; Default: 0x02(-); DF Addr: 024Ah; SRAM Offset: 021Dh ; SubClass Offset : 6
	///     
	U1 bFreqMulND2;                                            
	///  8. Abbrev: FREQ_HOP_MUL_DIFF2; Default: 0x01(-); DF Addr: 024Bh; SRAM Offset: 021Eh ; SubClass Offset : 7
	///     
	U1 bFreqMulDiff2;                                          
	///  9. Abbrev: FREQ_HOP_NUM2; Default: 0x00(-); DF Addr: 024Ch; SRAM Offset: 021Fh ; SubClass Offset : 8
	///     
	U1 bFreqNum2;                                              
} SubClassFreqHopType;	///< Total = 9 Bytes

/// =============================================================///
/// Data flash Class #15 => SYSFreqHop
/// =============================================================///
typedef struct PACK ClassSYSFreqHopSt
{
	SubClassFreqSetupType scFreqSetup;
	SubClassDynamicHopType scDynamicHop;
	SubClassFreqHopType scFreqHop;
}ClassSYSFreqHopType;	///< Total = 22 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #29 => NormalState
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNormalStateSt
{
	///  1. Abbrev: CLUSTER_WEIGHT_NORMAL; Default: 0x0096(-); DF Addr: 024Dh; SRAM Offset: 0220h ; SubClass Offset : 0
	///     [DYN] Cluster weight (Normal Mode)
	U2 wClusterWeightNormal;                                   
	///  2. Abbrev: CLUSTER_DEV_MAX; Default: 0x0046(-); DF Addr: 024Fh; SRAM Offset: 0222h ; SubClass Offset : 2
	///     [DYN] Cluster weight (Normal Mode)
	U2 wClusterDevMax;                                         
	///  3. Abbrev: CLUSTER_DEV_MIN; Default: 0x0032(-); DF Addr: 0251h; SRAM Offset: 0224h ; SubClass Offset : 4
	///     [DYN] Cluster weight (Normal Mode)
	U2 wClusterDevMin;                                         
	///  4. Abbrev: CLUSTER_WEIGHT_AVG_RATO; Default: 0x09(-); DF Addr: 0253h; SRAM Offset: 0226h ; SubClass Offset : 6
	///     
	U1 bClusterWeightDivider;                                  
	///  5. Abbrev: CLUSTER_PEAK_VELLY_DIVD_RATIO; Default: 0x41(-); DF Addr: 0254h; SRAM Offset: 0227h ; SubClass Offset : 7
	///     
	U1 bClusterPeakVellyDivdeRatio;                            
	///  6. Abbrev: CLUSTER_PEAK_VELLY_MERG_RATIO; Default: 0x50(-); DF Addr: 0255h; SRAM Offset: 0228h ; SubClass Offset : 8
	///     
	U1 bClusterPeakVellyMergeRatio;                            
	///  7. Abbrev: CLUSTER_SIZE_DELTA_DEC_RATIO; Default: 0x46(-); DF Addr: 0256h; SRAM Offset: 0229h ; SubClass Offset : 9
	///     When cluster size Dec under Size*n% than last round , will trigger Cluster-divid-merge statmachine, Range:100%~0%
	U1 bClusterSizeChangeDecRatio;                             
	///  8. Abbrev: CLUSTER_SIZE_DELTA_INC_RATIO; Default: 0x78(-); DF Addr: 0257h; SRAM Offset: 022Ah ; SubClass Offset : 10
	///     When cluster size Inc over Size*n% than last round , will trigger Cluster-divid-merge statmachine, Range:100%~199%
	U1 bClusterSizeChangeIncRatio;                             
	///  9. Abbrev: CLUSTER_NEG_DEV_FINGER_TH; Default: 0x0064(-); DF Addr: 0258h; SRAM Offset: 022Bh ; SubClass Offset : 11
	///     
	U2 wClusterNegDevFingerTH;                                 
} SubClassNormalStateType;	///< Total = 13 Bytes

/// =============================================================///
/// Data flash Class #16 => Cluster
/// =============================================================///
typedef struct PACK ClassClusterSt
{
	SubClassNormalStateType scNormalUseState;
}ClassClusterType;	///< Total = 13 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #30 => R1toR2
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassR1toR2St
{
	///  1. Abbrev: R1TOR2_EN; Default: 0x40(-); DF Addr: 025Ah; SRAM Offset: 022Dh ; SubClass Offset : 0
	///     
	U1 bR1toR2en;                                              
	///  2. Abbrev: R1TOR2_RATIO_O2E; Default: 0x40(-); DF Addr: 025Bh; SRAM Offset: 022Eh ; SubClass Offset : 1
	///     
	U1 bR1toR2RatioOdd2Even;                                   
	///  3. Abbrev: R1TOR2_RATIO_E2O; Default: 0x40(-); DF Addr: 025Ch; SRAM Offset: 022Fh ; SubClass Offset : 2
	///     
	U1 bR1toR2RatioEven2Odd;                                   
} SubClassR1toR2Type;	///< Total = 3 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #31 => NearToFar
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNearToFarSt
{
	///  1. Abbrev: NEAR_FAR_RATIO_DEV00; Default: 0x40(-); DF Addr: 025Dh; SRAM Offset: 0230h ; SubClass Offset : 0
	///     Drive Near Far Dev[0] Ratio : Raw Dev * N/64
	U1 bNFRatio00;                                             
	///  2. Abbrev: NEAR_FAR_RATIO_DEV01; Default: 0x40(-); DF Addr: 025Eh; SRAM Offset: 0231h ; SubClass Offset : 1
	///     Drive Near Far Dev[1] Ratio : Raw Dev * N/64
	U1 bNFRatio01;                                             
	///  3. Abbrev: NEAR_FAR_RATIO_DEV02; Default: 0x40(-); DF Addr: 025Fh; SRAM Offset: 0232h ; SubClass Offset : 2
	///     Drive Near Far Dev[2] Ratio : Raw Dev * N/64
	U1 bNFRatio02;                                             
	///  4. Abbrev: NEAR_FAR_RATIO_DEV03; Default: 0x40(-); DF Addr: 0260h; SRAM Offset: 0233h ; SubClass Offset : 3
	///     Drive Near Far Dev[3] Ratio : Raw Dev * N/64
	U1 bNFRatio03;                                             
	///  5. Abbrev: NEAR_FAR_RATIO_DEV04; Default: 0x40(-); DF Addr: 0261h; SRAM Offset: 0234h ; SubClass Offset : 4
	///     Drive Near Far Dev[4] Ratio : Raw Dev * N/64
	U1 bNFRatio04;                                             
	///  6. Abbrev: NEAR_FAR_RATIO_DEV05; Default: 0x40(-); DF Addr: 0262h; SRAM Offset: 0235h ; SubClass Offset : 5
	///     Drive Near Far Dev[5] Ratio : Raw Dev * N/64
	U1 bNFRatio05;                                             
	///  7. Abbrev: NEAR_FAR_RATIO_DEV06; Default: 0x40(-); DF Addr: 0263h; SRAM Offset: 0236h ; SubClass Offset : 6
	///     Drive Near Far Dev[6] Ratio : Raw Dev * N/64
	U1 bNFRatio06;                                             
	///  8. Abbrev: NEAR_FAR_RATIO_DEV07; Default: 0x40(-); DF Addr: 0264h; SRAM Offset: 0237h ; SubClass Offset : 7
	///     Drive Near Far Dev[7] Ratio : Raw Dev * N/64
	U1 bNFRatio07;                                             
	///  9. Abbrev: NEAR_FAR_RATIO_DEV08; Default: 0x40(-); DF Addr: 0265h; SRAM Offset: 0238h ; SubClass Offset : 8
	///     Drive Near Far Dev[8] Ratio : Raw Dev * N/64
	U1 bNFRatio08;                                             
	/// 10. Abbrev: NEAR_FAR_RATIO_DEV09; Default: 0x40(-); DF Addr: 0266h; SRAM Offset: 0239h ; SubClass Offset : 9
	///     Drive Near Far Dev[9] Ratio : Raw Dev * N/64
	U1 bNFRatio09;                                             
	/// 11. Abbrev: NEAR_FAR_RATIO_DEV10; Default: 0x40(-); DF Addr: 0267h; SRAM Offset: 023Ah ; SubClass Offset : 10
	///     Drive Near Far Dev[10] Ratio : Raw Dev * N/64
	U1 bNFRatio10;                                             
	/// 12. Abbrev: NEAR_FAR_RATIO_DEV11; Default: 0x40(-); DF Addr: 0268h; SRAM Offset: 023Bh ; SubClass Offset : 11
	///     Drive Near Far Dev[11] Ratio : Raw Dev * N/64
	U1 bNFRatio11;                                             
	/// 13. Abbrev: NEAR_FAR_RATIO_DEV12; Default: 0x40(-); DF Addr: 0269h; SRAM Offset: 023Ch ; SubClass Offset : 12
	///     Drive Near Far Dev[12] Ratio : Raw Dev * N/64
	U1 bNFRatio12;                                             
	/// 14. Abbrev: NEAR_FAR_RATIO_DEV13; Default: 0x40(-); DF Addr: 026Ah; SRAM Offset: 023Dh ; SubClass Offset : 13
	///     Drive Near Far Dev[13] Ratio : Raw Dev * N/64
	U1 bNFRatio13;                                             
	/// 15. Abbrev: NEAR_FAR_RATIO_DEV14; Default: 0x40(-); DF Addr: 026Bh; SRAM Offset: 023Eh ; SubClass Offset : 14
	///     Drive Near Far Dev[14] Ratio : Raw Dev * N/64
	U1 bNFRatio14;                                             
	/// 16. Abbrev: NEAR_FAR_RATIO_DEV15; Default: 0x40(-); DF Addr: 026Ch; SRAM Offset: 023Fh ; SubClass Offset : 15
	///     Drive Near Far Dev[15] Ratio : Raw Dev * N/64
	U1 bNFRatio15;                                             
	/// 17. Abbrev: NEAR_FAR_RATIO_DEV16; Default: 0x40(-); DF Addr: 026Dh; SRAM Offset: 0240h ; SubClass Offset : 16
	///     Drive Near Far Dev[16] Ratio : Raw Dev * N/64
	U1 bNFRatio16;                                             
	/// 18. Abbrev: NEAR_FAR_RATIO_DEV17; Default: 0x40(-); DF Addr: 026Eh; SRAM Offset: 0241h ; SubClass Offset : 17
	///     Drive Near Far Dev[17] Ratio : Raw Dev * N/64
	U1 bNFRatio17;                                             
	/// 19. Abbrev: NEAR_FAR_RATIO_DEV18; Default: 0x40(-); DF Addr: 026Fh; SRAM Offset: 0242h ; SubClass Offset : 18
	///     Drive Near Far Dev[18] Ratio : Raw Dev * N/64
	U1 bNFRatio18;                                             
	/// 20. Abbrev: NEAR_FAR_RATIO_DEV19; Default: 0x40(-); DF Addr: 0270h; SRAM Offset: 0243h ; SubClass Offset : 19
	///     Drive Near Far Dev[19] Ratio : Raw Dev * N/64
	U1 bNFRatio19;                                             
	/// 21. Abbrev: NEAR_FAR_RATIO_DEV20; Default: 0x40(-); DF Addr: 0271h; SRAM Offset: 0244h ; SubClass Offset : 20
	///     Drive Near Far Dev[20] Ratio : Raw Dev * N/64
	U1 bNFRatio20;                                             
	/// 22. Abbrev: NEAR_FAR_RATIO_DEV21; Default: 0x40(-); DF Addr: 0272h; SRAM Offset: 0245h ; SubClass Offset : 21
	///     Drive Near Far Dev[21] Ratio : Raw Dev * N/64
	U1 bNFRatio21;                                             
	/// 23. Abbrev: NEAR_FAR_RATIO_DEV22; Default: 0x40(-); DF Addr: 0273h; SRAM Offset: 0246h ; SubClass Offset : 22
	///     Drive Near Far Dev[22] Ratio : Raw Dev * N/64
	U1 bNFRatio22;                                             
	/// 24. Abbrev: NEAR_FAR_RATIO_DEV23; Default: 0x40(-); DF Addr: 0274h; SRAM Offset: 0247h ; SubClass Offset : 23
	///     Drive Near Far Dev[23] Ratio : Raw Dev * N/64
	U1 bNFRatio23;                                             
	/// 25. Abbrev: NEAR_FAR_RATIO_DEV24; Default: 0x40(-); DF Addr: 0275h; SRAM Offset: 0248h ; SubClass Offset : 24
	///     Drive Near Far Dev[24] Ratio : Raw Dev * N/64
	U1 bNFRatio24;                                             
	/// 26. Abbrev: NEAR_FAR_RATIO_DEV25; Default: 0x40(-); DF Addr: 0276h; SRAM Offset: 0249h ; SubClass Offset : 25
	///     Drive Near Far Dev[25] Ratio : Raw Dev * N/64
	U1 bNFRatio25;                                             
	/// 27. Abbrev: NEAR_FAR_RATIO_DEV26; Default: 0x40(-); DF Addr: 0277h; SRAM Offset: 024Ah ; SubClass Offset : 26
	///     Drive Near Far Dev[26] Ratio : Raw Dev * N/64
	U1 bNFRatio26;                                             
	/// 28. Abbrev: NEAR_FAR_RATIO_DEV27; Default: 0x40(-); DF Addr: 0278h; SRAM Offset: 024Bh ; SubClass Offset : 27
	///     Drive Near Far Dev[27] Ratio : Raw Dev * N/64
	U1 bNFRatio27;                                             
	/// 29. Abbrev: NEAR_FAR_RATIO_DEV28; Default: 0x40(-); DF Addr: 0279h; SRAM Offset: 024Ch ; SubClass Offset : 28
	///     Drive Near Far Dev[28] Ratio : Raw Dev * N/64
	U1 bNFRatio28;                                             
	/// 30. Abbrev: NEAR_FAR_RATIO_DEV29; Default: 0x40(-); DF Addr: 027Ah; SRAM Offset: 024Dh ; SubClass Offset : 29
	///     Drive Near Far Dev[29] Ratio : Raw Dev * N/64
	U1 bNFRatio29;                                             
	/// 31. Abbrev: NEAR_FAR_RATIO_DEV30; Default: 0x40(-); DF Addr: 027Bh; SRAM Offset: 024Eh ; SubClass Offset : 30
	///     Drive Near Far Dev[30] Ratio : Raw Dev * N/64
	U1 bNFRatio30;                                             
	/// 32. Abbrev: NEAR_FAR_RATIO_DEV31; Default: 0x40(-); DF Addr: 027Ch; SRAM Offset: 024Fh ; SubClass Offset : 31
	///     Drive Near Far Dev[31] Ratio : Raw Dev * N/64
	U1 bNFRatio31;                                             
} SubClassNearToFarType;	///< Total = 32 Bytes

/// =============================================================///
/// Data flash Class #17 => TpCrossTalkFix
/// =============================================================///
typedef struct PACK ClassTpCrossTalkFixSt
{
	SubClassR1toR2Type scR1toR2;
	SubClassNearToFarType scNearToFar;
}ClassTpCrossTalkFixType;	///< Total = 35 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #32 => DevCompensate
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDevCompensateSt
{
	///  1. Abbrev: INIT_MUT_RATIO_CTRL; Default: 0x00(-); DF Addr: 0280h; SRAM Offset: 0250h ; SubClass Offset : 0
	///     [DYN]enable Merge Mutual with initial
	U1 bInitMutRatioEnCtrl;                                    
	///  2. Abbrev: DRI_CMP_INIT_RATIO_NUMUMERATOR; Default: 0x00(-); DF Addr: 0281h; SRAM Offset: 0251h ; SubClass Offset : 1
	///     [DYN]Merge Mutual with initial
	U1 bDriAxisInitNumerator;                                  
	///  3. Abbrev: DRI_CMP_INIT_RATIO_DENOMINATOR; Default: 0x00(-); DF Addr: 0282h; SRAM Offset: 0252h ; SubClass Offset : 2
	///     [DYN]Merge Mutual with initial
	U1 bDriAxisInitDenominator;                                
	///  4. Abbrev: DRI_CMP_MUT_RATIO_NUMUMERATOR; Default: 0x00(-); DF Addr: 0283h; SRAM Offset: 0253h ; SubClass Offset : 3
	///     [DYN]Merge Mutual with initial
	U1 bDriAxisMutualNumerator;                                
	///  5. Abbrev: DRI_CMP_MUT_RATIO_DENOMINATOR; Default: 0x00(-); DF Addr: 0284h; SRAM Offset: 0254h ; SubClass Offset : 4
	///     [DYN]Merge Mutual with initial
	U1 bDriAxisMutualDenominator;                              
	///  6. Abbrev: INIT_MUT_MIN_TH; Default: 0x00(-); DF Addr: 0285h; SRAM Offset: 0255h ; SubClass Offset : 5
	///     [DYN]Merge Mutual with initial
	U1 bInitialMutualminTH;                                    
} SubClassDevCompensateType;	///< Total = 6 Bytes

/// =============================================================///
/// Data flash Class #18 => CoAxisAntiNoise
/// =============================================================///
typedef struct PACK ClassCoAxisAntiNoiseSt
{
	SubClassDevCompensateType scDevCompensate;
}ClassCoAxisAntiNoiseType;	///< Total = 6 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #33 => Normal
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNormalSt
{
	///  1. Abbrev: FINGER_TH; Default: 0x003C(-); DF Addr: 0286h; SRAM Offset: 0256h ; SubClass Offset : 0
	///     [DYN] Finger threshold
	U2 wFingerTH;                                              
	///  2. Abbrev: CLUSTER_FINGER_TH; Default: 0x0050(-); DF Addr: 0288h; SRAM Offset: 0258h ; SubClass Offset : 2
	///     [DYN] Finger threshold
	U2 wClusterPeakFingerTH;                                   
	///  3. Abbrev: FINGER_SLOP; Default: 0x46(-); DF Addr: 028Ah; SRAM Offset: 025Ah ; SubClass Offset : 4
	///     [DYN] Square Slope
	U1 bFingerSlop;                                            
	///  4. Abbrev: FINGER_WEIGHT; Default: 0x0050(-); DF Addr: 028Bh; SRAM Offset: 025Bh ; SubClass Offset : 5
	///     [DYN] Finger weight (Normal Mode)
	U2 wFingerWeight;                                          
	///  5. Abbrev: FINGER_TH_HEAVY; Default: 0x0037(-); DF Addr: 028Dh; SRAM Offset: 025Dh ; SubClass Offset : 7
	///     [DYN] Finger threshold When Heavy Press with out Charger
	U2 wFingerTHHeavy;                                         
	///  6. Abbrev: FINGER_SLOP_HEAVY; Default: 0x14(-); DF Addr: 028Fh; SRAM Offset: 025Fh ; SubClass Offset : 9
	///     [DYN] Square Slope Heavy Press
	U1 bFingerSlopHeavy;                                       
	///  7. Abbrev: FINGER_DEBOUNCE; Default: 0x02(-); DF Addr: 0290h; SRAM Offset: 0260h ; SubClass Offset : 10
	///     [DYN] One finger down debounce count for report (Normal Mode)
	U1 bDebounceCnt;                                           
	///  8. Abbrev: FINGER_DEBOUNCE_TWO; Default: 0x02(-); DF Addr: 0291h; SRAM Offset: 0261h ; SubClass Offset : 11
	///     [DYN]Tow fingers down debounce count for report (Normal Mode)
	U1 bDebounceCntTwo;                                        
	///  9. Abbrev: FINGER_DEBOUNCE_MUL; Default: 0x02(-); DF Addr: 0292h; SRAM Offset: 0262h ; SubClass Offset : 12
	///     [DYN] Multi-Finger down debounce count for report (Normal Mode)
	U1 bDebounceCntMulti;                                      
	/// 10. Abbrev: FINGER_UP_ENABLE; Default: 0x01(-); DF Addr: 0293h; SRAM Offset: 0263h ; SubClass Offset : 13
	///     [DYN] Finger Up enable
	U1 bFingerUpEnable;                                        
	/// 11. Abbrev: FINGER_UP_ONE_DEBOUNCE_CNT; Default: 0x01(-); DF Addr: 0294h; SRAM Offset: 0264h ; SubClass Offset : 14
	///     [DYN]  1 Finger Up Debounce count  (FINGER_UP_ENABLE shall be 1)
	U1 bDebounceUpCnt;                                         
	/// 12. Abbrev: FINGER_UP_MUL_DEBOUNCE_CNT; Default: 0x03(-); DF Addr: 0295h; SRAM Offset: 0265h ; SubClass Offset : 15
	///     [DYN] Mult-Finger Up Debounce count (FINGER_UP_ENABLE shall be 1)
	U1 bDebounceUpCntMulti;                                    
	/// 13. Abbrev: FINGER_WEIGHT_SCHMIT; Default: 0x00F0(-); DF Addr: 0296h; SRAM Offset: 0266h ; SubClass Offset : 16
	///     Finger Schmit Trigger:min-Finger-Dev to set trigger High
	U2 wFingerWeightSchmit;                                    
	/// 14. Abbrev: STEADY_RANGE; Default: 0x0A(-); DF Addr: 0298h; SRAM Offset: 0268h ; SubClass Offset : 18
	///     [DYN] Steady Range
	U1 bSteadyRange;                                           
	/// 15. Abbrev: STEADY_RANGE_MOVE; Default: 0x0A(-); DF Addr: 0299h; SRAM Offset: 0269h ; SubClass Offset : 19
	///     [DYN] Steady Range whe Finger is not move
	U1 bSteadyRangeMove;                                       
	/// 16. Abbrev: STEADY_RANGE_BOR; Default: 0x0A(-); DF Addr: 029Ah; SRAM Offset: 026Ah ; SubClass Offset : 20
	///     [DYN] Steady Range
	U1 bSteadyRangeBorder;                                     
	/// 17. Abbrev: STEADY_RANGE_BOR_MOVE; Default: 0x0A(-); DF Addr: 029Bh; SRAM Offset: 026Bh ; SubClass Offset : 21
	///     [DYN] Steady Range whe Finger is not move
	U1 bSteadyRangeBorderMove;                                 
	/// 18. Abbrev: STEADY_RANGE_BOR_COAXIS; Default: 0x0A(-); DF Addr: 029Ch; SRAM Offset: 026Ch ; SubClass Offset : 22
	///     [DYN] Steady Range whe Finger is not move
	U1 bSteadyRangeBorderCoAxis;                               
	/// 19. Abbrev: STEADY_RANGE_COAXIS_MOVE; Default: 0x0C(-); DF Addr: 029Dh; SRAM Offset: 026Dh ; SubClass Offset : 23
	///     [DYN] Co-Axis  Steady Range
	U1 bSteadyRangeCoAxisMove;                                 
	/// 20. Abbrev: STEADY_RANGE_COAXIS; Default: 0x0C(-); DF Addr: 029Eh; SRAM Offset: 026Eh ; SubClass Offset : 24
	///     [DYN] Co-Axis  Steady Range
	U1 bSteadyRangeCoAxis;                                     
} SubClassNormalType;	///< Total = 25 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #34 => PDypeSmoothCtrl
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassPDypeSmoothCtrlSt
{
	///  1. Abbrev: PD_TYPE_RATIOT; Default: 0x02(-); DF Addr: 029Fh; SRAM Offset: 026Fh ; SubClass Offset : 0
	///     [DYN] PD-Type smooth
	U1 bRatioT;                                                
	///  2. Abbrev: PD_TYPE_RATIOX; Default: 0x03(-); DF Addr: 02A0h; SRAM Offset: 0270h ; SubClass Offset : 1
	///     [DYN] PD-Type smooth
	U1 bRatioX;                                                
	///  3. Abbrev: PD_TYPE_RATIOY; Default: 0x03(-); DF Addr: 02A1h; SRAM Offset: 0271h ; SubClass Offset : 2
	///     [DYN] PD-Type smooth
	U1 bRatioY;                                                
	///  4. Abbrev: PD_TYPE_TH; Default: 0xFF(-); DF Addr: 02A2h; SRAM Offset: 0272h ; SubClass Offset : 3
	///     [DYN] PD-Type move threshold
	U1 bDistanceTH;                                            
} SubClassPDypeSmoothCtrlType;	///< Total = 4 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #35 => Charge
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassChargeSt
{
	///  1. Abbrev: FINGER_TH_CHG; Default: 0x0050(-); DF Addr: 02A3h; SRAM Offset: 0273h ; SubClass Offset : 0
	///     [DYN] Finger threshold in charger mode
	U2 wFingerTH;                                              
	///  2. Abbrev: FINGER_SLOP_CHG; Default: 0x5A(-); DF Addr: 02A5h; SRAM Offset: 0275h ; SubClass Offset : 2
	///     [DYN] Square Slope at charger mode
	U1 bFingerSlop;                                            
	///  3. Abbrev: FINGER_WEIGHT_CHG; Default: 0x0064(-); DF Addr: 02A6h; SRAM Offset: 0276h ; SubClass Offset : 3
	///     [DYN] Finger weight (charger Mode)
	U2 wFingerWeight;                                          
	///  4. Abbrev: FINGER_TH_CHG_HEAVY; Default: 0x0037(-); DF Addr: 02A8h; SRAM Offset: 0278h ; SubClass Offset : 5
	///     [DYN] Finger threshold When Heavy Press with Charger
	U2 wFingerTHHeavy;                                         
	///  5. Abbrev: FINGER_SLOP_CHG_HEAVY; Default: 0x14(-); DF Addr: 02AAh; SRAM Offset: 027Ah ; SubClass Offset : 7
	///     [DYN] Square Slope Heavy Press at charger mode
	U1 bFingerSlopHeavy;                                       
	///  6. Abbrev: FINGER_WEIGHT_CHG_HEAVY; Default: 0x005A(-); DF Addr: 02ABh; SRAM Offset: 027Bh ; SubClass Offset : 8
	///     [DYN] Finger weight (HeavyPress charger Mode)
	U2 wFingerWeightHeavy;                                     
	///  7. Abbrev: FINGER_DEBOUNCE_CHG; Default: 0x02(-); DF Addr: 02ADh; SRAM Offset: 027Dh ; SubClass Offset : 10
	///     [DYN] One finger down debounce count for report (charger Mode)
	U1 bDebounceCnt;                                           
	///  8. Abbrev: FINGER_DEBOUNCE_CHG_TWO; Default: 0x02(-); DF Addr: 02AEh; SRAM Offset: 027Eh ; SubClass Offset : 11
	///     [DYN] Tow fingers down debounce count for report (charger Mode)
	U1 bDebounceCntTwo;                                        
	///  9. Abbrev: FINGER_DEBOUNCE_CHG_MUL; Default: 0x02(-); DF Addr: 02AFh; SRAM Offset: 027Fh ; SubClass Offset : 12
	///     [DYN] Multi-Finger down debounce count for report (charger Mode)
	U1 bDebounceCntMulti;                                      
} SubClassChargeType;	///< Total = 13 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #36 => CoAxis
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassCoAxisSt
{
	///  1. Abbrev: COAXIS_ENABLE; Default: 0x00(-); DF Addr: 02B0h; SRAM Offset: 0280h ; SubClass Offset : 0
	///     [DYN] 0: Disable, 1: Charger mode enable, 2: Always enable, 3: Normal enable
	U1 bCoAxisEnable;                                          
	///  2. Abbrev: COAXIS_FINGER_TH; Default: 0x0028(-); DF Addr: 02B1h; SRAM Offset: 0281h ; SubClass Offset : 1
	///     [DYN] Finger TH for CO-Axis
	U2 wFingerTH;                                              
	///  3. Abbrev: COAXIS_FINGER_WRIGHT; Default: 0x0064(-); DF Addr: 02B3h; SRAM Offset: 0283h ; SubClass Offset : 3
	///     [DYN] Finger Weight for CO-Axis
	U2 wFingerWeight;                                          
	///  4. Abbrev: COAXIS_FINGERNUM_DEFINE; Default: 0x02(-); DF Addr: 02B5h; SRAM Offset: 0285h ; SubClass Offset : 5
	///     [DYN] The CoAxis Finger num define
	U1 bCoAxisFingerNumDefine;                                 
	///  5. Abbrev: COAXIS_DEBOUNCE_CNT_MULTI; Default: 0x0A(-); DF Addr: 02B6h; SRAM Offset: 0286h ; SubClass Offset : 6
	///     [DYN] Finger Debounce for Co-Axis
	U1 bDebounceCntMulti;                                      
	///  6. Abbrev: COAXIS_FINGER_UP_MUL_DEBOUNCE_CNT; Default: 0x01(-); DF Addr: 02B7h; SRAM Offset: 0287h ; SubClass Offset : 7
	///     [DYN] Mult-Finger up debounce count (FINGER_UP_ENABLE shall be 1) for Co-Axis
	U1 bFingerUpMulDebounceCnt;                                
	///  7. Abbrev: FINGER_WEIGHT_SCHMIT_COAXIS; Default: 0x010E(-); DF Addr: 02B8h; SRAM Offset: 0288h ; SubClass Offset : 8
	///     Finger Schmit Trigger:min-Finger-Dev to set trigger High
	U2 wFingerWeightSchmit;                                    
} SubClassCoAxisType;	///< Total = 10 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #37 => Border
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassBorderSt
{
	///  1. Abbrev: FINGER_WEIGHT_SCHMIT_BORDER; Default: 0x0078(-); DF Addr: 02BAh; SRAM Offset: 028Ah ; SubClass Offset : 0
	///     Finger Schmit Trigger Border:min-Finger-Dev to set trigger High
	U2 wFingerWeightSchmit;                                    
	///  2. Abbrev: DEBOUNCE_CNT_BORDER; Default: 0x02(-); DF Addr: 02BCh; SRAM Offset: 028Ch ; SubClass Offset : 2
	///     [DYN] Finger down debounce count for report (When Finger in Border)
	U1 bDebounceCnt;                                           
} SubClassBorderType;	///< Total = 3 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #38 => Water
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassWaterSt
{
	///  1. Abbrev: FINGER_WEIGHT_SCHMIT_ON_WATER; Default: 0x0064(-); DF Addr: 02BDh; SRAM Offset: 028Dh ; SubClass Offset : 0
	///     Finger Schmit Trigger:min-Finger-Dev to set trigger High
	U2 wFingerWeightSchmitOnWater;                             
	///  2. Abbrev: FINGER_WEIGHT_SCHMIT_OFF_WATER; Default: 0x0032(-); DF Addr: 02BFh; SRAM Offset: 028Fh ; SubClass Offset : 2
	///     [DYN] Cluster weight (Normal Mode)
	U2 wFingerWeightSchmitOffWater;                            
	///  3. Abbrev: CLUSTER_FINGER_TH_WATER; Default: 0x0046(-); DF Addr: 02C1h; SRAM Offset: 0291h ; SubClass Offset : 4
	///     [DYN] Finger threshold
	U2 wClusterPeakFingerTHWater;                              
	///  4. Abbrev: CLUSTER_DEV_MAX_WATER; Default: 0x005A(-); DF Addr: 02C3h; SRAM Offset: 0293h ; SubClass Offset : 6
	///     [DYN] Cluster weight (Normal Mode)
	U2 wClusterDevSchmiOn;                                     
	///  5. Abbrev: CLUSTER_DEV_MIN_WATER; Default: 0x0014(-); DF Addr: 02C5h; SRAM Offset: 0295h ; SubClass Offset : 8
	///     [DYN] Cluster weight (Normal Mode)
	U2 wClusterDevSchmiOff;                                    
	///  6. Abbrev: FINGER_NUM_WATER; Default: 0x00(-); DF Addr: 02C7h; SRAM Offset: 0297h ; SubClass Offset : 10
	///     
	U1 bFingerNumWater;                                        
	///  7. Abbrev: WATER_AREA_DRIVE_AXIS_START; Default: 0x01(-); DF Addr: 02C8h; SRAM Offset: 0298h ; SubClass Offset : 11
	///     
	U1 bWaterAreaDriveStart;                                   
	///  8. Abbrev: WATER_AREA_SENSE_AXIS_START; Default: 0x01(-); DF Addr: 02C9h; SRAM Offset: 0299h ; SubClass Offset : 12
	///     
	U1 bWaterAreaSenseStart;                                   
	///  9. Abbrev: WATER_AREA_DRIVE_AXIS_END; Default: 0x0A(-); DF Addr: 02CAh; SRAM Offset: 029Ah ; SubClass Offset : 13
	///     
	U1 bWaterAreaDriveEnd;                                     
	/// 10. Abbrev: WATER_AREA_SENSE_AXIS_END; Default: 0x0A(-); DF Addr: 02CBh; SRAM Offset: 029Bh ; SubClass Offset : 14
	///     
	U1 bWaterAreaSenseEnd;                                     
} SubClassWaterType;	///< Total = 15 Bytes

/// =============================================================///
/// Data flash Class #19 => Finger
/// =============================================================///
typedef struct PACK ClassFingerSt
{
	SubClassNormalType scNormal;
	SubClassPDypeSmoothCtrlType scPDypeSmoothCtrl;
	SubClassChargeType scCharge;
	SubClassCoAxisType scCoAxis;
	SubClassBorderType scBorder;
	SubClassWaterType scWater;
}ClassFingerType;	///< Total = 70 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #39 => AlgoProcDevTh
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoProcDevThSt
{
	///  1. Abbrev: SEN_SAVE_DEV_TH; Default: 0x0050(-Old Dev Process); DF Addr: 0300h; SRAM Offset: 029Ch ; SubClass Offset : 0
	///     [DYN] Sen Dev Finger Save threshold (Normal Mode)
	U2 wSenFingerSaveTH;                                       
	///  2. Abbrev: SEN_HEAVY_DEV_TH; Default: 0x003C(-); DF Addr: 0302h; SRAM Offset: 029Eh ; SubClass Offset : 2
	///     [DYN] Sen dev heavy press thredhold
	U2 wSenHeavyTH;                                            
	///  3. Abbrev: SEN_HEAVY_SAVE_DEV_TH; Default: 0x0078(-); DF Addr: 0304h; SRAM Offset: 02A0h ; SubClass Offset : 4
	///     [DYN] Sen Dev Finger Save threshold (Heavy pressed)
	U2 wSenHeavyFingerSaveTH;                                  
	///  4. Abbrev: DRI_SAVE_DEV_TH; Default: 0x0064(-); DF Addr: 0306h; SRAM Offset: 02A2h ; SubClass Offset : 6
	///     [DYN] Dri Dev Finger Save threshold (Normal Mode)
	U2 wDriFingerSaveTH;                                       
	///  5. Abbrev: DRI_CHG_SAVE_DEV_TH; Default: 0x0064(-); DF Addr: 0308h; SRAM Offset: 02A4h ; SubClass Offset : 8
	///     [DYN] Dri Dev Finger Save threshold (Charge Mode)
	U2 wDriChgFingerSaveTH;                                    
	///  6. Abbrev: INIT_SAVE_DEV_TH; Default: 0x0028(-); DF Addr: 030Ah; SRAM Offset: 02A6h ; SubClass Offset : 10
	///     Dev Process Init: init dev save threshold
	U2 wInitDevProcSaveTh;                                     
	///  7. Abbrev: INIT_SKIP_DEV_TH; Default: 0x00FF(-); DF Addr: 030Ch; SRAM Offset: 02A8h ; SubClass Offset : 12
	///     Dev Process Init: : init dev skip upper bound threshold
	U2 wInitDevProcSkipTh;                                     
} SubClassAlgoProcDevThType;	///< Total = 14 Bytes

/// =============================================================///
/// Data flash Class #20 => AxisProcess
/// =============================================================///
typedef struct PACK ClassAxisProcessSt
{
	SubClassAlgoProcDevThType scAlgoProcDevTh;
}ClassAxisProcessType;	///< Total = 14 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #40 => DevCut
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDevCutSt
{
	///  1. Abbrev: MUT_DEV_CUT; Default: 0x20(-); DF Addr: 030Eh; SRAM Offset: 02AAh ; SubClass Offset : 0
	///     [!!!] Mutual DEV Cut
	U1 bMutDevCut;                                             
	///  2. Abbrev: INIT_DEV_CUT; Default: 0x08(-); DF Addr: 030Fh; SRAM Offset: 02ABh ; SubClass Offset : 1
	///     [!!!] Init DEV Cut
	U1 bInitDevCut;                                            
	///  3. Abbrev: INIT_POST_DEV_CUT; Default: 0x10(-); DF Addr: 0310h; SRAM Offset: 02ACh ; SubClass Offset : 2
	///     [!!!] Init POST DEV CUT
	U1 bInitPostDevCut;                                        
} SubClassDevCutType;	///< Total = 3 Bytes

/// =============================================================///
/// Data flash Class #21 => DevCut
/// =============================================================///
typedef struct PACK ClassDevCutSt
{
	SubClassDevCutType scDevCut;
}ClassDevCutType;	///< Total = 3 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #41 => OneFingerCluster
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassOneFingerClusterSt
{
	///  1. Abbrev: ONE_FINGER_CLUSTER_EN; Default: 0x00(-); DF Addr: 0311h; SRAM Offset: 02ADh ; SubClass Offset : 0
	///     0 : Disable ; 1 : Enable : one finger cluster function, sharp dev proess
	U1 bOneFingerClusterEn;                                    
	///  2. Abbrev: SHARP_DEV_CUT_EN; Default: 0x00(-); DF Addr: 0312h; SRAM Offset: 02AEh ; SubClass Offset : 1
	///     0 : Disable ; 1 : Enable  : one finger cluster shall be enabled
	U1 bSharpDevCutEn;                                         
	///  3. Abbrev: SHARP_DEV_RATIO; Default: 0x04(-); DF Addr: 0313h; SRAM Offset: 02AFh ; SubClass Offset : 2
	///     Sharp Dev Ratio, the maximmum peak dev ratio to the dev-s around, if the max ratio exceed  the value, invoke sharp dev cut function
	U1 bSharpDevRatio;                                         
	///  4. Abbrev: SHARP_DEV_LEVEL; Default: 0x32(-); DF Addr: 0314h; SRAM Offset: 02B0h ; SubClass Offset : 3
	///     Sharp Dev Level in %, to  change the value of the max dev
	U1 bSharpDevLevel;                                         
	///  5. Abbrev: NINESQU_WEIGHT_AVG_RATO; Default: 0x04(-); DF Addr: 0315h; SRAM Offset: 02B1h ; SubClass Offset : 4
	///     Sharp Dev Level in %, to  change the value of the max dev
	U1 bNineSquWeightDivider;                                  
} SubClassOneFingerClusterType;	///< Total = 5 Bytes

/// =============================================================///
/// Data flash Class #22 => Linearity
/// =============================================================///
typedef struct PACK ClassLinearitySt
{
	SubClassOneFingerClusterType scOneFingerCluster;
}ClassLinearityType;	///< Total = 5 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #42 => AlgoBorderCoorComp
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoBorderCoorCompSt
{
	///  1. Abbrev: BORDER_COOR_ENABLE; Default: 0x01(-); DF Addr: 0316h; SRAM Offset: 02B2h ; SubClass Offset : 0
	///     [DYN] BorderCoordinate Compensation Enable: 0 : Disable, 1: Enable
	U1 bBorderCoorEnable;                                      
	///  2. Abbrev: BORDER_SEN_NEAR_COOR_PERCENT; Default: 0xF0(-); DF Addr: 0317h; SRAM Offset: 02B3h ; SubClass Offset : 1
	///     [DYN] Border Sense Axis Coordinate Compensation : Percentage from near side border to coordinate temination line
	U1 bBorderSenNearCoorPercent;                              
	///  3. Abbrev: BORDER_SEN_FAR_COOR_PERCENT; Default: 0xC8(-); DF Addr: 0318h; SRAM Offset: 02B4h ; SubClass Offset : 2
	///     [DYN] Border Sense Axis Coordinate Compensation : Percentage from far side border to coordinate temination line
	U1 bBorderSenFarCoorPercent;                               
	///  4. Abbrev: BORDER_DRI_NEAR_COOR_PERCENT; Default: 0x64(-); DF Addr: 0319h; SRAM Offset: 02B5h ; SubClass Offset : 3
	///     [DYN] Border Drive Axis Coordinate Compensation : Percentage from near side border to coordinate temination line
	U1 bBorderDriNearCoorPercent;                              
	///  5. Abbrev: BORDER_DRI_FAR_COOR_PERCENT; Default: 0x64(-); DF Addr: 031Ah; SRAM Offset: 02B6h ; SubClass Offset : 4
	///     [DYN] Border Drive Axis Coordinate Compensation : Percentage from far side border to coordinate temination line
	U1 bBorderDriFarCoorPercent;                               
	///  6. Abbrev: BORDER_HIT_SEN_NEAR_COOR_PERCENT; Default: 0xC8(-); DF Addr: 031Bh; SRAM Offset: 02B7h ; SubClass Offset : 5
	///     [DYN] Border Hit Sense Axis Coordinate Compensation : Percentage from near side border to coordinate temination line for border hit
	U1 bBorderHitSenNearCoorPercent;                           
	///  7. Abbrev: BORDER_HIT_SEN_FAR_COOR_PERCENT; Default: 0x90(-); DF Addr: 031Ch; SRAM Offset: 02B8h ; SubClass Offset : 6
	///     [DYN] Border Hit Sense Axis Coordinate Compensation : Percentage from far side border to coordinate temination line for border hit
	U1 bBorderHitSenFarCoorPercent;                            
	///  8. Abbrev: BORDER_HIT_DRI_NEAR_COOR_PERCENT; Default: 0x64(-); DF Addr: 031Dh; SRAM Offset: 02B9h ; SubClass Offset : 7
	///     [DYN] Border Hit Drive Axis Coordinate Compensation : Percentage from near side border to coordinate temination line for border hit
	U1 bBorderHitDriNearCoorPercent;                           
	///  9. Abbrev: BORDER_HIT_DRI_FAR_COOR_PERCENT; Default: 0x64(-); DF Addr: 031Eh; SRAM Offset: 02BAh ; SubClass Offset : 8
	///     [DYN] Border Hit Drive Axis Coordinate Compensation : Percentage from far side border to coordinate temination line for border hit
	U1 bBorderHitDriFarCoorPercent;                            
} SubClassAlgoBorderCoorCompType;	///< Total = 9 Bytes

/// =============================================================///
/// Data flash Class #23 => BorderComp
/// =============================================================///
typedef struct PACK ClassBorderCompSt
{
	SubClassAlgoBorderCoorCompType scAlgoBorderCoorComp;
}ClassBorderCompType;	///< Total = 9 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #43 => CoordRemapSetup
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassCoordRemapSetupSt
{
	///  1. Abbrev: COORD_REMAP_ENABLE; Default: 0(-); DF Addr: 031Fh; SRAM Offset: 02BBh ; SubClass Offset : 0
	///     Coordinate Remap for different length pitch Pin: 1:Enable, 0:Disable
	U1 bCoordRemapEnable;                                      
	///  2. Abbrev: COORD_REMAP_DRIVE_AXIS00; Default: 0(-); DF Addr: 0320h; SRAM Offset: 02BCh ; SubClass Offset : 1
	///     Coordinate Remap Drive Axis00:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis00;                                 
	///  3. Abbrev: COORD_REMAP_DRIVE_AXIS01; Default: 1(-); DF Addr: 0321h; SRAM Offset: 02BDh ; SubClass Offset : 2
	///     Coordinate Remap Drive Axis01:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis01;                                 
	///  4. Abbrev: COORD_REMAP_DRIVE_AXIS02; Default: 2(-); DF Addr: 0322h; SRAM Offset: 02BEh ; SubClass Offset : 3
	///     Coordinate Remap Drive Axis02:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis02;                                 
	///  5. Abbrev: COORD_REMAP_DRIVE_AXIS03; Default: 3(-); DF Addr: 0323h; SRAM Offset: 02BFh ; SubClass Offset : 4
	///     Coordinate Remap Drive Axis03:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis03;                                 
	///  6. Abbrev: COORD_REMAP_DRIVE_AXIS04; Default: 4(-); DF Addr: 0324h; SRAM Offset: 02C0h ; SubClass Offset : 5
	///     Coordinate Remap Drive Axis04:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis04;                                 
	///  7. Abbrev: COORD_REMAP_DRIVE_AXIS05; Default: 5(-); DF Addr: 0325h; SRAM Offset: 02C1h ; SubClass Offset : 6
	///     Coordinate Remap Drive Axis05:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis05;                                 
	///  8. Abbrev: COORD_REMAP_DRIVE_AXIS06; Default: 6(-); DF Addr: 0326h; SRAM Offset: 02C2h ; SubClass Offset : 7
	///     Coordinate Remap Drive Axis06:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis06;                                 
	///  9. Abbrev: COORD_REMAP_DRIVE_AXIS07; Default: 7(-); DF Addr: 0327h; SRAM Offset: 02C3h ; SubClass Offset : 8
	///     Coordinate Remap Drive Axis07:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis07;                                 
	/// 10. Abbrev: COORD_REMAP_DRIVE_AXIS08; Default: 8(-); DF Addr: 0328h; SRAM Offset: 02C4h ; SubClass Offset : 9
	///     Coordinate Remap Drive Axis08:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis08;                                 
	/// 11. Abbrev: COORD_REMAP_DRIVE_AXIS09; Default: 9(-); DF Addr: 0329h; SRAM Offset: 02C5h ; SubClass Offset : 10
	///     Coordinate Remap Drive Axis09:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis09;                                 
	/// 12. Abbrev: COORD_REMAP_DRIVE_AXIS10; Default: 10(-); DF Addr: 032Ah; SRAM Offset: 02C6h ; SubClass Offset : 11
	///     Coordinate Remap Drive Axis10:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis10;                                 
	/// 13. Abbrev: COORD_REMAP_DRIVE_AXIS11; Default: 11(-); DF Addr: 032Bh; SRAM Offset: 02C7h ; SubClass Offset : 12
	///     Coordinate Remap Drive Axis11:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis11;                                 
	/// 14. Abbrev: COORD_REMAP_DRIVE_AXIS12; Default: 12(-); DF Addr: 032Ch; SRAM Offset: 02C8h ; SubClass Offset : 13
	///     Coordinate Remap Drive Axis12:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis12;                                 
	/// 15. Abbrev: COORD_REMAP_DRIVE_AXIS13; Default: 13(-); DF Addr: 032Dh; SRAM Offset: 02C9h ; SubClass Offset : 14
	///     Coordinate Remap Drive Axis13:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis13;                                 
	/// 16. Abbrev: COORD_REMAP_DRIVE_AXIS14; Default: 14(-); DF Addr: 032Eh; SRAM Offset: 02CAh ; SubClass Offset : 15
	///     Coordinate Remap Drive Axis14:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis14;                                 
	/// 17. Abbrev: COORD_REMAP_DRIVE_AXIS15; Default: 15(-); DF Addr: 032Fh; SRAM Offset: 02CBh ; SubClass Offset : 16
	///     Coordinate Remap Drive Axis15:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis15;                                 
	/// 18. Abbrev: COORD_REMAP_DRIVE_AXIS16; Default: 16(-); DF Addr: 0330h; SRAM Offset: 02CCh ; SubClass Offset : 17
	///     Coordinate Remap Drive Axis16:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis16;                                 
	/// 19. Abbrev: COORD_REMAP_DRIVE_AXIS17; Default: 17(-); DF Addr: 0331h; SRAM Offset: 02CDh ; SubClass Offset : 18
	///     Coordinate Remap Drive Axis17:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis17;                                 
	/// 20. Abbrev: COORD_REMAP_DRIVE_AXIS18; Default: 18(-); DF Addr: 0332h; SRAM Offset: 02CEh ; SubClass Offset : 19
	///     Coordinate Remap Drive Axis18:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis18;                                 
	/// 21. Abbrev: COORD_REMAP_DRIVE_AXIS19; Default: 19(-); DF Addr: 0333h; SRAM Offset: 02CFh ; SubClass Offset : 20
	///     Coordinate Remap Drive Axis19:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis19;                                 
	/// 22. Abbrev: COORD_REMAP_DRIVE_AXIS20; Default: 20(-); DF Addr: 0334h; SRAM Offset: 02D0h ; SubClass Offset : 21
	///     Coordinate Remap Drive Axis20:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis20;                                 
	/// 23. Abbrev: COORD_REMAP_DRIVE_AXIS21; Default: 21(-); DF Addr: 0335h; SRAM Offset: 02D1h ; SubClass Offset : 22
	///     Coordinate Remap Drive Axis21:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis21;                                 
	/// 24. Abbrev: COORD_REMAP_DRIVE_AXIS22; Default: 22(-); DF Addr: 0336h; SRAM Offset: 02D2h ; SubClass Offset : 23
	///     Coordinate Remap Drive Axis22:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis22;                                 
	/// 25. Abbrev: COORD_REMAP_DRIVE_AXIS23; Default: 23(-); DF Addr: 0337h; SRAM Offset: 02D3h ; SubClass Offset : 24
	///     Coordinate Remap Drive Axis23:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis23;                                 
	/// 26. Abbrev: COORD_REMAP_DRIVE_AXIS24; Default: 24(-); DF Addr: 0338h; SRAM Offset: 02D4h ; SubClass Offset : 25
	///     Coordinate Remap Drive Axis24:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis24;                                 
	/// 27. Abbrev: COORD_REMAP_DRIVE_AXIS25; Default: 25(-); DF Addr: 0339h; SRAM Offset: 02D5h ; SubClass Offset : 26
	///     Coordinate Remap Drive Axis25:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis25;                                 
	/// 28. Abbrev: COORD_REMAP_DRIVE_AXIS26; Default: 26(-); DF Addr: 033Ah; SRAM Offset: 02D6h ; SubClass Offset : 27
	///     Coordinate Remap Drive Axis26:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis26;                                 
	/// 29. Abbrev: COORD_REMAP_DRIVE_AXIS27; Default: 27(-); DF Addr: 033Bh; SRAM Offset: 02D7h ; SubClass Offset : 28
	///     Coordinate Remap Drive Axis27:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis27;                                 
	/// 30. Abbrev: COORD_REMAP_DRIVE_AXIS28; Default: 28(-); DF Addr: 033Ch; SRAM Offset: 02D8h ; SubClass Offset : 29
	///     Coordinate Remap Drive Axis28:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis28;                                 
	/// 31. Abbrev: COORD_REMAP_DRIVE_AXIS29; Default: 29(-); DF Addr: 033Dh; SRAM Offset: 02D9h ; SubClass Offset : 30
	///     Coordinate Remap Drive Axis29:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis29;                                 
	/// 32. Abbrev: COORD_REMAP_DRIVE_AXIS30; Default: 30(-); DF Addr: 033Eh; SRAM Offset: 02DAh ; SubClass Offset : 31
	///     Coordinate Remap Drive Axis30:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis30;                                 
	/// 33. Abbrev: COORD_REMAP_DRIVE_AXIS31; Default: 31(-); DF Addr: 033Fh; SRAM Offset: 02DBh ; SubClass Offset : 32
	///     Coordinate Remap Drive Axis31:Pin Unit Coordinate.
	U1 bCoordRemapDriveAxis31;                                 
	/// 34. Abbrev: COORD_REMAP_SENSE_AXIS00; Default: 0(-); DF Addr: 0340h; SRAM Offset: 02DCh ; SubClass Offset : 33
	///     Coordinate Remap Sense Axis00:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis00;                                 
	/// 35. Abbrev: COORD_REMAP_SENSE_AXIS01; Default: 1(-); DF Addr: 0341h; SRAM Offset: 02DDh ; SubClass Offset : 34
	///     Coordinate Remap Sense Axis01:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis01;                                 
	/// 36. Abbrev: COORD_REMAP_SENSE_AXIS02; Default: 2(-); DF Addr: 0342h; SRAM Offset: 02DEh ; SubClass Offset : 35
	///     Coordinate Remap Sense Axis02:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis02;                                 
	/// 37. Abbrev: COORD_REMAP_SENSE_AXIS03; Default: 3(-); DF Addr: 0343h; SRAM Offset: 02DFh ; SubClass Offset : 36
	///     Coordinate Remap Sense Axis03:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis03;                                 
	/// 38. Abbrev: COORD_REMAP_SENSE_AXIS04; Default: 4(-); DF Addr: 0344h; SRAM Offset: 02E0h ; SubClass Offset : 37
	///     Coordinate Remap Sense Axis04:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis04;                                 
	/// 39. Abbrev: COORD_REMAP_SENSE_AXIS05; Default: 5(-); DF Addr: 0345h; SRAM Offset: 02E1h ; SubClass Offset : 38
	///     Coordinate Remap Sense Axis05:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis05;                                 
	/// 40. Abbrev: COORD_REMAP_SENSE_AXIS06; Default: 6(-); DF Addr: 0346h; SRAM Offset: 02E2h ; SubClass Offset : 39
	///     Coordinate Remap Sense Axis06:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis06;                                 
	/// 41. Abbrev: COORD_REMAP_SENSE_AXIS07; Default: 7(-); DF Addr: 0347h; SRAM Offset: 02E3h ; SubClass Offset : 40
	///     Coordinate Remap Sense Axis07:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis07;                                 
	/// 42. Abbrev: COORD_REMAP_SENSE_AXIS08; Default: 8(-); DF Addr: 0348h; SRAM Offset: 02E4h ; SubClass Offset : 41
	///     Coordinate Remap Sense Axis08:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis08;                                 
	/// 43. Abbrev: COORD_REMAP_SENSE_AXIS09; Default: 9(-); DF Addr: 0349h; SRAM Offset: 02E5h ; SubClass Offset : 42
	///     Coordinate Remap Sense Axis09:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis09;                                 
	/// 44. Abbrev: COORD_REMAP_SENSE_AXIS10; Default: 10(-); DF Addr: 034Ah; SRAM Offset: 02E6h ; SubClass Offset : 43
	///     Coordinate Remap Sense Axis10:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis10;                                 
	/// 45. Abbrev: COORD_REMAP_SENSE_AXIS11; Default: 11(-); DF Addr: 034Bh; SRAM Offset: 02E7h ; SubClass Offset : 44
	///     Coordinate Remap Sense Axis11:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis11;                                 
	/// 46. Abbrev: COORD_REMAP_SENSE_AXIS12; Default: 12(-); DF Addr: 034Ch; SRAM Offset: 02E8h ; SubClass Offset : 45
	///     Coordinate Remap Sense Axis12:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis12;                                 
	/// 47. Abbrev: COORD_REMAP_SENSE_AXIS13; Default: 13(-); DF Addr: 034Dh; SRAM Offset: 02E9h ; SubClass Offset : 46
	///     Coordinate Remap Sense Axis13:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis13;                                 
	/// 48. Abbrev: COORD_REMAP_SENSE_AXIS14; Default: 14(-); DF Addr: 034Eh; SRAM Offset: 02EAh ; SubClass Offset : 47
	///     Coordinate Remap Sense Axis14:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis14;                                 
	/// 49. Abbrev: COORD_REMAP_SENSE_AXIS15; Default: 15(-); DF Addr: 034Fh; SRAM Offset: 02EBh ; SubClass Offset : 48
	///     Coordinate Remap Sense Axis15:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis15;                                 
	/// 50. Abbrev: COORD_REMAP_SENSE_AXIS16; Default: 16(-); DF Addr: 0350h; SRAM Offset: 02ECh ; SubClass Offset : 49
	///     Coordinate Remap Sense Axis16:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis16;                                 
	/// 51. Abbrev: COORD_REMAP_SENSE_AXIS17; Default: 17(-); DF Addr: 0351h; SRAM Offset: 02EDh ; SubClass Offset : 50
	///     Coordinate Remap Sense Axis17:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis17;                                 
	/// 52. Abbrev: COORD_REMAP_SENSE_AXIS18; Default: 18(-); DF Addr: 0352h; SRAM Offset: 02EEh ; SubClass Offset : 51
	///     Coordinate Remap Sense Axis18:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis18;                                 
	/// 53. Abbrev: COORD_REMAP_SENSE_AXIS19; Default: 19(-); DF Addr: 0353h; SRAM Offset: 02EFh ; SubClass Offset : 52
	///     Coordinate Remap Sense Axis19:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis19;                                 
	/// 54. Abbrev: COORD_REMAP_SENSE_AXIS20; Default: 20(-); DF Addr: 0354h; SRAM Offset: 02F0h ; SubClass Offset : 53
	///     Coordinate Remap Sense Axis20:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis20;                                 
	/// 55. Abbrev: COORD_REMAP_SENSE_AXIS21; Default: 21(-); DF Addr: 0355h; SRAM Offset: 02F1h ; SubClass Offset : 54
	///     Coordinate Remap Sense Axis21:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis21;                                 
	/// 56. Abbrev: COORD_REMAP_SENSE_AXIS22; Default: 22(-); DF Addr: 0356h; SRAM Offset: 02F2h ; SubClass Offset : 55
	///     Coordinate Remap Sense Axis22:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis22;                                 
	/// 57. Abbrev: COORD_REMAP_SENSE_AXIS23; Default: 23(-); DF Addr: 0357h; SRAM Offset: 02F3h ; SubClass Offset : 56
	///     Coordinate Remap Sense Axis23:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis23;                                 
	/// 58. Abbrev: COORD_REMAP_SENSE_AXIS24; Default: 24(-); DF Addr: 0358h; SRAM Offset: 02F4h ; SubClass Offset : 57
	///     Coordinate Remap Sense Axis24:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis24;                                 
	/// 59. Abbrev: COORD_REMAP_SENSE_AXIS25; Default: 25(-); DF Addr: 0359h; SRAM Offset: 02F5h ; SubClass Offset : 58
	///     Coordinate Remap Sense Axis25:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis25;                                 
	/// 60. Abbrev: COORD_REMAP_SENSE_AXIS26; Default: 26(-); DF Addr: 035Ah; SRAM Offset: 02F6h ; SubClass Offset : 59
	///     Coordinate Remap Sense Axis26:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis26;                                 
	/// 61. Abbrev: COORD_REMAP_SENSE_AXIS27; Default: 27(-); DF Addr: 035Bh; SRAM Offset: 02F7h ; SubClass Offset : 60
	///     Coordinate Remap Sense Axis27:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis27;                                 
	/// 62. Abbrev: COORD_REMAP_SENSE_AXIS28; Default: 28(-); DF Addr: 035Ch; SRAM Offset: 02F8h ; SubClass Offset : 61
	///     Coordinate Remap Sense Axis28:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis28;                                 
	/// 63. Abbrev: COORD_REMAP_SENSE_AXIS29; Default: 29(-); DF Addr: 035Dh; SRAM Offset: 02F9h ; SubClass Offset : 62
	///     Coordinate Remap Sense Axis29:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis29;                                 
	/// 64. Abbrev: COORD_REMAP_SENSE_AXIS30; Default: 30(-); DF Addr: 035Eh; SRAM Offset: 02FAh ; SubClass Offset : 63
	///     Coordinate Remap Sense Axis30:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis30;                                 
	/// 65. Abbrev: COORD_REMAP_SENSE_AXIS31; Default: 31(-); DF Addr: 035Fh; SRAM Offset: 02FBh ; SubClass Offset : 64
	///     Coordinate Remap Sense Axis31:Pin Unit Coordinate.
	U1 bCoordRemapSenseAxis31;                                 
} SubClassCoordRemapSetupType;	///< Total = 65 Bytes

/// =============================================================///
/// Data flash Class #24 => CoordRemap
/// =============================================================///
typedef struct PACK ClassCoordRemapSt
{
	SubClassCoordRemapSetupType scCoordRemapSetup;
}ClassCoordRemapType;	///< Total = 65 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #44 => CoordLimit
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassCoordLimitSt
{
	///  1. Abbrev: COOR_LIMIT_ENABLE; Default: 0x00(-); DF Addr: 0380h; SRAM Offset: 02FCh ; SubClass Offset : 0
	///     0: Disable 1:Enable Border Coordinate Lock
	U1 bCoordLimitEnableType;                                  
	///  2. Abbrev: SEN_COOR_LOCK_NEAR; Default: 0x18(-); DF Addr: 0381h; SRAM Offset: 02FDh ; SubClass Offset : 1
	///     the Sense-Axis Near Border Lock Pixel
	U1 bSenseBorderLockNear;                                   
	///  3. Abbrev: SEN_COOR_LOCK_FAR; Default: 0x18(-); DF Addr: 0382h; SRAM Offset: 02FEh ; SubClass Offset : 2
	///     the Sense-Axis Far Border Lock Pixel
	U1 bSenseBorderLockFar;                                    
	///  4. Abbrev: DRI_COOR_LOCK_NEAR; Default: 0x18(-); DF Addr: 0383h; SRAM Offset: 02FFh ; SubClass Offset : 3
	///     the Drive-Axis Near Border Lock Pixel
	U1 bDriveBorderLockNear;                                   
	///  5. Abbrev: DRI_COOR_LOCK_FAR; Default: 0x18(-); DF Addr: 0384h; SRAM Offset: 0300h ; SubClass Offset : 4
	///     the Drive-Axis Far Border Lock Pixel
	U1 bDriveBorderLockFar;                                    
	///  6. Abbrev: COOR_REPORT_LOCK_EDGE_PIXEL; Default: 0x06(-); DF Addr: 0385h; SRAM Offset: 0301h ; SubClass Offset : 5
	///     the Border Lock Pixel
	U1 bCoordLockEdgePixel;                                    
} SubClassCoordLimitType;	///< Total = 6 Bytes

/// =============================================================///
/// Data flash Class #25 => CoorComp
/// =============================================================///
typedef struct PACK ClassCoorCompSt
{
	SubClassCoordLimitType scCoordLimit;
}ClassCoorCompType;	///< Total = 6 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #45 => AlgoFrameRate
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoFrameRateSt
{
	///  1. Abbrev: TIMER_EN; Default: 0x00(-); DF Addr: 0386h; SRAM Offset: 0302h ; SubClass Offset : 0
	///     Enable Timer Function
	U1 bTimerEnable;                                           
	///  2. Abbrev: TIMER_PERIOD; Default: 0x0FA0(-); DF Addr: 0387h; SRAM Offset: 0303h ; SubClass Offset : 1
	///     Timer Perid
	U2 wTimerPeriod;                                           
	///  3. Abbrev: GREEN_MODE_TIMER_PERIOD; Default: 24220(-); DF Addr: 0389h; SRAM Offset: 0305h ; SubClass Offset : 3
	///     Green mode Timer Perid
	U2 wGreenTimerPeriod;                                      
	///  4. Abbrev: YELLOW_MODE_TIMER_PERIOD; Default: 8000(-); DF Addr: 038Bh; SRAM Offset: 0307h ; SubClass Offset : 5
	///     Yellow mode Timer Perid
	U2 wYellowTimerPeriod;                                     
} SubClassAlgoFrameRateType;	///< Total = 7 Bytes

/// =============================================================///
/// Data flash Class #26 => FrameRate
/// =============================================================///
typedef struct PACK ClassFrameRateSt
{
	SubClassAlgoFrameRateType scAlgoFrameRate;
}ClassFrameRateType;	///< Total = 7 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #46 => IRsetup
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassIRsetupSt
{
	///  1. Abbrev: IR_SENSITIVITY; Default: 100(-); DF Addr: 038Dh; SRAM Offset: 0309h ; SubClass Offset : 0
	///     IR Sensitivity level:[0~255]
	U1 bIRSensitivity;                                         
} SubClassIRsetupType;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #27 => IRsetupPage
/// =============================================================///
typedef struct PACK ClassIRsetupPageSt
{
	SubClassIRsetupType scIRsetup;
}ClassIRsetupPageType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #47 => FingerDist
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassFingerDistSt
{
	///  1. Abbrev: FORCE_FINGER_UP_DIST; Default: 0x0200(-); DF Addr: 038Eh; SRAM Offset: 030Ah ; SubClass Offset : 0
	///     Finger distance : the distance that force finger up
	U2 wForceFingerUpDist;                                     
	///  2. Abbrev: FORCE_NO_SMOOTH_DIST; Default: 0x0100(-); DF Addr: 0390h; SRAM Offset: 030Ch ; SubClass Offset : 2
	///     Finger distance : the distance that force no smooth
	U2 wForceNoSmoothDist;                                     
} SubClassFingerDistType;	///< Total = 4 Bytes

/// =============================================================///
/// Data flash Class #28 => FingerDist
/// =============================================================///
typedef struct PACK ClassFingerDistSt
{
	SubClassFingerDistType scFingerDist;
}ClassFingerDistType;	///< Total = 4 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #48 => IForceSetup
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassIForceSetupSt
{
	///  1. Abbrev: FORCE_AREA_TH; Default: 0x46(-); DF Addr: 0392h; SRAM Offset: 030Eh ; SubClass Offset : 0
	///     Force area threshold
	U1 bForceAreaTH;                                           
	///  2. Abbrev: ENTRY_FORCE_TH; Default: 0xA0(-); DF Addr: 0393h; SRAM Offset: 030Fh ; SubClass Offset : 1
	///     Entry Force threshold
	U1 bEntryForceTH;                                          
	///  3. Abbrev: CELL_SUM_NEW_WEIGHT; Default: 0x01(-); DF Addr: 0394h; SRAM Offset: 0310h ; SubClass Offset : 2
	///     CELL_SUM_NEW_WEIGHT
	U1 bCellSumNewWeight;                                      
	///  4. Abbrev: CELL_SUM_OLD_WEIGHT; Default: 0x01(-); DF Addr: 0395h; SRAM Offset: 0311h ; SubClass Offset : 3
	///     CELL_SUM_OLD_WEIGHT
	U1 bCellSumOldWeight;                                      
	///  5. Abbrev: SUM_GAIN; Default: 0x0A(-); DF Addr: 0396h; SRAM Offset: 0312h ; SubClass Offset : 4
	///     Sum Gain
	U1 bSumGain;                                               
	///  6. Abbrev: FORCE_DEV_TH; Default: 0x00(-); DF Addr: 0397h; SRAM Offset: 0313h ; SubClass Offset : 5
	///     Force dev threshold
	U1 bForceDevTH;                                            
	///  7. Abbrev: PEAK_Y1_CELLGAIN; Default: 0x06(-); DF Addr: 0398h; SRAM Offset: 0314h ; SubClass Offset : 6
	///     Peak Y1 Cell gain
	U1 bPeakY1CellGain;                                        
	///  8. Abbrev: PEAK_Y2_CELLGAIN; Default: 0x04(-); DF Addr: 0399h; SRAM Offset: 0315h ; SubClass Offset : 7
	///     Peak Y2 Cell gain
	U1 bPeakY2CellGain;                                        
	///  9. Abbrev: PEAK_Y3_CELLGAIN; Default: 0x04(-); DF Addr: 039Ah; SRAM Offset: 0316h ; SubClass Offset : 8
	///     Peak Y3 Cell gain
	U1 bPeakY3CellGain;                                        
	/// 10. Abbrev: PEAK_Y4_CELLGAIN; Default: 0x04(-); DF Addr: 039Bh; SRAM Offset: 0317h ; SubClass Offset : 9
	///     Peak Y4 Cell gain
	U1 bPeakY4CellGain;                                        
	/// 11. Abbrev: PEAK_Y5_CELLGAIN; Default: 0x04(-); DF Addr: 039Ch; SRAM Offset: 0318h ; SubClass Offset : 10
	///     Peak Y5 Cell gain
	U1 bPeakY5CellGain;                                        
	/// 12. Abbrev: PEAK_Y6_CELLGAIN; Default: 0x04(-); DF Addr: 039Dh; SRAM Offset: 0319h ; SubClass Offset : 11
	///     Peak Y6 Cell gain
	U1 bPeakY6CellGain;                                        
	/// 13. Abbrev: PEAK_Y7_CELLGAIN; Default: 0x04(-); DF Addr: 039Eh; SRAM Offset: 031Ah ; SubClass Offset : 12
	///     Peak Y7 Cell gain
	U1 bPeakY7CellGain;                                        
	/// 14. Abbrev: PEAK_Y8_CELLGAIN; Default: 0x04(-); DF Addr: 039Fh; SRAM Offset: 031Bh ; SubClass Offset : 13
	///     Peak Y8 Cell gain
	U1 bPeakY8CellGain;                                        
	/// 15. Abbrev: PEAK_Y9_CELLGAIN; Default: 0x04(-); DF Addr: 03A0h; SRAM Offset: 031Ch ; SubClass Offset : 14
	///     Peak Y9 Cell gain
	U1 bPeakY9CellGain;                                        
	/// 16. Abbrev: PEAK_Y10_CELLGAIN; Default: 0x04(-); DF Addr: 03A1h; SRAM Offset: 031Dh ; SubClass Offset : 15
	///     Peak Y10 Cell gain
	U1 bPeakY10CellGain;                                       
	/// 17. Abbrev: PEAK_Y11_CELLGAIN; Default: 0x05(-); DF Addr: 03A2h; SRAM Offset: 031Eh ; SubClass Offset : 16
	///     Peak Y11 Cell gain
	U1 bPeakY11CellGain;                                       
	/// 18. Abbrev: PEAK_Y12_CELLGAIN; Default: 0x05(-); DF Addr: 03A3h; SRAM Offset: 031Fh ; SubClass Offset : 17
	///     Peak Y12 Cell gain
	U1 bPeakY12CellGain;                                       
	/// 19. Abbrev: PEAK_Y13_CELLGAIN; Default: 0x06(-); DF Addr: 03A4h; SRAM Offset: 0320h ; SubClass Offset : 18
	///     Peak Y13 Cell gain
	U1 bPeakY13CellGain;                                       
	/// 20. Abbrev: PEAK_Y14_CELLGAIN; Default: 0x06(-); DF Addr: 03A5h; SRAM Offset: 0321h ; SubClass Offset : 19
	///     Peak Y14 Cell gain
	U1 bPeakY14CellGain;                                       
	/// 21. Abbrev: PEAK_Y15_CELLGAIN; Default: 0x08(-); DF Addr: 03A6h; SRAM Offset: 0322h ; SubClass Offset : 20
	///     Peak Y15 Cell gain
	U1 bPeakY15CellGain;                                       
	/// 22. Abbrev: PEAK_Y16_CELLGAIN; Default: 0x08(-); DF Addr: 03A7h; SRAM Offset: 0323h ; SubClass Offset : 21
	///     Peak Y16 Cell gain
	U1 bPeakY16CellGain;                                       
	/// 23. Abbrev: PEAK_Y17_CELLGAIN; Default: 0x05(-); DF Addr: 03A8h; SRAM Offset: 0324h ; SubClass Offset : 22
	///     Peak Y17 Cell gain
	U1 bPeakY17CellGain;                                       
	/// 24. Abbrev: PEAK_Y18_CELLGAIN; Default: 0x06(-); DF Addr: 03A9h; SRAM Offset: 0325h ; SubClass Offset : 23
	///     Peak Y18 Cell gain
	U1 bPeakY18CellGain;                                       
	/// 25. Abbrev: PEAK_Y19_CELLGAIN; Default: 0x06(-); DF Addr: 03AAh; SRAM Offset: 0326h ; SubClass Offset : 24
	///     Peak Y19 Cell gain
	U1 bPeakY19CellGain;                                       
	/// 26. Abbrev: PEAK_Y20_CELLGAIN; Default: 0x08(-); DF Addr: 03ABh; SRAM Offset: 0327h ; SubClass Offset : 25
	///     Peak Y20 Cell gain
	U1 bPeakY20CellGain;                                       
	/// 27. Abbrev: PEAK_Y21_CELLGAIN; Default: 0x08(-); DF Addr: 03ACh; SRAM Offset: 0328h ; SubClass Offset : 26
	///     Peak Y21 Cell gain
	U1 bPeakY21CellGain;                                       
	/// 28. Abbrev: PEAK_X1_CELLGAIN; Default: 0x20(-); DF Addr: 03ADh; SRAM Offset: 0329h ; SubClass Offset : 27
	///     Peak X1 Cell gain
	U1 bPeakX1CellGain;                                        
	/// 29. Abbrev: PEAK_X2_CELLGAIN; Default: 0x20(-); DF Addr: 03AEh; SRAM Offset: 032Ah ; SubClass Offset : 28
	///     Peak X2 Cell gain
	U1 bPeakX2CellGain;                                        
	/// 30. Abbrev: PEAK_X3_CELLGAIN; Default: 0x20(-); DF Addr: 03AFh; SRAM Offset: 032Bh ; SubClass Offset : 29
	///     Peak X3 Cell gain
	U1 bPeakX3CellGain;                                        
	/// 31. Abbrev: PEAK_X4_CELLGAIN; Default: 0x20(-); DF Addr: 03B0h; SRAM Offset: 032Ch ; SubClass Offset : 30
	///     Peak X4 Cell gain
	U1 bPeakX4CellGain;                                        
	/// 32. Abbrev: PEAK_X5_CELLGAIN; Default: 0x20(-); DF Addr: 03B1h; SRAM Offset: 032Dh ; SubClass Offset : 31
	///     Peak X5 Cell gain
	U1 bPeakX5CellGain;                                        
	/// 33. Abbrev: PEAK_X6_CELLGAIN; Default: 0x20(-); DF Addr: 03B2h; SRAM Offset: 032Eh ; SubClass Offset : 32
	///     Peak X6 Cell gain
	U1 bPeakX6CellGain;                                        
	/// 34. Abbrev: PEAK_X7_CELLGAIN; Default: 0x20(-); DF Addr: 03B3h; SRAM Offset: 032Fh ; SubClass Offset : 33
	///     Peak X7 Cell gain
	U1 bPeakX7CellGain;                                        
	/// 35. Abbrev: PEAK_X8_CELLGAIN; Default: 0x20(-); DF Addr: 03B4h; SRAM Offset: 0330h ; SubClass Offset : 34
	///     Peak X8 Cell gain
	U1 bPeakX8CellGain;                                        
	/// 36. Abbrev: PEAK_X9_CELLGAIN; Default: 0x20(-); DF Addr: 03B5h; SRAM Offset: 0331h ; SubClass Offset : 35
	///     Peak X9 Cell gain
	U1 bPeakX9CellGain;                                        
	/// 37. Abbrev: PEAK_X10_CELLGAIN; Default: 0x20(-); DF Addr: 03B6h; SRAM Offset: 0332h ; SubClass Offset : 36
	///     Peak X10 Cell gain
	U1 bPeakX10CellGain;                                       
	/// 38. Abbrev: PEAK_X11_CELLGAIN; Default: 0x20(-); DF Addr: 03B7h; SRAM Offset: 0333h ; SubClass Offset : 37
	///     Peak X11 Cell gain
	U1 bPeakX11CellGain;                                       
	/// 39. Abbrev: PEAK_X12_CELLGAIN; Default: 0x20(-); DF Addr: 03B8h; SRAM Offset: 0334h ; SubClass Offset : 38
	///     Peak X12 Cell gain
	U1 bPeakX12CellGain;                                       
} SubClassIForceSetupType;	///< Total = 39 Bytes

/// =============================================================///
/// Data flash Class #29 => IForceSetup
/// =============================================================///
typedef struct PACK ClassIForceSetupSt
{
	SubClassIForceSetupType scIForceSetup;
}ClassIForceSetupType;	///< Total = 39 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #49 => DummyPage7
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDummyPage7St
{
	///  1. Abbrev: PAGE7_DUMMY_BYTE0; Default: 0xFF(-); DF Addr: 03B9h; SRAM Offset: 0335h ; SubClass Offset : 0
	///     PAGE 7 : DUMMY BYTE
	U1 bPage7DummyByte;                                        
} SubClassDummyPage7Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #30 => DummyPage7
/// =============================================================///
typedef struct PACK ClassDummyPage7St
{
	SubClassDummyPage7Type scDummyPage7;
}ClassDummyPage7Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #50 => DFProtect
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDFProtectSt
{
	///  1. Abbrev: DF_PROTECT; Default: 0xFFFF(-); DF Addr: 0400h; SRAM Offset: 0336h ; SubClass Offset : 0
	///     
	U2 wDFProtect;                                             
} SubClassDFProtectType;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #31 => DFProtect
/// =============================================================///
typedef struct PACK ClassDFProtectSt
{
	SubClassDFProtectType scDFProtect;
}ClassDFProtectType;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #51 => NearToFarContri
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNearToFarContriSt
{
} SubClassNearToFarContriType;	///< Total = 0 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #52 => NearToFar1
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassNearToFar1St
{
	///  1. Abbrev: NEAR_FAR_RATIO_LAST_E1_TO_E2; Default: 0x00(-); DF Addr: 0402h; SRAM Offset: 0338h ; SubClass Offset : 0
	///     
	U1 bNFRatioLastE1toE2;                                     
	///  2. Abbrev: NEAR_FAR_RATIO_LAST_E2_TO_E1; Default: 0x00(-); DF Addr: 0403h; SRAM Offset: 0339h ; SubClass Offset : 1
	///     
	U1 bNFRatioLastE2toE1;                                     
	///  3. Abbrev: NEAR_FAR_RATIO_LAST_E3_TO_E4; Default: 0x00(-); DF Addr: 0404h; SRAM Offset: 033Ah ; SubClass Offset : 2
	///     
	U1 bNFRatioLastE3toE4;                                     
	///  4. Abbrev: NEAR_FAR_RATIO_LAST_E4_TO_E3; Default: 0x00(-); DF Addr: 0405h; SRAM Offset: 033Bh ; SubClass Offset : 3
	///     
	U1 bNFRatioLastE4toE3;                                     
	///  5. Abbrev: NEAR_FAR_RATIO_LAST_E5_TO_E6; Default: 0x00(-); DF Addr: 0406h; SRAM Offset: 033Ch ; SubClass Offset : 4
	///     
	U1 bNFRatioLastE5toE6;                                     
	///  6. Abbrev: NEAR_FAR_RATIO_LAST_E6_TO_E5; Default: 0x00(-); DF Addr: 0407h; SRAM Offset: 033Dh ; SubClass Offset : 5
	///     
	U1 bNFRatioLastE6toE5;                                     
	///  7. Abbrev: NEAR_FAR_RATIO_LAST_O2_TO_O3; Default: 0x00(-); DF Addr: 0408h; SRAM Offset: 033Eh ; SubClass Offset : 6
	///     
	U1 bNFRatioLastO2toO3;                                     
	///  8. Abbrev: NEAR_FAR_RATIO_LAST_O3_TO_O2; Default: 0x00(-); DF Addr: 0409h; SRAM Offset: 033Fh ; SubClass Offset : 7
	///     
	U1 bNFRatioLastO3toO2;                                     
	///  9. Abbrev: NEAR_FAR_RATIO_LAST_O4_TO_O5; Default: 0x00(-); DF Addr: 040Ah; SRAM Offset: 0340h ; SubClass Offset : 8
	///     
	U1 bNFRatioLastO4toO5;                                     
	/// 10. Abbrev: NEAR_FAR_RATIO_LAST_O5_TO_O4; Default: 0x00(-); DF Addr: 040Bh; SRAM Offset: 0341h ; SubClass Offset : 9
	///     
	U1 bNFRatioLastO5toO4;                                     
	/// 11. Abbrev: NEAR_FAR_RATIO_LAST_O6_TO_O7; Default: 0x00(-); DF Addr: 040Ch; SRAM Offset: 0342h ; SubClass Offset : 10
	///     
	U1 bNFRatioLastO6toO7;                                     
	/// 12. Abbrev: NEAR_FAR_RATIO_LAST_O7_TO_O6; Default: 0x00(-); DF Addr: 040Dh; SRAM Offset: 0343h ; SubClass Offset : 11
	///     
	U1 bNFRatioLastO7toO6;                                     
	/// 13. Abbrev: NEAR_FAR_RATIO_LAST_O3O4_TO_O1O2; Default: 0x00(-); DF Addr: 040Eh; SRAM Offset: 0344h ; SubClass Offset : 12
	///     
	U1 bNFRatioLastO3O4toO1O2;                                 
	/// 14. Abbrev: NEAR_FAR_RATIO_LAST_O4E4_TO_O3; Default: 0x00(-); DF Addr: 040Fh; SRAM Offset: 0345h ; SubClass Offset : 13
	///     
	U1 bNFRatioLastO4E4toO3;                                   
	/// 15. Abbrev: NEAR_FAR_RATIO_LAST_O4E4_TO_E5; Default: 0x00(-); DF Addr: 0410h; SRAM Offset: 0346h ; SubClass Offset : 14
	///     
	U1 bNFRatioLastO4E4toE5;                                   
	/// 16. Abbrev: NEAR_FAR_RATIO_LAST_E7O7TO_E6; Default: 0x00(-); DF Addr: 0411h; SRAM Offset: 0347h ; SubClass Offset : 15
	///     
	U1 bNFRatioLastE7O7toE6;                                   
	/// 17. Abbrev: NEAR_FAR_RATIO_LAST_E7O7TO_O8; Default: 0x00(-); DF Addr: 0412h; SRAM Offset: 0348h ; SubClass Offset : 16
	///     
	U1 bNFRatioLastE7O7toO8;                                   
	/// 18. Abbrev: NEAR_FAR_RATIO_LAST_O10E10_TO_O9; Default: 0x00(-); DF Addr: 0413h; SRAM Offset: 0349h ; SubClass Offset : 17
	///     
	U1 bNFRatioLastO10E10toO9;                                 
	/// 19. Abbrev: NEAR_FAR_RATIO_LAST_O10E10_TO_E11; Default: 0x00(-); DF Addr: 0414h; SRAM Offset: 034Ah ; SubClass Offset : 18
	///     
	U1 bNFRatioLastO10E10toE11;                                
	/// 20. Abbrev: NEAR_FAR_RATIO_LAST_LEFT_COL2_E1O1_TO_LEFT_COL1_E1; Default: 0x00(-); DF Addr: 0415h; SRAM Offset: 034Bh ; SubClass Offset : 19
	///     
	U1 bNFRatioLastCol2E1O1toLeftCol1E1;                       
	/// 21. Abbrev: NEAR_FAR_RATIO_LAST_LEFT_COL2_E1O1_TO_LEFT_COL1_O1; Default: 0x00(-); DF Addr: 0416h; SRAM Offset: 034Ch ; SubClass Offset : 20
	///     
	U1 bNFRatioLastCol2E1O1toLeftCol1O1;                       
} SubClassNearToFar1Type;	///< Total = 21 Bytes

/// =============================================================///
/// Data flash Class #32 => TpCrossTalkFix1
/// =============================================================///
typedef struct PACK ClassTpCrossTalkFix1St
{
	SubClassNearToFarContriType scNearToFarContri;
	SubClassNearToFar1Type scNearToFar1;
}ClassTpCrossTalkFix1Type;	///< Total = 21 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #53 => wPnParam
///-------------------------------------------------------------------------------///
typedef struct PACK SubClasswPnParamSt
{
	///  1. Abbrev: RX_SINE_SIZE; Default: 0x0031(-); DF Addr: 0417h; SRAM Offset: 034Dh ; SubClass Offset : 0
	///     
	U2 wRxSineSize;                                            
	///  2. Abbrev: RX_COEF_SIZE; Default: 0x0061(-); DF Addr: 0419h; SRAM Offset: 034Fh ; SubClass Offset : 2
	///     
	U2 wRxCoefSize;                                            
	///  3. Abbrev: TX_SINE_SIZE; Default: 0x0188(-); DF Addr: 041Bh; SRAM Offset: 0351h ; SubClass Offset : 4
	///     
	U2 wTxSineSize;                                            
	///  4. Abbrev: SGN_EXT_SIZE; Default: 0x0001(-); DF Addr: 041Dh; SRAM Offset: 0353h ; SubClass Offset : 6
	///     
	U2 wSgnExtSize;                                            
	///  5. Abbrev: SGN_EXT_BYTE_0; Default: 0x05(-); DF Addr: 041Fh; SRAM Offset: 0355h ; SubClass Offset : 8
	///     
	U1 bSgnExtByte0;                                           
} SubClasswPnParamType;	///< Total = 9 Bytes

/// =============================================================///
/// Data flash Class #33 => TableLength
/// =============================================================///
typedef struct PACK ClassTableLengthSt
{
	SubClasswPnParamType scPnParam;
}ClassTableLengthType;	///< Total = 9 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #54 => Gesture
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassGestureSt
{
	///  1. Abbrev: GESTURE_EN; Default: 0x01(-); DF Addr: 0420h; SRAM Offset: 0356h ; SubClass Offset : 0
	///     
	U1 bGestureEn;                                             
	///  2. Abbrev: GESTURE_ACC; Default: 0x01(-); DF Addr: 0421h; SRAM Offset: 0357h ; SubClass Offset : 1
	///     
	U1 bGestureAccuracy;                                       
} SubClassGestureType;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #34 => Gesture
/// =============================================================///
typedef struct PACK ClassGestureSt
{
	SubClassGestureType scGesture;
}ClassGestureType;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #55 => AlgoData
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoDataSt
{
	///  1. Abbrev: PALM_DEV_TH; Default: 0x96(-); DF Addr: 0422h; SRAM Offset: 0358h ; SubClass Offset : 0
	///     [DYN] Palm Dev Threshold
	U1 bPalmDevTH;                                             
	///  2. Abbrev: PALM_DEV_CNT; Default: 0xFF(-); DF Addr: 0423h; SRAM Offset: 0359h ; SubClass Offset : 1
	///     [DYN] Palm Dev Count
	U1 bPalmDevCount;                                          
	///  3. Abbrev: FINGER_SEARCH_ALGO; Default: 0x00(-); DF Addr: 0424h; SRAM Offset: 035Ah ; SubClass Offset : 2
	///     [DYN] Finger Search Algorithm: 0: NINE SQUARE ; 1 : ADV NINE SQUARE ;
	U1 bFingerSearchAlgo;                                      
	///  4. Abbrev: RECAL_EN; Default: 0x01(-); DF Addr: 0425h; SRAM Offset: 035Bh ; SubClass Offset : 3
	///     [DYN]  Enable Re-Calibration
	U1 bEnableReCalibration;                                   
	///  5. Abbrev: RECAL_TYPE; Default: 0x00(-); DF Addr: 0426h; SRAM Offset: 035Ch ; SubClass Offset : 4
	///     [DYN] Re-Calibration Type 0: Check Every Round ,1:Check Only When Tacking-Base
	U1 bReCalibrationType;                                     
	///  6. Abbrev: BASE_TRACK_PERIOD; Default: 0x04(-); DF Addr: 0427h; SRAM Offset: 035Dh ; SubClass Offset : 5
	///     [DYN]Tacking-Base Period
	U1 bBaseTrackPeriod;                                       
	///  7. Abbrev: FORCE_BASE_TRACK_MAX_STEP; Default: 0x06(-); DF Addr: 0428h; SRAM Offset: 035Eh ; SubClass Offset : 6
	///     [DYN]Force touch Tacking-Base max step
	U1 bForceBaseTrackMaxStep;                                 
	///  8. Abbrev: FORCE_BASE_TRACK_MIN_STEP; Default: 0x03(-); DF Addr: 0429h; SRAM Offset: 035Fh ; SubClass Offset : 7
	///     [DYN]Force touch Tacking-Base min step
	U1 bForceBaseTrackMinStep;                                 
	///  9. Abbrev: RECAL_DET_CNT; Default: 0x1E(-); DF Addr: 042Ah; SRAM Offset: 0360h ; SubClass Offset : 8
	///     [DYN] Re-calibration detect count. the negtive value of dev shall be visible for # times
	U1 bReCalDetCnt;                                           
	/// 10. Abbrev: RECAL_DET_ACC_CNT; Default: 0x04(-); DF Addr: 042Bh; SRAM Offset: 0361h ; SubClass Offset : 9
	///     [DYN] Re-calibration detect count. the negtive value of dev shall be visible for # times
	U1 bReCalDetAccCnt;                                        
	/// 11. Abbrev: BASE_TRACK_PERIOD_RST; Default: 0x20(-); DF Addr: 042Ch; SRAM Offset: 0362h ; SubClass Offset : 10
	///     Track Base Period  After reset unstable state
	U1 bBaseTrackPeriodWhenRst;                                
	/// 12. Abbrev: RESET_UNSTABLE_PERIOD; Default: 0x0020(-); DF Addr: 042Dh; SRAM Offset: 0363h ; SubClass Offset : 11
	///     The unstable state after reset (unit: Round , Frame)
	U2 wResetUnstablePeriod;                                   
	/// 13. Abbrev: BASE_TRACK_PERIOD_REK; Default: 0x80(-); DF Addr: 042Fh; SRAM Offset: 0365h ; SubClass Offset : 13
	///     Track Base Period  After reset unstable state
	U1 bBaseTrackPeriodWhenReK;                                
	/// 14. Abbrev: REK_UNSTABLE_PERIOD; Default: 0x0008(-); DF Addr: 0430h; SRAM Offset: 0366h ; SubClass Offset : 14
	///     The unstable state after reset (unit: Round , Frame)
	U2 wReKUnstablekPeriod;                                    
	/// 15. Abbrev: ALG_FUNC_CTRL; Default: 0x50(-); DF Addr: 0432h; SRAM Offset: 0368h ; SubClass Offset : 16
	///     [DYN] ALgorithm Function Bit Control : [0][1][2][3][4][5][6][7] for different Function Control ,[7]=1:output Raw Data
	U1 bAlgFuncCtrl;                                           
	/// 16. Abbrev: ALG_FUNC_CTRL1; Default: 0x01(-); DF Addr: 0433h; SRAM Offset: 0369h ; SubClass Offset : 17
	///     [DYN] ALgorithm Function Bit Control 1 : [0][1][2][3][4][5][6][7] for different Function Control ,[7]=1:output Raw Data
	U1 bAlgFuncCtrl1;                                          
	/// 17. Abbrev: SEN_DEV_PROC_INTEN_CTRL; Default: 0x01(-); DF Addr: 0434h; SRAM Offset: 036Ah ; SubClass Offset : 18
	///     [DYN] Intensity Control for Sense Axis Dev Process (0~7), 0:The Lowest Intensity, 7: The Highest Intensity
	U1 bSenDevProcIntenCtrl;                                   
} SubClassAlgoDataType;	///< Total = 19 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #56 => SmoothCtrl
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassSmoothCtrlSt
{
	///  1. Abbrev: SMOOTH_DIST1; Default: 0x10(-); DF Addr: 0435h; SRAM Offset: 036Bh ; SubClass Offset : 0
	///     [DYN] Smooth Distance #1
	U1 bSmoothDist1;                                           
	///  2. Abbrev: SMOOTH_DIST2; Default: 0x20(-); DF Addr: 0436h; SRAM Offset: 036Ch ; SubClass Offset : 1
	///     [DYN] Smooth Distance #2
	U1 bSmoothDist2;                                           
	///  3. Abbrev: SMOOTH_DIST3; Default: 0x80(-); DF Addr: 0437h; SRAM Offset: 036Dh ; SubClass Offset : 2
	///     [DYN] Smooth Distance #3
	U1 bSmoothDist3;                                           
	///  4. Abbrev: SMOOTH_DIST4; Default: 0xFF(-); DF Addr: 0438h; SRAM Offset: 036Eh ; SubClass Offset : 3
	///     [DYN] Smooth Distance #4
	U1 bSmoothDist4;                                           
	///  5. Abbrev: SMOOTH_LEVEL1; Default: 0x07(-); DF Addr: 0439h; SRAM Offset: 036Fh ; SubClass Offset : 4
	///     [DYN] Smooth Level #1
	U1 bSmoothLevel1;                                          
	///  6. Abbrev: SMOOTH_LEVEL2; Default: 0x06(-); DF Addr: 043Ah; SRAM Offset: 0370h ; SubClass Offset : 5
	///     [DYN] Smooth Level #2
	U1 bSmoothLevel2;                                          
	///  7. Abbrev: SMOOTH_LEVEL3; Default: 0x05(-); DF Addr: 043Bh; SRAM Offset: 0371h ; SubClass Offset : 6
	///     [DYN] Smooth Level #3
	U1 bSmoothLevel3;                                          
	///  8. Abbrev: SMOOTH_LEVEL4; Default: 0x04(-); DF Addr: 043Ch; SRAM Offset: 0372h ; SubClass Offset : 7
	///     [DYN] Smooth Level #4
	U1 bSmoothLevel4;                                          
	///  9. Abbrev: SMOOTH_ENABLE; Default: 0x01(-); DF Addr: 043Dh; SRAM Offset: 0373h ; SubClass Offset : 8
	///     [DYN] Dynamic Smooth Enable
	U1 bDynSmoothEnable;                                       
	/// 10. Abbrev: SMOOTH_LEVEL_SHIFT; Default: 0x06(-); DF Addr: 043Eh; SRAM Offset: 0374h ; SubClass Offset : 9
	///     [DYN] Smooth Level Shift
	U1 bSmoothLevelShift;                                      
	/// 11. Abbrev: SMOOTH_LEVEL_COAXIS; Default: 0x02(-); DF Addr: 043Fh; SRAM Offset: 0375h ; SubClass Offset : 10
	///     [DYN] Smooth Level Shift for CoAxis
	U1 bSmoothLevelCoAxis;                                     
	/// 12. Abbrev: SMOOTH_DEBOUNCE; Default: 0x08(-); DF Addr: 0440h; SRAM Offset: 0376h ; SubClass Offset : 11
	///     [DYN] Smooth debounce
	U1 bSmoothDebounce;                                        
	/// 13. Abbrev: TIME_SMOOTH_ENABLE; Default: 0x01(-); DF Addr: 0441h; SRAM Offset: 0377h ; SubClass Offset : 12
	///     [DYN] Time Smooth enable
	U1 bTimeSmoothEnable;                                      
	/// 14. Abbrev: PAST_SMOOTH_WEIGHT; Default: 0x07(-); DF Addr: 0442h; SRAM Offset: 0378h ; SubClass Offset : 13
	///     [DYN] Past Smooth weight
	U1 bPastSmoothWeight;                                      
	/// 15. Abbrev: NOW_SMOOTH_WEIGHT; Default: 0x03(-); DF Addr: 0443h; SRAM Offset: 0379h ; SubClass Offset : 14
	///     [DYN] Now Smooth weight
	U1 bNowSmoothWeight;                                       
	/// 16. Abbrev: FUTURE_SMOOTH_WEIGHT; Default: 0x07(-); DF Addr: 0444h; SRAM Offset: 037Ah ; SubClass Offset : 15
	///     [DYN] Future Smooth weight
	U1 bFutureSmoothWeight;                                    
} SubClassSmoothCtrlType;	///< Total = 16 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #57 => MergeFinger
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassMergeFingerSt
{
	///  1. Abbrev: MERGE_GLUE_DIST; Default: 0x05(-); DF Addr: 0445h; SRAM Offset: 037Bh ; SubClass Offset : 0
	///     Merge Close Finger
	U1 bMergeDist;                                             
	///  2. Abbrev: MERGE_DIVID_DIST; Default: 0x0C(-); DF Addr: 0446h; SRAM Offset: 037Ch ; SubClass Offset : 1
	///     Merge Close Finger
	U1 bDividDist;                                             
	///  3. Abbrev: MERGE_RADIUS; Default: 0x02(-); DF Addr: 0447h; SRAM Offset: 037Dh ; SubClass Offset : 2
	///     Merge Close Finger Radius
	U1 bRadiusRange;                                           
} SubClassMergeFingerType;	///< Total = 3 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #58 => AlgoHeavyPressByInit
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoHeavyPressByInitSt
{
	///  1. Abbrev: HEAVY_PRESS_TRIG_TH; Default: 0x19(-); DF Addr: 0448h; SRAM Offset: 037Eh ; SubClass Offset : 0
	///     [DYN] The Heavy Press Threshold of Init/Mut Avg Dev
	U1 bHeavyPressByInitDevTh;                                 
	///  2. Abbrev: HEAVY_PRESS_ALG_DETECT_EN; Default: 0x00(-); DF Addr: 0449h; SRAM Offset: 037Fh ; SubClass Offset : 1
	///     enable Heavy Press algorithm(Average-Dev-Cut) By Average-Initial-Dev  -> 0 : Disable ; 1 : Enable
	U1 bHeavyPressAlgCtrl;                                     
	///  3. Abbrev: HEAVY_PRESS_CUT_DEV_RATIO; Default: 0x20(-); DF Addr: 044Ah; SRAM Offset: 0380h ; SubClass Offset : 2
	///     [DYN] Heavy press dev cut upper bound
	U1 bHeavyPressByInitCutRatio;                              
	///  4. Abbrev: HEAVY_PRESS_BY_INIT_DEBOUNCE; Default: 0x10(-); DF Addr: 044Bh; SRAM Offset: 0381h ; SubClass Offset : 3
	///     Finger Down Debounce When Init-Detected HeavyPress-On/HeavyPress-Off
	U1 bHeavyPressDebounce;                                    
} SubClassAlgoHeavyPressByInitType;	///< Total = 4 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #59 => HeavyPressCutAlg
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassHeavyPressCutAlgSt
{
	///  1. Abbrev: HEAVY_PRESS_TH; Default: 0x96(-); DF Addr: 044Ch; SRAM Offset: 0382h ; SubClass Offset : 0
	///     [DYN] Heavy Press Dev Sum Threshold
	U1 bHeavyPressDevTh;                                       
	///  2. Abbrev: HEAVY_PRESS_EN; Default: 0x00(-); DF Addr: 044Dh; SRAM Offset: 0383h ; SubClass Offset : 1
	///     enable Heavy Press algorithm(Average-Dev-Cut) By Max-Mutual-Dev  -> 0 : Disable ; 1 : Enable
	U1 bHeavyPressEnable;                                      
} SubClassHeavyPressCutAlgType;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #60 => AlgoChargerMode
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoChargerModeSt
{
	///  1. Abbrev: CHARGER_MUT_CUT_DEV; Default: 0x20(-); DF Addr: 044Eh; SRAM Offset: 0384h ; SubClass Offset : 0
	///     [DYN] Charger Mode Cut Dev at Mutual Scan
	U1 bChargerMutCutDev;                                      
	///  2. Abbrev: CHARGER_INIT_CUT_DEV; Default: 0x10(-); DF Addr: 044Fh; SRAM Offset: 0385h ; SubClass Offset : 1
	///     [DYN] Charger Mode Cut Dev at Init Scan
	U1 bChargerInitCutDev;                                     
	///  3. Abbrev: CHARGER_INIT_POST_CUT_DEV; Default: 0x14(-); DF Addr: 0450h; SRAM Offset: 0386h ; SubClass Offset : 2
	///     [DYN] Charger Mode Post Cut Dev at Init Scan
	U1 bChargerInitPostCutDev;                                 
	///  4. Abbrev: DEV_PROC_MUT_CTRL; Default: 0xFF(-); DF Addr: 0451h; SRAM Offset: 0387h ; SubClass Offset : 3
	///     [DYN] Bit0:Sen Normal,Bit1:Sen Normal CoAxis,Bit2:Sen Charger,Bit3:Sen Charger CoAxis ; Bit4:Dri Normal,Bit5:Dri Normal CoAxis,Bit6:Dri Charger,Bit7:Dri Charger CoAxis
	U1 bDevProcessMode;                                        
	///  5. Abbrev: DRI_DEV_PROC_CNT; Default: 0x00(-Old Dev Process); DF Addr: 0452h; SRAM Offset: 0388h ; SubClass Offset : 4
	///     The non-zero dev value  > DRI_DEV_PROC_CNT do the process
	U1 bDriDevProcCnt;                                         
	///  6. Abbrev: DEV_PROC_INIT_CTRL; Default: 0x01(-); DF Addr: 0453h; SRAM Offset: 0389h ; SubClass Offset : 5
	///     Initial Dev do the Dev process
	U1 bDevProcModeInit;                                       
	///  7. Abbrev: DEV_PROC_INIT_CTRL_SCALE; Default: 0x04(-); DF Addr: 0454h; SRAM Offset: 038Ah ; SubClass Offset : 6
	///     The scaling control for the initial Dev process
	U1 bDevProcModeInitScale;                                  
	///  8. Abbrev: SEN_DEV_PROC_CNT; Default: 0x03(-); DF Addr: 0455h; SRAM Offset: 038Bh ; SubClass Offset : 7
	///     The non-zero dev value  > DRI_SEN_PROC_CNT do the process
	U1 bSenDevProcCnt;                                         
	///  9. Abbrev: SEN_DEV_PROC_LOW_BOUND; Default: 0x32(-); DF Addr: 0456h; SRAM Offset: 038Ch ; SubClass Offset : 8
	///     Sense dev process shmit trigger lower bound in % of the save dev
	U1 bSenDevProcLowerBound;                                  
	/// 10. Abbrev: DRI_DEV_PROC_LOW_BOUND; Default: 0x32(-); DF Addr: 0457h; SRAM Offset: 038Dh ; SubClass Offset : 9
	///     Drive dev process shmit trigger lower bound in % of the save dev
	U1 bDriDevProcLowerBound;                                  
} SubClassAlgoChargerModeType;	///< Total = 10 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #61 => AlgoTapping
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoTappingSt
{
	///  1. Abbrev: TAP_ACC_TH; Default: 0x00C8(-); DF Addr: 0458h; SRAM Offset: 038Eh ; SubClass Offset : 0
	///     [DYN] Tapping Count
	U2 wTapAccTH;                                              
	///  2. Abbrev: TAP_VEL_TH; Default: 0x0190(-); DF Addr: 045Ah; SRAM Offset: 0390h ; SubClass Offset : 2
	///     [DYN] Tapping Count
	U2 wTapVelTH;                                              
} SubClassAlgoTappingType;	///< Total = 4 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #62 => AlgoBaseAverage
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoBaseAverageSt
{
	///  1. Abbrev: BASE_AVG_CNT; Default: 0x01(-); DF Addr: 045Ch; SRAM Offset: 0392h ; SubClass Offset : 0
	///     [DYN] Base average counts, Default 8 times
	U1 bBaseAvgCnt;                                            
} SubClassAlgoBaseAverageType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #63 => AlgoPowerSave
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassAlgoPowerSaveSt
{
	///  1. Abbrev: POWER_SAVE_EN; Default: 0x00(-); DF Addr: 045Dh; SRAM Offset: 0393h ; SubClass Offset : 0
	///     [DYN] 1: Power Saving Mode Enable, 0: Disable
	U1 bPowerSaveEnable;                                       
	///  2. Abbrev: POWER_SAVE_TIMER; Default: 0xC8(-); DF Addr: 045Eh; SRAM Offset: 0394h ; SubClass Offset : 1
	///     [DYN] # of round with out finger than enter power saving mode
	U1 bPowerSaveTimer;                                        
	///  3. Abbrev: TIME_SEC_TO_GREEN_MODE; Default: 0x03(-); DF Addr: 045Fh; SRAM Offset: 0395h ; SubClass Offset : 2
	///     How many second does touch ic enter to green mode
	U1 bTimeToGreenMode;                                       
	///  4. Abbrev: TIME_SEC_TO_YELLOW_MODE; Default: 0x05(-); DF Addr: 0460h; SRAM Offset: 0396h ; SubClass Offset : 3
	///     How many second does touch ic enter to yellow mode
	U1 bTimeToYellowMode;                                      
	///  5. Abbrev: GREEN_MODE_DEBOUNCE; Default: 0x08(-); DF Addr: 0461h; SRAM Offset: 0397h ; SubClass Offset : 4
	///     Green mode debounce
	U1 bGreenModeDebounce;                                     
} SubClassAlgoPowerSaveType;	///< Total = 5 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #64 => ResetSpecialCtrl
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassResetSpecialCtrlSt
{
	///  1. Abbrev: RST_REPORT_ZERO_CNT; Default: 0x00(-); DF Addr: 0462h; SRAM Offset: 0398h ; SubClass Offset : 0
	///     Report zero-Finger Counts After Reset
	U1 bResetReportZeroCnt;                                    
	///  2. Abbrev: RST_RECAL_MIN_NEED_ROUND_AFTER_RST; Default: 0x00(-); DF Addr: 0463h; SRAM Offset: 0399h ; SubClass Offset : 1
	///     Min round need when exe ReCal or BaseTrack After Reset/Reset Done
	U1 bResetReCalMinNeedRound;                                
} SubClassResetSpecialCtrlType;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #35 => AlgoPage
/// =============================================================///
typedef struct PACK ClassAlgoPageSt
{
	SubClassAlgoDataType scAlgoData;
	SubClassSmoothCtrlType scSmoothCtrl;
	SubClassMergeFingerType scMergeFingerCtrl;
	SubClassAlgoHeavyPressByInitType scAlgoHeavyPressByInit;
	SubClassHeavyPressCutAlgType scHeavyPressCutAlg;
	SubClassAlgoChargerModeType scAlgoChargerMode;
	SubClassAlgoTappingType scAlgoTapping;
	SubClassAlgoBaseAverageType scAlgoBaseAverage;
	SubClassAlgoPowerSaveType scAlgoPowerSave;
	SubClassResetSpecialCtrlType scResetSpecialCtrl;
}ClassAlgoPageType;	///< Total = 66 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #65 => DummyPage8
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassDummyPage8St
{
	///  1. Abbrev: PAGE8_DUMMY_BYTE0; Default: 0xFF(-); DF Addr: 0464h; SRAM Offset: 039Ah ; SubClass Offset : 0
	///     PAGE 8 : DUMMY BYTE
	U1 bPage8DummyByte;                                        
} SubClassDummyPage8Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #36 => DummyPage8
/// =============================================================///
typedef struct PACK ClassDummyPage8St
{
	SubClassDummyPage8Type scDummyPage8;
}ClassDummyPage8Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #66 => wRxSineWord0
///-------------------------------------------------------------------------------///
typedef struct PACK SubClasswRxSineWord0St
{
	///  1. Abbrev: RxSine_WORD_0; Default: 0x0000(-); DF Addr: 0480h; SRAM Offset: 039Bh ; SubClass Offset : 0
	///     RX Sine0 Table Word[0]
	U2 wRxSineWord0;                                           
} SubClasswRxSineWord0Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #37 => RxSineWord0
/// =============================================================///
typedef struct PACK ClassRxSineWord0St
{
	SubClasswRxSineWord0Type scRxSineWord0;
}ClassRxSineWord0Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #67 => wRxSineWord64
///-------------------------------------------------------------------------------///
typedef struct PACK SubClasswRxSineWord64St
{
	///  1. Abbrev: RxSine_WORD_64; Default: 0x07FF(-); DF Addr: 0500h; SRAM Offset: 039Dh ; SubClass Offset : 0
	///     RX Sine0 Table Word[64]
	U2 wRxSineWord64;                                          
} SubClasswRxSineWord64Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #38 => RxSineWord64
/// =============================================================///
typedef struct PACK ClassRxSineWord64St
{
	SubClasswRxSineWord64Type scRxSineWord64;
}ClassRxSineWord64Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #68 => wRxSineWord128
///-------------------------------------------------------------------------------///
typedef struct PACK SubClasswRxSineWord128St
{
	///  1. Abbrev: RxSine_WORD_128; Default: 0xFFFF(-); DF Addr: 0580h; SRAM Offset: 039Fh ; SubClass Offset : 0
	///     RX Sine0 Table Word[128]
	U2 wRxSineWord128;                                         
} SubClasswRxSineWord128Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #39 => RxSineWord128
/// =============================================================///
typedef struct PACK ClassRxSineWord128St
{
	SubClasswRxSineWord128Type scRxSineWord128;
}ClassRxSineWord128Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #69 => wRxSineWord192
///-------------------------------------------------------------------------------///
typedef struct PACK SubClasswRxSineWord192St
{
	///  1. Abbrev: RxSine_WORD_192; Default: 0x0000(-); DF Addr: 0600h; SRAM Offset: 03A1h ; SubClass Offset : 0
	///     RX Sine0 Table Word[192]
	U2 wRxSineWord192;                                         
	///  2. Abbrev: RxSine_WORD_193; Default: 0x00FF(-); DF Addr: 0602h; SRAM Offset: 03A3h ; SubClass Offset : 2
	///     RX Sine0 Table Word[193]
	U2 wRxSineWord193;                                         
	///  3. Abbrev: RxSine_WORD_194; Default: 0x00FF(-); DF Addr: 0604h; SRAM Offset: 03A5h ; SubClass Offset : 4
	///     RX Sine0 Table Word[194]
	U2 wRxSineWord194;                                         
	///  4. Abbrev: RxSine_WORD_195; Default: 0x00FF(-); DF Addr: 0606h; SRAM Offset: 03A7h ; SubClass Offset : 6
	///     RX Sine0 Table Word[195]
	U2 wRxSineWord195;                                         
	///  5. Abbrev: RxSine_WORD_196; Default: 0x00FF(-); DF Addr: 0608h; SRAM Offset: 03A9h ; SubClass Offset : 8
	///     RX Sine0 Table Word[196]
	U2 wRxSineWord196;                                         
	///  6. Abbrev: RxSine_WORD_197; Default: 0x00FF(-); DF Addr: 060Ah; SRAM Offset: 03ABh ; SubClass Offset : 10
	///     RX Sine0 Table Word[197]
	U2 wRxSineWord197;                                         
	///  7. Abbrev: RxSine_WORD_198; Default: 0x00FF(-); DF Addr: 060Ch; SRAM Offset: 03ADh ; SubClass Offset : 12
	///     RX Sine0 Table Word[198]
	U2 wRxSineWord198;                                         
	///  8. Abbrev: RxSine_WORD_199; Default: 0x00FF(-); DF Addr: 060Eh; SRAM Offset: 03AFh ; SubClass Offset : 14
	///     RX Sine0 Table Word[199]
	U2 wRxSineWord199;                                         
	///  9. Abbrev: RxSine_WORD_200; Default: 0x00FF(-); DF Addr: 0610h; SRAM Offset: 03B1h ; SubClass Offset : 16
	///     RX Sine0 Table Word[200]
	U2 wRxSineWord200;                                         
	/// 10. Abbrev: RxSine_WORD_201; Default: 0x00FF(-); DF Addr: 0612h; SRAM Offset: 03B3h ; SubClass Offset : 18
	///     RX Sine0 Table Word[201]
	U2 wRxSineWord201;                                         
	/// 11. Abbrev: RxSine_WORD_202; Default: 0x00FF(-); DF Addr: 0614h; SRAM Offset: 03B5h ; SubClass Offset : 20
	///     RX Sine0 Table Word[202]
	U2 wRxSineWord202;                                         
	/// 12. Abbrev: RxSine_WORD_203; Default: 0x00FF(-); DF Addr: 0616h; SRAM Offset: 03B7h ; SubClass Offset : 22
	///     RX Sine0 Table Word[203]
	U2 wRxSineWord203;                                         
	/// 13. Abbrev: RxSine_WORD_204; Default: 0x00FF(-); DF Addr: 0618h; SRAM Offset: 03B9h ; SubClass Offset : 24
	///     RX Sine0 Table Word[204]
	U2 wRxSineWord204;                                         
	/// 14. Abbrev: RxSine_WORD_205; Default: 0x00FF(-); DF Addr: 061Ah; SRAM Offset: 03BBh ; SubClass Offset : 26
	///     RX Sine0 Table Word[205]
	U2 wRxSineWord205;                                         
	/// 15. Abbrev: RxSine_WORD_206; Default: 0x00FF(-); DF Addr: 061Ch; SRAM Offset: 03BDh ; SubClass Offset : 28
	///     RX Sine0 Table Word[206]
	U2 wRxSineWord206;                                         
	/// 16. Abbrev: RxSine_WORD_207; Default: 0x00FF(-); DF Addr: 061Eh; SRAM Offset: 03BFh ; SubClass Offset : 30
	///     RX Sine0 Table Word[207]
	U2 wRxSineWord207;                                         
	/// 17. Abbrev: RxSine_WORD_208; Default: 0x00FF(-); DF Addr: 0620h; SRAM Offset: 03C1h ; SubClass Offset : 32
	///     RX Sine0 Table Word[208]
	U2 wRxSineWord208;                                         
	/// 18. Abbrev: RxSine_WORD_209; Default: 0x00FF(-); DF Addr: 0622h; SRAM Offset: 03C3h ; SubClass Offset : 34
	///     RX Sine0 Table Word[209]
	U2 wRxSineWord209;                                         
	/// 19. Abbrev: RxSine_WORD_210; Default: 0x00FF(-); DF Addr: 0624h; SRAM Offset: 03C5h ; SubClass Offset : 36
	///     RX Sine0 Table Word[210]
	U2 wRxSineWord210;                                         
	/// 20. Abbrev: RxSine_WORD_211; Default: 0x00FF(-); DF Addr: 0626h; SRAM Offset: 03C7h ; SubClass Offset : 38
	///     RX Sine0 Table Word[211]
	U2 wRxSineWord211;                                         
	/// 21. Abbrev: RxSine_WORD_212; Default: 0x00FF(-); DF Addr: 0628h; SRAM Offset: 03C9h ; SubClass Offset : 40
	///     RX Sine0 Table Word[212]
	U2 wRxSineWord212;                                         
	/// 22. Abbrev: RxSine_WORD_213; Default: 0x00FF(-); DF Addr: 062Ah; SRAM Offset: 03CBh ; SubClass Offset : 42
	///     RX Sine0 Table Word[213]
	U2 wRxSineWord213;                                         
	/// 23. Abbrev: RxSine_WORD_214; Default: 0x00FF(-); DF Addr: 062Ch; SRAM Offset: 03CDh ; SubClass Offset : 44
	///     RX Sine0 Table Word[214]
	U2 wRxSineWord214;                                         
	/// 24. Abbrev: RxSine_WORD_215; Default: 0x00FF(-); DF Addr: 062Eh; SRAM Offset: 03CFh ; SubClass Offset : 46
	///     RX Sine0 Table Word[215]
	U2 wRxSineWord215;                                         
	/// 25. Abbrev: RxSine_WORD_216; Default: 0x00FF(-); DF Addr: 0630h; SRAM Offset: 03D1h ; SubClass Offset : 48
	///     RX Sine0 Table Word[216]
	U2 wRxSineWord216;                                         
	/// 26. Abbrev: RxSine_WORD_217; Default: 0x00FF(-); DF Addr: 0632h; SRAM Offset: 03D3h ; SubClass Offset : 50
	///     RX Sine0 Table Word[217]
	U2 wRxSineWord217;                                         
	/// 27. Abbrev: RxSine_WORD_218; Default: 0x00FF(-); DF Addr: 0634h; SRAM Offset: 03D5h ; SubClass Offset : 52
	///     RX Sine0 Table Word[218]
	U2 wRxSineWord218;                                         
	/// 28. Abbrev: RxSine_WORD_219; Default: 0x00FF(-); DF Addr: 0636h; SRAM Offset: 03D7h ; SubClass Offset : 54
	///     RX Sine0 Table Word[219]
	U2 wRxSineWord219;                                         
	/// 29. Abbrev: RxSine_WORD_220; Default: 0x00FF(-); DF Addr: 0638h; SRAM Offset: 03D9h ; SubClass Offset : 56
	///     RX Sine0 Table Word[220]
	U2 wRxSineWord220;                                         
	/// 30. Abbrev: RxSine_WORD_221; Default: 0x00FF(-); DF Addr: 063Ah; SRAM Offset: 03DBh ; SubClass Offset : 58
	///     RX Sine0 Table Word[221]
	U2 wRxSineWord221;                                         
	/// 31. Abbrev: RxSine_WORD_222; Default: 0x00FF(-); DF Addr: 063Ch; SRAM Offset: 03DDh ; SubClass Offset : 60
	///     RX Sine0 Table Word[222]
	U2 wRxSineWord222;                                         
	/// 32. Abbrev: RxSine_WORD_223; Default: 0x00FF(-); DF Addr: 063Eh; SRAM Offset: 03DFh ; SubClass Offset : 62
	///     RX Sine0 Table Word[223]
	U2 wRxSineWord223;                                         
} SubClasswRxSineWord192Type;	///< Total = 64 Bytes

/// =============================================================///
/// Data flash Class #40 => RxSineWord192
/// =============================================================///
typedef struct PACK ClassRxSineWord192St
{
	SubClasswRxSineWord192Type scRxSineWord192;
}ClassRxSineWord192Type;	///< Total = 64 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #70 => RxSineCoeffWord0
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord0St
{
	///  1. Abbrev: RxSineCoef_WORD_00; Default: 0x0000(-); DF Addr: 0640h; SRAM Offset: 03E1h ; SubClass Offset : 0
	///     RX Sine Coef Table Word[00]
	U2 wRxSineCoefWord00;                                      
} SubClassRxSineCoeffWord0Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #41 => RxSineCoeffWord0
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord0St
{
	SubClassRxSineCoeffWord0Type scRxSineCoeffWord0;
}ClassRxSineCoeffWord0Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #71 => RxSineCoeffWord32
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord32St
{
	///  1. Abbrev: RxSineCoef_WORD_32; Default: 0x014B(-); DF Addr: 0680h; SRAM Offset: 03E3h ; SubClass Offset : 0
	///     RX Sine Coef Table Word[32]
	U2 wRxSineCoefWord32;                                      
} SubClassRxSineCoeffWord32Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #42 => RxSineCoeffWord32
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord32St
{
	SubClassRxSineCoeffWord32Type scRxSineCoeffWord32;
}ClassRxSineCoeffWord32Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #72 => RxSineCoeffWord96
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord96St
{
	///  1. Abbrev: RxSineCoef_WORD_96; Default: 0x0539(-); DF Addr: 0700h; SRAM Offset: 03E5h ; SubClass Offset : 0
	///     RX Sine Coef Table Word[96]
	U2 wRxSineCoefWord96;                                      
} SubClassRxSineCoeffWord96Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #43 => RxSineCoeffWord96
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord96St
{
	SubClassRxSineCoeffWord96Type scRxSineCoeffWord96;
}ClassRxSineCoeffWord96Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #73 => RxSineCoeffWord160
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord160St
{
	///  1. Abbrev: RxSineCoef_WORD_160; Default: 0x0587(-); DF Addr: 0780h; SRAM Offset: 03E7h ; SubClass Offset : 0
	///     RX Sine Coef Table Word[160]
	U2 wRxSineCoefWord160;                                     
} SubClassRxSineCoeffWord160Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #44 => RxSineCoeffWord160
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord160St
{
	SubClassRxSineCoeffWord160Type scRxSineCoeffWord160;
}ClassRxSineCoeffWord160Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #74 => RxSineCoeffWord224
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord224St
{
	///  1. Abbrev: RxSineCoef_WORD_224; Default: 0xFFFF(-); DF Addr: 0800h; SRAM Offset: 03E9h ; SubClass Offset : 0
	///     RX Sine Coef Table Word[224]
	U2 wRxSineCoefWord224;                                     
} SubClassRxSineCoeffWord224Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #45 => RxSineCoeffWord224
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord224St
{
	SubClassRxSineCoeffWord224Type scRxSineCoeffWord224;
}ClassRxSineCoeffWord224Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #75 => RxSineCoeffWord288
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord288St
{
	///  1. Abbrev: RxSineCoef_WORD_288; Default: 0xFFFF(-); DF Addr: 0880h; SRAM Offset: 03EBh ; SubClass Offset : 0
	///     RX Sine Coef Table Word[288]
	U2 wRxSineCoefWord288;                                     
} SubClassRxSineCoeffWord288Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #46 => RxSineCoeffWord288
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord288St
{
	SubClassRxSineCoeffWord288Type scRxSineCoeffWord288;
}ClassRxSineCoeffWord288Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #76 => RxSineCoeffWord352
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassRxSineCoeffWord352St
{
	///  1. Abbrev: RxSineCoef_WORD_352; Default: 0x0000(-); DF Addr: 0900h; SRAM Offset: 03EDh ; SubClass Offset : 0
	///     RX Sine Coef Table Word[351]
	U2 wRxSineCoefWord352;                                     
} SubClassRxSineCoeffWord352Type;	///< Total = 2 Bytes

/// =============================================================///
/// Data flash Class #47 => RxSineCoeffWord352
/// =============================================================///
typedef struct PACK ClassRxSineCoeffWord352St
{
	SubClassRxSineCoeffWord352Type scRxSineCoeffWord352;
}ClassRxSineCoeffWord352Type;	///< Total = 2 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #77 => TxSineByte0
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte0St
{
	///  1. Abbrev: TXSINE_BYTE_00; Default: TX Sine Byte[00](-); DF Addr: 0980h; SRAM Offset: 03EFh ; SubClass Offset : 0
	///     
	U1 bTxSineByte00;                                          
} SubClassTxSineByte0Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #48 => TxSineByte0
/// =============================================================///
typedef struct PACK ClassTxSineByte0St
{
	SubClassTxSineByte0Type scTxSineByte0;
}ClassTxSineByte0Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #78 => TxSineByte128
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte128St
{
	///  1. Abbrev: TXSINE_BYTE_128; Default: 0x3F(-); DF Addr: 0A00h; SRAM Offset: 03F0h ; SubClass Offset : 0
	///     TX Sine Byte[128]
	U1 bTxSineByte128;                                         
} SubClassTxSineByte128Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #49 => TxSineByte128
/// =============================================================///
typedef struct PACK ClassTxSineByte128St
{
	SubClassTxSineByte128Type scTxSineByte128;
}ClassTxSineByte128Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #79 => TxSineByte256
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte256St
{
	///  1. Abbrev: TXSINE_BYTE_256; Default: TX Sine Byte[256](-); DF Addr: 0A80h; SRAM Offset: 03F1h ; SubClass Offset : 0
	///     
	U1 bTxSineByte256;                                         
} SubClassTxSineByte256Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #50 => TxSineByte256
/// =============================================================///
typedef struct PACK ClassTxSineByte256St
{
	SubClassTxSineByte256Type scTxSineByte256;
}ClassTxSineByte256Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #80 => TxSineByte384
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte384St
{
	///  1. Abbrev: TXSINE_BYTE_384; Default: TX Sine Byte[384](-); DF Addr: 0B00h; SRAM Offset: 03F2h ; SubClass Offset : 0
	///     
	U1 bTxSineByte384;                                         
} SubClassTxSineByte384Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #51 => TxSineByte384
/// =============================================================///
typedef struct PACK ClassTxSineByte384St
{
	SubClassTxSineByte384Type scTxSineByte384;
}ClassTxSineByte384Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #81 => TxSineByte512
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte512St
{
	///  1. Abbrev: TXSINE_BYTE_512; Default: TX Sine Byte[512](-); DF Addr: 0B80h; SRAM Offset: 03F3h ; SubClass Offset : 0
	///     
	U1 bTxSineByte512;                                         
} SubClassTxSineByte512Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #52 => TxSineByte512
/// =============================================================///
typedef struct PACK ClassTxSineByte512St
{
	SubClassTxSineByte512Type scTxSineByte512;
}ClassTxSineByte512Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #82 => TxSineByte640
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte640St
{
	///  1. Abbrev: TXSINE_BYTE_640; Default: TX Sine Byte[640](-); DF Addr: 0C00h; SRAM Offset: 03F4h ; SubClass Offset : 0
	///     
	U1 bTxSineByte640;                                         
} SubClassTxSineByte640Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #53 => TxSineByte640
/// =============================================================///
typedef struct PACK ClassTxSineByte640St
{
	SubClassTxSineByte640Type scTxSineByte640;
}ClassTxSineByte640Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #83 => TxSineByte768
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte768St
{
	///  1. Abbrev: TXSINE_BYTE_768; Default: TX Sine Byte[768](-); DF Addr: 0C80h; SRAM Offset: 03F5h ; SubClass Offset : 0
	///     
	U1 bTxSineByte768;                                         
} SubClassTxSineByte768Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #54 => TxSineByte768
/// =============================================================///
typedef struct PACK ClassTxSineByte768St
{
	SubClassTxSineByte768Type scTxSineByte768;
}ClassTxSineByte768Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #84 => TxSineByte896
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte896St
{
	///  1. Abbrev: TXSINE_BYTE_896; Default: TX Sine Byte[896](-); DF Addr: 0D00h; SRAM Offset: 03F6h ; SubClass Offset : 0
	///     
	U1 bTxSineByte896;                                         
} SubClassTxSineByte896Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #55 => TxSineByte896
/// =============================================================///
typedef struct PACK ClassTxSineByte896St
{
	SubClassTxSineByte896Type scTxSineByte896;
}ClassTxSineByte896Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #85 => TxSineByte1024
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte1024St
{
	///  1. Abbrev: TXSINE_BYTE_1024; Default: TX Sine Byte[1024](-); DF Addr: 0D80h; SRAM Offset: 03F7h ; SubClass Offset : 0
	///     
	U1 bTxSineByte1024;                                        
} SubClassTxSineByte1024Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #56 => TxSineByte1024
/// =============================================================///
typedef struct PACK ClassTxSineByte1024St
{
	SubClassTxSineByte1024Type scTxSineByte1024;
}ClassTxSineByte1024Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #86 => TxSineByte1152
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte1152St
{
	///  1. Abbrev: TXSINE_BYTE_1152; Default: 0xCC(-); DF Addr: 0E00h; SRAM Offset: 03F8h ; SubClass Offset : 0
	///     TX Sine Byte[1151]
	U1 bTxSineByte1152;                                        
} SubClassTxSineByte1152Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #57 => TxSineByte1152
/// =============================================================///
typedef struct PACK ClassTxSineByte1152St
{
	SubClassTxSineByte1152Type scTxSineByte1152;
}ClassTxSineByte1152Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #87 => TxSineByte1280
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte1280St
{
	///  1. Abbrev: TXSINE_BYTE_1280; Default: 0xCC(-); DF Addr: 0E80h; SRAM Offset: 03F9h ; SubClass Offset : 0
	///     TX Sine Byte[1280]
	U1 bTxSineByte1280;                                        
} SubClassTxSineByte1280Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #58 => TxSineByte1280
/// =============================================================///
typedef struct PACK ClassTxSineByte1280St
{
	SubClassTxSineByte1280Type scTxSineByte1280;
}ClassTxSineByte1280Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #88 => TxSineByte1408
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte1408St
{
	///  1. Abbrev: TXSINE_BYTE_1408; Default: TX Sine Byte[1408](-); DF Addr: 0F00h; SRAM Offset: 03FAh ; SubClass Offset : 0
	///     
	U1 bTxSineByte1408;                                        
} SubClassTxSineByte1408Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #59 => TxSineByte1408
/// =============================================================///
typedef struct PACK ClassTxSineByte1408St
{
	SubClassTxSineByte1408Type scTxSineByte1408;
}ClassTxSineByte1408Type;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #89 => TxSineByte1536
///-------------------------------------------------------------------------------///
typedef struct PACK SubClassTxSineByte1536St
{
	///  1. Abbrev: TXSINE_BYTE_1536; Default: TX Sine Byte[1536](-); DF Addr: 0F80h; SRAM Offset: 03FBh ; SubClass Offset : 0
	///     
	U1 bTxSineByte1536;                                        
} SubClassTxSineByte1536Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data flash Class #60 => TxSineByte1536
/// =============================================================///
typedef struct PACK ClassTxSineByte1536St
{
	SubClassTxSineByte1536Type scTxSineByte1536;
}ClassTxSineByte1536Type;	///< Total = 1 Bytes

/// =============================================================///
/// Data Flash Structure
/// =============================================================////
typedef struct PACK GlobalDFVarSt
{
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#00 (99 Bytes)
	///-------------------------------------------------------------------------------///
	ClassGlobalDataType  cGlobalData;
	ClassInitialDevType  cInitdsevRatio;
	BYTE bDummyPage00[29];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#01 (127 Bytes)
	///-------------------------------------------------------------------------------///
	ClassDevDataType  cDevData;
	ClassTraceSetupType  cTraceSetup;
	ClassAxisCfgType  cAxisCfg;
	BYTE bDummyPage01[1];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#02 (123 Bytes)
	///-------------------------------------------------------------------------------///
	ClassInitialAnalogType  cInitialAnalog;
	ClassMutualAnalogType  cMutualAnalog;
	ClassForceAnalogType  cForceAnalog;
	ClassMiscRegisterType  cMiscRegister;
	ClassAlogAxisDevRatioType  cAlogAxisDevRatio;
	ClassFloatingIssueType  cFloatingIssue;
	BYTE bDummyPage02[5];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#03 (118 Bytes)
	///-------------------------------------------------------------------------------///
	ClassForceTraceSetupType  cForceTraceSetup;
	ClassFormatCtrlType  cFormatCtrl;
	BYTE bDummyPage03[10];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#04 (125 Bytes)
	///-------------------------------------------------------------------------------///
	ClassKeyCtrlType  cKeyCtrl;
	ClassFuncCtrlType  cFuncCtrl;
	ClassSYSFreqHopType  cSYSFreqHop;
	ClassClusterType  cCluster;
	ClassTpCrossTalkFixType  cTpCrossTalkFix;
	BYTE bDummyPage04[3];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#05 (76 Bytes)
	///-------------------------------------------------------------------------------///
	ClassCoAxisAntiNoiseType  cCoAxisAntiNoise;
	ClassFingerType  cFinger;
	BYTE bDummyPage05[52];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#06 (96 Bytes)
	///-------------------------------------------------------------------------------///
	ClassAxisProcessType  cAxisProcess;
	ClassDevCutType  cDevCut;
	ClassLinearityType  cLinearity;
	ClassBorderCompType  cBorderComp;
	ClassCoordRemapType  cCoordRemap;
	BYTE bDummyPage06[32];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#07 (58 Bytes)
	///-------------------------------------------------------------------------------///
	ClassCoorCompType  cCoorComp;
	ClassFrameRateType  cFrameRate;
	ClassIRsetupPageType  cIRsetupPage;
	ClassFingerDistType  cFingerDist;
	ClassIForceSetupType  cIForceSetup;
	ClassDummyPage7Type  cDummyPage7;
	BYTE bDummyPage07[70];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#08 (101 Bytes)
	///-------------------------------------------------------------------------------///
	ClassDFProtectType  cDFProtect;
	ClassTpCrossTalkFix1Type  cTpCrossTalkFix1;
	ClassTableLengthType  cTableLength;
	ClassGestureType  cGesture;
	ClassAlgoPageType  cAlgoPage;
	ClassDummyPage8Type  cDummyPage8;
	BYTE bDummyPage08[27];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#09 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineWord0Type  cRxSineWord0;
	BYTE bDummyPage09[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#10 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineWord64Type  cRxSineWord64;
	BYTE bDummyPage10[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#11 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineWord128Type  cRxSineWord128;
	BYTE bDummyPage11[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#12 (66 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineWord192Type  cRxSineWord192;
	ClassRxSineCoeffWord0Type  cRxSineCoeffWord0;
	BYTE bDummyPage12[62];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#13 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord32Type  cRxSineCoeffWord32;
	BYTE bDummyPage13[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#14 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord96Type  cRxSineCoeffWord96;
	BYTE bDummyPage14[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#15 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord160Type  cRxSineCoeffWord160;
	BYTE bDummyPage15[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#16 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord224Type  cRxSineCoeffWord224;
	BYTE bDummyPage16[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#17 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord288Type  cRxSineCoeffWord288;
	BYTE bDummyPage17[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#18 (2 Bytes)
	///-------------------------------------------------------------------------------///
	ClassRxSineCoeffWord352Type  cRxSineCoeffWord352;
	BYTE bDummyPage18[126];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#19 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte0Type  cTxSineByte0;
	BYTE bDummyPage19[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#20 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte128Type  cTxSineByte128;
	BYTE bDummyPage20[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#21 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte256Type  cTxSineByte256;
	BYTE bDummyPage21[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#22 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte384Type  cTxSineByte384;
	BYTE bDummyPage22[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#23 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte512Type  cTxSineByte512;
	BYTE bDummyPage23[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#24 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte640Type  cTxSineByte640;
	BYTE bDummyPage24[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#25 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte768Type  cTxSineByte768;
	BYTE bDummyPage25[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#26 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte896Type  cTxSineByte896;
	BYTE bDummyPage26[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#27 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte1024Type  cTxSineByte1024;
	BYTE bDummyPage27[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#28 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte1152Type  cTxSineByte1152;
	BYTE bDummyPage28[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#29 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte1280Type  cTxSineByte1280;
	BYTE bDummyPage29[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#30 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte1408Type  cTxSineByte1408;
	BYTE bDummyPage30[127];
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#31 (1 Bytes)
	///-------------------------------------------------------------------------------///
	ClassTxSineByte1536Type  cTxSineByte1536;
	BYTE bDummyPage31[127];
} GlobalDFVarType;


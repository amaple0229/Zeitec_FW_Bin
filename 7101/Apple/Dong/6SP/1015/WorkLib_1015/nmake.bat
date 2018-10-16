::==================================================================::
:: File  : Make.BAT
:: Description: Make the Keil C source code 
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
::==================================================================::
@echo on
cls
CALL nsetting.bat
CALL clean.bat
::SET "BUILD_TYPE=GEN_LIB" 
::-----------------------------------------------------------------::
::    STEP 1: SELECT PANEL and DEF_TYPE
::-----------------------------------------------------------------::
IF "%COMPILE_TARGET_IC%" == "ZET7100" copy  SRC\ZetDF_7100.xml SRC\ZetDF.xml
IF "%COMPILE_TARGET_IC%" == "ZET7101" copy  SRC\ZetDF_7101.xml SRC\ZetDF.xml

SET SUCCESS=OK
CALL SetTarget.BAT %1
CALL nSetOption.BAT %2


echo #define ICDEVICENAME "%DEF_NAME%" > INC\ICDeviceName.h
::-----------------------------------------------------------------::
::    STEP 2: Configure
::-----------------------------------------------------------------::
CALL GENDF.BAT


::CALL JVER.BAT
::@echo on

::-----------------------------------------------------------------::
::    STEP 3: Compile
::-----------------------------------------------------------------::
::-----------------------------------------------------------------::
::    JCC  [FILE]       	[OPTIMIZE]     [OPTIMIZE_TYPE]		[OTHER]
::
:: [OPTIMIZE] :
:: Os1 ==> Enable minimum code size optimizations. Performance is still concerned.
:: Os2 ==> Enable partial code size optimizations with little performance concern.
:: Os3 ==> Same as -Os option. Enable all code size optimizations. Performance may
::-----------------------------------------------------------------::
@echo ##################################################
@echo #                   COMPILE                      #
@echo ##################################################
CALL  nJCC  I2c    					Os3               SIZE
CALL  nJCC  Isr     				Os3               SIZE
CALL  nJCC  SPI     				Os3               SIZE
CALL  nJCC  Main          	Os3               SIZE
CALL  nJCC  Customer          	Os3               SIZE

::-----------------------------------------------------------------::
::    JAA  [FILE]       	[SUB]
::-----------------------------------------------------------------::
:: Check the compile OK
:: IF NOT %SUCCESS%==OK (GOTO LABEL_FAIL)
:: [NOTE] : For ZET6231 strobe issue
::CALL  JAA   Isr						A51

::-----------------------------------------------------------------::
::    STEP 4: Link 
::-----------------------------------------------------------------::
:: Check the compile OK
IF NOT %SUCCESS%==OK (GOTO LABEL_FAIL)
CALL JLINK Os3


:: -----------------------------------------------------------------::
::    STEP 5: Merge image
:: -----------------------------------------------------------------::
:: Check the compile OK
@IF NOT %SUCCESS%==OK (GOTO LABEL_FAIL)
::%PRJ_PATH%\BinMerge.exe %BIN_PATH%\JOUT.bin %PRJ_PATH%\DB.BIN %BIN_PATH%\%PROJECT_NAME%.BIN	0x9C00 2000

IF "%COMPILE_TARGET_IC%" == "ZET7100"  goto LABEL_ZET7100_SETUP
IF "%COMPILE_TARGET_IC%" == "ZET7101"  goto LABEL_ZET7101_SETUP

:LABEL_ZET7100_SETUP
  %PRJ_PATH%\BinMerge.exe %BIN_PATH%\JOUT.bin %PRJ_PATH%\DB.BIN %BIN_PATH%\%PROJECT_NAME%.BIN	0xE000 2000  
	GOTO LABEL_SETUP_END	
		

:LABEL_ZET7101_SETUP

::%PRJ_PATH%\BinMerge.exe %BIN_PATH%\JOUT.bin %PRJ_PATH%\DB.BIN %BIN_PATH%\%PROJECT_NAME%.BIN	0xE000 2000  
  %PRJ_PATH%\CSGen.exe %BIN_PATH%\JOUT.bin %BIN_PATH%\JOUT-CS.bin ZET7101_PROM
  %PRJ_PATH%\CSGen.exe %PRJ_PATH%\DB.BIN %PRJ_PATH%\DB-CS.BIN ZET7101_PARA 
  %PRJ_PATH%\BinMerge.exe %BIN_PATH%\JOUT-CS.bin %PRJ_PATH%\DB-CS.BIN %BIN_PATH%\%PROJECT_NAME%.BIN	0xC000 500  

  
	GOTO LABEL_SETUP_END	
				
		
:LABEL_SETUP_END:

%PRJ_PATH%\GENNDSHEX.exe %BIN_PATH%\%PROJECT_NAME%.bin   %BIN_PATH%\%PROJECT_NAME%.HEX


:: -----------------------------------------------------------------::
::    STEP 6: Backup image
:: -----------------------------------------------------------------::
COPY %BIN_PATH%\%PROJECT_NAME%.BIN  %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\%PROJECT_NAME%_%DEF_NAME%.BIN
::COPY %BIN_PATH%\%PROJECT_NAME%.BIN  %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\


@echo ##################################################
@echo #                   SUCCEED                      #
@echo ##################################################

:LABEL_SUCCEED
set file=BIN\%PROJECT_NAME%.NM
call:ExtractCompileResult

GOTO LABEL_END

::------------------------------------------
:: FAIL END
::------------------------------------------
:LABEL_FAIL
@echo on
@echo ##################################################
@echo #                BUILD FAIL                      #
@echo ##################################################
@echo %SUCCESS%
GOTO LABEL_END

::------------------------------------------
:: Show Compile Result Srame/ROM Size
::------------------------------------------
:ExtractCompileResult

set "LineOfROMsize="
for /f "delims=: " %%a in ('findstr /n "PROG_ROM_size" "%file%"') do (
      if          not defined LineOfROMsize (set "LineOfROMsize=%%a"
			)
)
set "LineOfSRAMsize="
for /f "delims=: " %%a in ('findstr /n "_end" "%file%"') do (
      if          not defined LineOfSRAMsize (set "LineOfSRAMsize=%%a"
			)
)

set LineNo=%LineOfROMsize%
set "CompileROMSize="
set/a LineNo-=1
for /f "delims= " %%a in ('more/e +%LineNo% ^< %file%') do (
  if not defined CompileROMSize set "CompileROMSize=%%a"
)
set /a vROMSize=0x%CompileROMSize%

set LineNo=%LineOfSRAMsize%
set  "CompileSRAMSize="
set/a LineNo-=1
for /f "delims= " %%a in ('more/e +%LineNo% ^< %file%') do (
  if not defined CompileSRAMSize set "CompileSRAMSize=%%a"
)
set /a vSRAMSize=0x%CompileSRAMSize:~4,4%

@set  digitals=%vSRAMSize%

@echo ##################################################
@echo #        Compile  ROM/SRAM Cost                  #
@echo ##################################################
@echo Device_%DEF_NAME% RomSize  SramSize 
::@echo 0x%CompileROMSize:~4,4% (%vROMSize%)       0x%CompileSRAMSize:~4,4% (%vSRAMSize%) 
::@echo %vROMSize% %vSRAMSize%


@if %digitals% GTR 9999 (
@echo %vROMSize% %vSRAMSize%
) else (	
@echo %vROMSize%  %vSRAMSize%
)
   

:::::------------------------------------------
::::: SCRIPT END
:::::------------------------------------------
:LABEL_END



   

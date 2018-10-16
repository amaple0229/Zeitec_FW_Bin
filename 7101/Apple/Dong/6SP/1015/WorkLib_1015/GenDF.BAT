::------------------------------------------------------------------::
:: File  : GENDF.BAT
:: Description: Generate the data flash parameter
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
:: -----------------------------------------------------------------::
CALL nSETTING.BAT

@echo on
::-----------------------------------------------
:: Delete the .h and the out.xml
::-----------------------------------------------
DEL %INC_PATH%\ZetDF.h

::-----------------------------------------------
:: Delete the current setup file
::-----------------------------------------------
COPY %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\SETUP.TXT %SRC_PATH%\SETUP.TXT

@echo on
::%PRJ_PATH%\GenXML.EXE %SRC_PATH%\ZetDF.XML %SRC_PATH%\OUT.XML	%SRC_PATH%\SETUP.TXT %PROJECT_NAME%    
%PRJ_PATH%\GenXML.EXE %SRC_PATH%\ZetDF.XML %SRC_PATH%\OUT.XML	%SRC_PATH%\SETUP.TXT "ZET7100"   

@echo off
::%PRJ_PATH%\ZetDF1.EXE %SRC_PATH%\OUT.XML SPAN LE F3
IF "%COMPILE_TARGET_IC%" == "ZET7100"  goto LABEL_ZET7100_SETUP
IF "%COMPILE_TARGET_IC%" == "ZET7101"  goto LABEL_ZET7101_SETUP

:LABEL_ZET7100_SETUP  
::%PRJ_PATH%\ZetDF1old.EXE %SRC_PATH%\OUT.XML   SPAN     LE    FF    %INC_PATH%\Revision.h    %DEF_NAME%
::%PRJ_PATH%\ZetDF1.EXE      %SRC_PATH%\OUT.XML  /d:LE /o:FF /i:%INC_PATH%\Revision.h /n:%DEF_NAME%  /s:4096
%PRJ_PATH%\ZetDF1.EXE      %SRC_PATH%\OUT.XML /e:NC /d:LE /o:FF /i:%INC_PATH%\Revision.h  /j:%INC_PATH%\SubRevision.h /n:%DEF_NAME%  /s:4096
	GOTO LABEL_SETUP_END	

:LABEL_ZET7101_SETUP  
::%PRJ_PATH%\ZetDF1old.EXE %SRC_PATH%\OUT.XML   SPAN     LE    FF    %INC_PATH%\Revision.h    %DEF_NAME%
::%PRJ_PATH%\ZetDF1.EXE      %SRC_PATH%\OUT.XML  /d:LE /o:FF /i:%INC_PATH%\Revision.h /n:%DEF_NAME%  /s:4096
%PRJ_PATH%\ZetDF1.EXE      %SRC_PATH%\OUT.XML /e:NC /d:LE /o:FF /i:%INC_PATH%\Revision.h  /j:%INC_PATH%\SubRevision.h /n:%DEF_NAME%  /s:4096
	GOTO LABEL_SETUP_END	

		 
:LABEL_SETUP_END:

DEL *.CFG
::-----------------------------------------------
:: Delete the current setup file
::-----------------------------------------------
DEL %SRC_PATH%\SETUP.TXT

::-----------------------------------------------
:: COPY THE SETTING FILE FOR Software
::-----------------------------------------------
@DEL %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\%PROJECT_NAME%.bin
REM @DEL %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\%PROJECT_NAME%_%DEF_NAME%.bin

REM COPY .\*.CFG %DEV_PATH%\%PROJECT_NAME%\

::-----------------------------------------------
:: COPY DEFAULT
::-----------------------------------------------
DEL  %DEV_PATH%\%PROJECT_NAME%\ZetDF_DEFAULT.XML
COPY %SRC_PATH%\OUT.XML %DEV_PATH%\%PROJECT_NAME%\ZetDF_DEFAULT.XML

REM MOVE %SRC_PATH%\OUT.XML %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\ 
DEL %SRC_PATH%\OUT.XML

MOVE *.XML %DEV_PATH%\%PROJECT_NAME%\

REM COPY db.bin  %DEV_PATH%\%PROJECT_NAME%\%PANEL_TYPE%\

::-----------------------------------------------
:: COPY RESULT FILES
::-----------------------------------------------
MOVE ZetDF.h %INC_PATH%
MOVE db.bin  %PRJ_PATH%

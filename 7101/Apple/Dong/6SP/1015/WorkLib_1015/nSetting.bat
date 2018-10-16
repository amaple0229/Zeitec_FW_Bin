::==================================================================::
:: File  : Setting.BAT
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
::==================================================================::
@echo off
::-----------------------------------------------------------------::
::    STEP 1: SET PROJECT NAME
::-----------------------------------------------------------------::
SET PROJECT_NAME=%COMPILE_TARGET_IC%
::-----------------------------------------------------------------::
::    STEP 2: SET SOURCE
::-----------------------------------------------------------------::

set WDIR=%cd%

SET "SRC_PATH=.\SRC"
SET "INC_PATH=.\INC"
SET "BIN_PATH=.\BIN"

SET "JCC_SRC_PATH=./SRC"
SET "JCC_INC_PATH=./INC"
SET "JCC_BIN_PATH=./BIN"

SET "PRJ_PATH=.\PRJ"
SET "DEV_PATH=.\DEV"
SET "DOC_PATH=.\DOC"

::-----------------------------------------------------------------::
::    STEP 3: SET ANDES TOOLKITS
::-----------------------------------------------------------------::
SET CROSS_COMPILE=nds32le-elf-
SET CC=%CROSS_COMPILE%gcc
SET AS=%CROSS_COMPILE%as
SET LD=%CROSS_COMPILE%ld
SET NM=%CROSS_COMPILE%nm
SET AR=%CROSS_COMPILE%ar
SET OBJDUMP=%CROSS_COMPILE%objdump
SET OBJCOPY=%CROSS_COMPILE%objcopy
SET SIZE=%CROSS_COMPILE%size

:: For SAG TO LD
SET LDSAG=%PRJ_PATH%\nds_ldsag.exe

::-----------------------------------------------------------------::
::    STEP 4: INIT VARIABLES
::-----------------------------------------------------------------::
SET OBJ_LIST=

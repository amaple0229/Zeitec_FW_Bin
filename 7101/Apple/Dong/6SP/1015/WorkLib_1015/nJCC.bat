::------------------------------------------------------------------::
:: File  : JCC.BAT
:: Description: Compile the C file
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
:: -----------------------------------------------------------------::
IF NOT %SUCCESS%==OK (GOTO LABEL_FAIL)
SET C_FILE=%1
SET OPTIZE_LEVEL=%2
SET OPTION_SRC=%4

SET INC_DIR=%JCC_INC_PATH%

SET CFLAGS= -DDEMO_INT_C_EXT  -DUSE_C_EXT -%OPTIZE_LEVEL% -g3 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections  -I %JCC_INC_PATH% -MMD -MP  -fno-strict-aliasing

@echo on
@echo		nCC:  %C_FILE%.o


@echo off
%CC%    %CFLAGS%    %JCC_SRC_PATH%/%C_FILE%.c  -I%INC_DIR%   -o %JCC_BIN_PATH%/%C_FILE%.o  %DEF_TYPE%
@echo off 
::REM %CC%    %CFLAGS% -S   %JCC_SRC_PATH%/%C_FILE%.c  -I%INC_DIR%   -o %JCC_BIN_PATH%/%C_FILE%.s

IF NOT EXIST  %BIN_PATH%\%C_FILE%.o (GOTO LABEL_FAIL)

::------------------------------------------
:: Add OBJECT into object list
::------------------------------------------
SET OBJ_LIST=%OBJ_LIST% %C_FILE%.o

GOTO LABEL_END

::------------------------------------------
:: Fail End
::------------------------------------------
:LABEL_FAIL
SET	SUCCESS=FAIL

::------------------------------------------
:: Script End
::------------------------------------------
:LABEL_END
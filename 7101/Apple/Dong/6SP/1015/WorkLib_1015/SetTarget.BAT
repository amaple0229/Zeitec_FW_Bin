::==================================================================::
:: File  : SetTarget.BAT
:: Description: Set the target device
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
::==================================================================::

CALL nsetting.bat

SET "SEL=%1"

:LABEL_CHECK
::=========================================================================
IF "%SEL%" == "1"  goto LABEL_ZET_DEFAULT


::=========================================================================
		ECHO ------------------------------------------------------------------	
		ECHO Please select the targer panel
		ECHO ------------------------------------------------------------------	
		ECHO [#] = 1  : ZET_28X16(DEFAULT)"
		echo ------------------------------------------------------------------
  	SET INPUT=
		SET /P INPUT=Please select target:%=% 
		SET SEL=%INPUT%
		
		GOTO LABEL_CHECK
		
::=========================================================================
:LABEL_ZET_DEFAULT
::=========================================================================
	SET "PANEL_TYPE=ZET_DEFAULT"
	GOTO LABEL_TRAGE_PANEL_END		
	
::=========================================================================	
:LABEL_TRAGE_PANEL_END:

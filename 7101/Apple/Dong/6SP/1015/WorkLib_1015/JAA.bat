::------------------------------------------------------------------::
:: File  : JAA.BAT
:: Description: Compile the asm file
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
:: -----------------------------------------------------------------::
SET  A_FILE=%1
SET  A_SUB=%2

A51 %SRC_PATH%\%A_FILE%.%A_SUB% DEBUG PRINT(%BIN_PATH%\%A_FILE%.lst) OBJECT(%BIN_PATH%\%A_FILE%.obj)
MOVE %SRC_PATH%\%A_FILE%.SRC    %BIN_PATH%\%A_FILE%.SRC 
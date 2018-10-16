::------------------------------------------------------------------::
:: File  : JLS.BAT
:: Description: Link section
:: Author: JLJuang
:: E-Mail: JLJuang@csie.nctu.edu.tw
:: NOTE  : All copy rights reserved (c) since 2003
:: -----------------------------------------------------------------::
@ECHO OFF
SET SEC_NAME=%1
SET SEC_ADDR=%2

::---------------------------------::
:: Set the link position
::---------------------------------::
SET LDFLAGS=%LDFLAGS%,--section-start=%SEC_NAME%=%SEC_ADDR%

::---------------------------------::
:: Remove section for binary file
::---------------------------------::
SET RMSEC=%RMSEC%  -R %SEC_NAME%
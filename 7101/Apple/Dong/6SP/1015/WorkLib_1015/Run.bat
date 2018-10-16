cls
cls
@echo %time:~0,5% > CompilelogX.txt
::@nmakeall 2>&1  | .\PRJ\mtee  /+ Compilelog0.txt


@echo off
SETLOCAL EnableDelayedExpansion
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set "DEL=%%a"
)

SET "COMPILE_TARGET_IC=ZET7101"
SET "COMPILE_OPTION=RELEASE"

IF "%COMPILE_OPTION%" == "RELEASE"  Goto :ReleaseCompile
Goto :BinCompile

::========================================================================================
:: Release Bin Compile
::========================================================================================
:ReleaseCompile

::del SvnRevCheck.log
::@SubWCRev .\  Release\ZET7101\ReleaseVerCheck.tmpl SvnRevCheck.log
::@FOR /F %%i IN (SvnRevCheck.log) DO @set /A DataNum=%%i
  
  ::echo FileChange = %DataNum%
  ::@set CheckResuklt=%DataNum%
  @set/a CheckResuklt-=0
		
	  Goto :BinCompile




:BinCompile
		call :ColorText 0b "################################################################"
		@echo\ 	
		call :ColorText 0b "##### Start Compile Source Code                       ##########"
		@echo\ 	
		call :ColorText 0b "################################################################"
    @echo\ 
::@if "%COMPILE_TARGET_IC%" == "all" (
@echo ===================== Compile ALL =====================
@nmakeall 2>&1  | .\PRJ\mtee  /+ CompilelogX.txt
::) else (	
::@echo ===================== Compile Target %SEL% =====================
::@nmake %COMPILE_TARGET_IC% %SEL2%  2>&1  | mtee  /+ CompilelogX.txt
::)





:DisplayLog

:call :ColorText 0b "################################################################"
:@echo\ 
:call :ColorText 0b "######        If You wanna Save Log , press y     ############"
:@echo\ 
:call :ColorText 0b "################################################################"
:@e:cho\ 
::@echo Save Log (y/n) :
::@set /p note=
::@If '%note%'=='y' (
:@echo Save Log Yes/No in 5 sec  :


::CHOICE /t 5 /d y
goto sub_save


IF errorlevel 2 goto sub_abort
IF errorlevel 1 goto sub_save

:sub_save
@echo\ 
@copy Compilelog1.txt Compilelog2.txt
@copy Compilelog0.txt Compilelog1.txt
@copy CompilelogX.txt Compilelog0.txt
@echo\ 
call :ColorText 0b "------- Save Log Done !!! ---------------------"
@echo\ 
call :ColorText 0b "------- Show New Compile Result !!! -----------"
@echo\ 
goto LABEL_END

:sub_abort

@echo\ 
call :ColorText 0c "------- Save Log Abort !!! ---------------------"
@echo\ 
call :ColorText 0c "------- Show Last Compile Result !!! -----------"
@echo\ 
)
::@@@@@@@@@@@@@@@@@@@@@@@@@@
::ùùùø

:LABEL_END

call :ColorText 0f "######################################################################"
@echo\ 
call :ColorText 0f "###############    History Result Compare    #########################"
@echo\ 
call :ColorText 0f "######################################################################"
@echo\ 
call :ColorText 0f "         ùø  H-Type   ùø   L-Type  ùø    1T2R   ùø     C     ùø     CF    ùø     CK    ùø     H + C ùø"
@echo\ 
call :ColorText 0f "    Time ùø ROM    RAMùø ROM    RAMùø ROM    RAMùø ROM    RAMùø ROM    RAMùø ROM    RAMùø ROM    RAMùø"
::call :ColorText 0f "           text     data    bss    dec      hex  filename"
@echo\ 
set file=Compilelog0.txt
call:ExtractCompileLog

if exist Compilelog1.txt (
set file=Compilelog1.txt
call:ExtractCompileLog
)

if exist Compilelog2.txt (
set file=Compilelog2.txt
call:ExtractCompileLog
)


::========================================================================================
:: Dump Bin
::========================================================================================
IF "%COMPILE_OPTION%" == "RELEASE" (
cd Release\ZET7101
ReleaseBinBatch.bat
)

Goto :Eof





Goto :Eof


::#####################################################################
::#####################################################################
::#####################################################################
::#####################################################################
::#####################################################################
::#####################################################################





::======Show the newest compile Result =============================
:ExtractCompileLog

set "LineOfSet1="
set "LineOfSet2="
set "LineOfSet3="
set "LineOfSet4="
set "LineOfSet5="
set "LineOfSet6="
set "LineOfSet7="
set "LineOfSet8="
set "LineOfSet9="
set "LineOfSet10="
set "LineOfSet11="


set /a KeyWordCnt=0
::for /f "delims=:" %%a in ('findstr /n "RomSize" "%file%"') do (
::      if          not defined LineOfSet1 (set "LineOfSet1=%%a"      
::      ) else if   not defined LineOfSet2 (set "LineOfSet2=%%a"
::      ) else if   not defined LineOfSet3 (set "LineOfSet3=%%a"
::      ) else if   not defined LineOfSet4 (set "LineOfSet4=%%a"           
::      ) else if   not defined LineOfSet5 (set "LineOfSet5=%%a"
::      ) else if   not defined LineOfSet6 (set "LineOfSet6=%%a"      
::      ) else if   not defined LineOfSet7 (set "LineOfSet7=%%a"
::      ) else if   not defined LineOfSet8 (set "LineOfSet8=%%a"      
::	) else if   not defined LineOfSet9 (set "LineOfSet9=%%a"            
::      ) else if   not defined LineOfSet10 (set "LineOfSet10=%%a"
::	) else if   not defined LineOfSet11 (set "LineOfSet11=%%a")  
::     
::     set /a KeyWordCnt=KeyWordCnt+1
::)

for /f "delims=:" %%a in ('findstr /n "Device_Htype-LPitch" "%file%"') do (
      if          not defined LineOfSet1 (set "LineOfSet1=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_Ltype-LPitch" "%file%"') do (
      if          not defined LineOfSet2 (set "LineOfSet2=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-LPitch" "%file%"') do (
      if          not defined LineOfSet3 (set "LineOfSet3=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_Htype-SPitch" "%file%"') do (
      if          not defined LineOfSet4 (set "LineOfSet4=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_Ltype-SPitch" "%file%"') do (
      if          not defined LineOfSet5 (set "LineOfSet5=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-SPitch" "%file%"') do (
      if          not defined LineOfSet6 (set "LineOfSet6=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-SP-C" "%file%"') do (
      if          not defined LineOfSet7 (set "LineOfSet7=%%a"   
      )     
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-SP-CF" "%file%"') do (
      if          not defined LineOfSet8 (set "LineOfSet8=%%a"   
      )
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-SP-CK" "%file%"') do (
      if          not defined LineOfSet9 (set "LineOfSet9=%%a"   
      )
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_Htype-LP-C" "%file%"') do (
      if          not defined LineOfSet10 (set "LineOfSet10=%%a"   
      )
     set /a KeyWordCnt=KeyWordCnt+1
)
for /f "delims=:" %%a in ('findstr /n "Device_1T2R-LP-C" "%file%"') do (
      if          not defined LineOfSet11 (set "LineOfSet11=%%a"   
      )
     set /a KeyWordCnt=KeyWordCnt+1
)
::echo %LineOfSet1% %LineOfSet2% %LineOfSet3%



set /a WARNINGCnt=0
for /f "delims=:" %%a in ('findstr /n "UNCALLED unreferenced" "%file%" ') do (
     set /a WARNINGCnt=WARNINGCnt+1
)

set /a ERRORCnt=0
for /f "delims=:" %%a in ('findstr /n "error" "%file%" ') do (
     set /a ERRORCnt=ERRORCnt+1
)



::------------ show compile time -----------------------------------
@echo off & setlocal ENABLEEXTENSIONS
set LineNo=0
set "CompileTime="
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file% ') do (
  if not defined CompileTime set "CompileTime=%%a"
)


::------------ show compile option #1 ------------------------------
set LineNo=%LineOfSet1%
set "CompileResult1="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult1=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult1 set "CompileResult1=%%a"
	)
)
::------------ show compile option #2 ------------------------------
set LineNo=%LineOfSet2%
set "CompileResult2="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult2=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult2 set "CompileResult2=%%a"
	)
)

::------------ show compile option #3 ------------------------------
set LineNo=%LineOfSet3%
set "CompileResult3="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult3=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult3 set "CompileResult3=%%a"
	)
)
::------------ show compile option #4 ------------------------------
set LineNo=%LineOfSet4%
set "CompileResult4="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult4=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult4 set "CompileResult4=%%a"
	)
)
::------------ show compile option #5 ------------------------------
set LineNo=%LineOfSet5%
set "CompileResult5="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult5=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult5 set "CompileResult5=%%a"
	)
)
::------------ show compile option #6 ------------------------------
set LineNo=%LineOfSet6%
set "CompileResult6="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult6=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult6 set "CompileResult6=%%a"
	)
)

::------------ show compile option #7 ------------------------------
set LineNo=%LineOfSet7%
set "CompileResult7="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult7=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult7 set "CompileResult7=%%a"
	)
)

::------------ show compile option #8 ------------------------------
set LineNo=%LineOfSet8%
set "CompileResult8="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult8=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult8 set "CompileResult8=%%a"
	)
)

::------------ show compile option #9 ------------------------------
set LineNo=%LineOfSet9%
set "CompileResult9="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult9=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult9 set "CompileResult9=%%a"
	)
)
::------------ show compile option #10 ------------------------------
set LineNo=%LineOfSet10%
set "CompileResult10="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult10=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult10 set "CompileResult10=%%a"
	)
)

::------------ show compile option #11 ------------------------------
set LineNo=%LineOfSet11%
set "CompileResult11="
set/a LineNo-=0
if %LineNo% EQU 0 (
   set "CompileResult11=           "
) else (
	for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
	   if not defined CompileResult11 set "CompileResult11=%%a"
	)
)


::------------ show compile Checksum #1 ------------------------------
set LineNo=%LineOfChecksum1%
set "String1="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String1 set "String1=%%a"
)

::------------ show compile Checksum #2 ------------------------------
set LineNo=%LineOfChecksum2%
set "String2="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String2 set "String2=%%a"
)

::------------ show compile Checksum #3 ------------------------------
set LineNo=%LineOfChecksum3%
set "String3="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String3 set "String3=%%a"
)

::------------ show compile Checksum #4 ------------------------------
set LineNo=%LineOfChecksum4%
set "String4="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String4 set "String4=%%a"
)

::------------ show compile Checksum #5 ------------------------------
set LineNo=%LineOfChecksum5%
set "String5="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String5 set "String5=%%a"
)

::------------ show compile Checksum #6 ------------------------------
set LineNo=%LineOfChecksum6%
set "String6="
set/a LineNo-=1
for /f "delims=" %%a in ('more/e +%LineNo% ^< %file%') do (
   if not defined String6 set "String6=%%a"
)
::SET CompileChecksum1=%~1
::SET CompileChecksum1=%String1:~13,6%
::SET CompileChecksum2=%~1
::SET CompileChecksum2=%String2:~13,6%
::SET CompileChecksum3=%~1
::SET CompileChecksum3=%String3:~13,6%
::SET CompileChecksum4=%~1
::SET CompileChecksum4=%String4:~13,6%
::SET CompileChecksum5=%~1
::SET CompileChecksum5=%String5:~13,6%
::SET CompileChecksum6=%~1
::SET CompileChecksum6=%String6:~13,6%

SET CompileChecksum1=
SET CompileChecksum1=
SET CompileChecksum2=
SET CompileChecksum2=
SET CompileChecksum3=
SET CompileChecksum3=
SET CompileChecksum4=
SET CompileChecksum4=
SET CompileChecksum5=
SET CompileChecksum5=
SET CompileChecksum6=
SET CompileChecksum6=


SET Block= 




if  %ERRORCnt% EQU 0 (
	if  %WARNINGCnt% EQU 0 (
	goto :ShowResult
	) else (	
	goto :ShowWarningResult
	)
) else (
goto :ShowErrResult
)




:ShowResult
SETLOCAL EnableDelayedExpansion
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set "DEL=%%a"
)
call :ColorText 0A "O"
::@if "%COMPILE_TARGET_IC%" == "all" (


call :ColorText 02 "Lp"
echo %CompileTime%[%CompileResult1% %CompileResult2% %CompileResult3% %CompileResult7% %CompileResult9%]
call :ColorText 0e " Sp"
echo %CompileTime%[%CompileResult4% %CompileResult5% %CompileResult6% %CompileResult8% %CompileResult10%]



::) else (	
::@if "%SEL2%" == "1" (
::echo %CompileTime%[ %CompileResult1% ][                    ]
::) else (
::echo %CompileTime%[                    ][ %CompileResult1% ]
::) 
)


Goto :Eof


:ShowWarningResult
SETLOCAL EnableDelayedExpansion
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set "DEL=%%a"
)
call :ColorText 0b "X"
echo  %CompileTime% %CompileResult1% 
Goto :Eof

:ShowErrResult
SETLOCAL EnableDelayedExpansion
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set "DEL=%%a"
)

call :ColorText c0 "X"
call :ColorText 02 "Lp"
echo %CompileTime%[%CompileResult1% %CompileResult2% %CompileResult3% %CompileResult7% %CompileResult9%]
call :ColorText 0e " Sp"
echo %CompileTime%[%CompileResult4% %CompileResult5% %CompileResult6% %CompileResult8% %CompileResult10%]
Goto :Eof


:ColorText
echo off
<nul set /p ".=%DEL%" > "%~2"
findstr /v /a:%1 /R "^$" "%~2" nul
del "%~2" > nul 2>&1
Goto :Eof

@echo off
SETLOCAL EnableDelayedExpansion
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set "DEL=%%a"
)


:PackageBin


	

call :ColorText 0a "================================================================"
@echo\ 
call :ColorText 0a "===      Start Package Release Bin files                     ==="
@echo\ 
call :ColorText 0a "================================================================"
@echo\ 
	@set "PjID="
	@set "DataFlashH="
	@set "DataFlashL="
	@set BINpath=..\..\DEV\ZET7101\ZET_DEFAULT
	@set XMLpath=..\..\DEV\ZET7101
	@set SRCpath=..\..\SRC
	@set INCpath=..\..\INC
	@set LIBpath=..\..\LIB
copy %BINpath%\ZET7101_Ltype-SPitch.BIN  ZET7101.BIN

set $file=ZET7101.BIN
set $Choice=n
	::if exist hexadump.log (set /p $Choice=A dump file already exist, press y to renew it or enter to continue with the current :) else (set $Choice=y)
	::if /i !$Choice! EQU y (
		call :GetSize !$file!
		echo %time% Dumping file in hexa !$file!, !$GetSize! bytes...
		set /a $GetSize/=2
		>nul 2>nul del /Q /F !$file!.nul
		>nul fsutil file createnew !$file!.nul !$GetSize!
		>!$file!2.nul sort !$file!.nul
		call :GetSize !$file!
		>nul 2>nul del /Q /F !$file!.nul
		>nul fsutil file createnew !$file!.nul !$GetSize!	
		>comparison.log fc /b !$file! !$file!.nul
		>comparison2.log fc /b !$file! !$file!2.nul
		>comparison3.log type comparison.log
		>>comparison3.log type comparison2.log
		>hexadump.log (sort comparison3.log | findstr /v /i nul)
		>nul 2>nul del /Q /F comparison.log
		>nul 2>nul del /Q /F comparison2.log
		>nul 2>nul del /Q /F comparison3.log
		::echo %time% Dump done
	::)
	set /a $LastOffset-=1
	for /f "tokens=1*" %%i in ('type hexadump.log ^| more +%$LastOffset%') do (set $LastOffset=%%i)
	set $LastOffset=%$LastOffset:~,8%
	
	@call:GetOffset 0000E002 PjID
	@call:GetOffset 0000E08C DataFlashH
	@call:GetOffset 0000E08D DataFlashL
	@call:GetOffset 0000E08E DataFlashSubH
	@call:GetOffset 0000E08F DataFlashSubL

	
	@set /a DFH=0x%DataFlashH%
	@set /a DFL=0x%DataFlashL%	
	@set /a DataflashNo = %DFH%+%DFL%*256
	@set /a DFSH=0x%DataFlashSubH%
	@set /a DFSL=0x%DataFlashSubL%	
	@set /a DataflashSubNo = %DFSH%+%DFSL%*256
@del *.nul
@del *.log
@del *.bin


set "TargePath=00%DataflashNo%.%DataflashSubNo%"
@mkdir  %TargePath% 

@cd %TargePath%
@echo off 
@copy ..\%BINpath%\ZET7101_Ltype-SPitch.BIN  ZET7101_Ltype-SPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_Ltype-LPitch.BIN  ZET7101_Ltype-LPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_Htype-SPitch.BIN  ZET7101_Htype-SPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_Htype-LPitch.BIN  ZET7101_Htype-LPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_1T2R-SPitch.BIN   ZET7101_1T2R-SPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_1T2R-LPitch.BIN   ZET7101_1T2R-LPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_F3D-LPitch.BIN    ZET7101_F3D-LPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_F3D-SPitch.BIN    ZET7101_F3D-SPitch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_ForceTouch.BIN    ZET7101_ForceTouch.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_1T2R-SP-C.BIN    ZET7101_1T2R-SP-C.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%BINpath%\ZET7101_1T2R-SP-CF.BIN    ZET7101_1T2R-SP-CR.v0%DataflashNo%.%DataflashSubNo%.bin
@copy ..\%XMLpath%\ZetDF_0%DataflashNo%.XML
::@rar a -r ZET7101_v%DataflashNo%_XML.rar *.bin *.XML

@echo\ 
@echo\ 
call :ColorText 0a "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
@echo\ 
call :ColorText 0a "@@@@@@@@@@@   Release Bin is ready            @@@@@@@@@"
@echo\ 
call :ColorText 0a "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
@echo\ 



Goto :Eof

:ColorText
echo off
<nul set /p ".=%DEL%" > "%~2"
findstr /v /a:%1 /R "^$" "%~2" nul
del "%~2" > nul 2>&1
Goto :Eof


:GetSize
			set $GetSize=%~z1
exit /b


:GetOffset
			setlocal
			set $Offset=%1
			for /f "tokens=1,2,*" %%i in ('type hexadump.log ^| findstr /i !$Offset!') do (set $GetOffset=%%j)
			::if "%$GetOffset%" NEQ "" (echo Offset !$Offset! value : [%$GetOffset%]) else (echo Offset !$Offset! is not available)
			::if !$Offset! EQU "31746" (set "PjCode3=GetOffset") else (echo ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR)
			::set "PjCode3=%$GetOffset%"			
			::endlocal& set GetOffset=%$GetOffset%
			ENDLOCAL&set "%~2=%$GetOffset%"			
exit /b

:Dec2Hexa
			set LOOKUP=0123456789abcdef &set HEXSTR=&set PREFIX=
			set /a A=%* || exit /b 1
			if !A! LSS 0 set /a A=0xfffffff + !A! + 1 & set PREFIX=f
			:loop
				set /a B=!A! %% 16 & set /a A=!A! / 16
				set HEXSTR=!LOOKUP:~%B%,1!%HEXSTR%
				if %A% GTR 0 goto :loop
			set $Dec2Hexa=%PREFIX%%HEXSTR%
exit /b

:Length
			set $Length.Arguments=%*
			set $Length.Length=0
			for /l %%i in (0,1,9000) do (
					if "!$Length.Arguments:~%%i,1!" NEQ "" (set /a $Length.Length+=1)
			)
exit /b
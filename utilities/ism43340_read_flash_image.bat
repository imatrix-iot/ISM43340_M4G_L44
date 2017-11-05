@echo off
setlocal enabledelayedexpansion

rem tools folder level
set WLEVEL=..\..\..
set LLEVEL=../../..

rem Product
set sname=ism43340
set mcu=stm32f4x
set fsize=1048576

rem Check command line
if [%1] == [] goto error
goto clean

:error
rem Input error
echo.
echo Usage: !sname!_read_flash_image {file name prefix}
echo        ex. !sname!_read_flash_image !sname!_m4g_l44
echo            file name: !sname!_m4g_l44_flash_image.bin
echo.
goto cleanup

:clean
rem Clean if exsiting file name 
del /q !fname!_flash_image.bin

rem File name
set fname=%1_flash_image.bin
echo.
echo Reading to: !fname!

rem Read flash image
!WLEVEL!\tools\OpenOCD\Win32\openocd-all-brcm-libftdi.exe -f !WLEVEL!\tools\OpenOCD\BCM9WCD1EVAL1.cfg -f !WLEVEL!\tools\OpenOCD\!mcu!.cfg -f !WLEVEL!\tools\OpenOCD\!mcu!-flash-app.cfg -c "flash read_bank 0 "!fname!" 0 "!fsize! -c shutdown > read.log 2>&1

:cleanup
rem Erase varaibles
set mcu=
set fsize=
set fname=
set WLEVEL=
set LLEVEL=
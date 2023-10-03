@echo off

call install-fs.bat
if not %ErrorLevel% equ 0 (
   exit /b 1
)

call install-wifi_s3.bat
if not %ErrorLevel% equ 0 (
   exit /b 1
)
echo Starting fluidterm
win64\fluidterm.exe

pause

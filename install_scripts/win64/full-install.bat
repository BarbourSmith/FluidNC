@echo off

echo *  ------ Erasing flash  *
call erase.bat
if not %ErrorLevel% equ 0 (
   exit /b 1
)

echo *  ------- Installing filesystem   *
call install-fs.bat
if not %ErrorLevel% equ 0 (
   exit /b 1
)

echo *  ------- Installing firmware   *
call install-wifi_s3.bat
if not %ErrorLevel% equ 0 (
   exit /b 1
)

echo *  ------- Done   *
echo Starting fluidterm
win64\fluidterm.exe

pause
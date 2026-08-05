@echo off
setlocal
del sdkconfig
call C:\esp\v5.5.3\esp-idf\export.bat
if errorlevel 1 exit /b %errorlevel%
idf.py -B build-esp32p4-pre-v3 -D SDKCONFIG=sdkconfig.esp32p4-pre-v3 -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config\esp32p4_rev_pre_v3.defaults" set-target esp32p4 build
set BUILD_RESULT=%errorlevel%
endlocal & exit /b %BUILD_RESULT%

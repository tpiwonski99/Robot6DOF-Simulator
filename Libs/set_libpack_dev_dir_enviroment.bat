echo off & cls
setx ROBOT_SIM_DEV "%~dp0\"
if not errorlevel 1 echo ROBOT_SIM_DEV: %~dp0
pause

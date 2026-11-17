@echo off
cd /d "%~dp0"
py -3 interface_exo.py COM6
if errorlevel 1 python interface_exo.py COM6
if errorlevel 1 pause

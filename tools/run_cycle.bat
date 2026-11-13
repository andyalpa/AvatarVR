@echo off
rem Double-click to build, load-proof, deploy and launch. Status: ..\build\cycle_status.txt
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_cycle.ps1"
timeout /t 60

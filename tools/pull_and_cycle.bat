@echo off
rem Double-click to pull the latest changes, then build, load-proof, deploy and launch.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pull_and_cycle.ps1"
timeout /t 60

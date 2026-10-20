@echo off
REM Double-click to start the EMF scanner. First run installs what it needs.
cd /d "%~dp0"
if not exist .venv ( py -m venv .venv )
call .venv\Scripts\activate
python -m pip install -q -r requirements.txt
python emf_scan.py %*
pause

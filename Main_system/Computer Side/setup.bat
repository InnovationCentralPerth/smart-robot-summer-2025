@echo off
cd /d "%~dp0"
echo Installing dependencies...
pip install -r requirements.txt
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Error installing dependencies. Please check your Python installation.
) else (
    echo.
    echo Dependencies installed successfully.
)
pause

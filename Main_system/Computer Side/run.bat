@echo off
cd /d "%~dp0"
echo ===================================================
echo   Smart Robot Control System
echo ===================================================
echo.
echo 1. Start Main Application (Voice/AI)
echo 2. Start Robot Bridge (MQTT8)
echo 3. Start Mock Robot (for testing without hardware)
echo.
set /p choice="Select an option (1-3): "

if "%choice%"=="1" (
    echo Starting Main Application...
    python -m smart_stacker_frontend.main
) else if "%choice%"=="2" (
    echo Starting Robot Bridge...
    cd MQTT8
    python main.py
    cd ..
) else if "%choice%"=="3" (
    echo Starting Mock Robot...
    python -m smart_stacker_frontend.mock_mqtt
) else (
    echo Invalid choice.
)

pause

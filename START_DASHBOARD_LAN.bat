@echo off
setlocal
cd /d "%~dp0"

echo [1/3] Checking Python dependencies...
python -c "import fastapi,uvicorn,serial,paho.mqtt.client,amqtt" >nul 2>&1
if errorlevel 1 (
    python -m pip install -r dashboard\requirements.txt
    if errorlevel 1 goto :failed
)

echo [2/3] Checking dashboard assets...
if not exist "dashboard\node_modules\three\build\three.module.min.js" (
    call npm install --prefix dashboard
    if errorlevel 1 goto :failed
)

netsh advfirewall firewall show rule name="Smart Trolley Dashboard LAN" >nul 2>&1
if errorlevel 1 (
    echo Windows will request administrator permission once for LAN firewall rules.
    powershell -NoProfile -Command "Start-Process powershell -Verb RunAs -Wait -ArgumentList '-NoProfile -ExecutionPolicy Bypass -File ""%~dp0scripts\configure_lan_firewall.ps1""'"
    if errorlevel 1 goto :failed
)

echo [3/3] Starting dashboard and embedded MQTT broker...
echo Open http://192.168.0.100:8000 from a device on the same LAN.
python dashboard\main.py
goto :end

:failed
echo.
echo Startup failed. Read the error above, then press any key.
pause >nul

:end
endlocal

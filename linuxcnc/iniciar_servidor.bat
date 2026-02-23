@echo off
echo Iniciando servidor puente RTCP y simulador grblHAL...

:: Lanzar en una nueva ventana/background el cargador de configuraciones
call python c:\simulador\linuxcnc\grbl_capture.py
pause
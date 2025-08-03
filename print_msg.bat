@echo off

::set TCL_PATH=tcl
::set INTERFACE=interface/stlink.cfg
set TARGET=board/stm32f4discovery.cfg

openocd -f %TARGET% -c "init" -c "arm semihosting enable" -c "reset"

pause
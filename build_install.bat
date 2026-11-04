@echo off
setlocal

call conda-build .conda
if errorlevel 1 exit /b 1

call conda install --use-local --freeze-installed --force-reinstall gnuradio-dl5eu
if errorlevel 1 exit /b 1

endlocal
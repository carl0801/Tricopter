@echo off
setlocal
set start=%time%

echo Building the project...
mingw32-make -f ./cpp/Makefile

set end=%time%

:: Split the start and end times into hours, minutes, and seconds
for /f "tokens=1-3 delims=.,:" %%a in ("%start%") do (
    set /a start_h=%%a
    set /a start_m=%%b
    set /a start_s=%%c
)
for /f "tokens=1-3 delims=.,:" %%a in ("%end%") do (
    set /a end_h=%%a
    set /a end_m=%%b
    set /a end_s=%%c
)

:: Calculate the elapsed time in seconds
set /a elapsed_s=(end_h*3600 + end_m*60 + end_s) - (start_h*3600 + start_m*60 + start_s)
echo.
echo Elapsed time: %elapsed_s% seconds


echo Running the compiled executable in python simulation...
python ./simulation/simulation.py

endlocal
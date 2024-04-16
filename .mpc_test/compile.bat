@echo off
setlocal

rem Set the name of your precompiled header file
set PCH_FILE=droneModel.hpp

rem Set the name of your precompiled header output file
set PCH_OUTPUT_FILE=%PCH_FILE%.gch

rem Set the name of your source file
set SOURCE_FILE=IPM.cpp

rem Set the name of the output executable
set OUTPUT_FILE=IPM

rem Check if the precompiled header needs to be created or updated
if not exist %PCH_OUTPUT_FILE% (
    echo Creating precompiled header...
    g++ -c %PCH_FILE% -o %PCH_OUTPUT_FILE%
) else (
    echo Header file already exists!
)

rem Compile the source file using the precompiled header
g++ %SOURCE_FILE% -o %OUTPUT_FILE% -include %PCH_FILE%

rem Run the compiled executable
.\%OUTPUT_FILE%

endlocal
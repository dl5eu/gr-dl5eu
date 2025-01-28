setlocal EnableDelayedExpansion
@echo on

set PREFIX="%CONDA_PREFIX%"
set LIBRARY_PREFIX="%CONDA_PREFIX%\Library"
set SP_DIR="%CONDA_PREFIX%\Lib\site-packages"
set CPU_COUNT=2

:: Make a build folder and change to it
cmake -E make_directory buildconda
cd buildconda

:: configure
cmake -G "Ninja" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
    -DCMAKE_PREFIX_PATH="%LIBRARY_PREFIX%" ^
    -DGR_PYTHON_DIR="%SP_DIR%" ^
    -DENABLE_DOXYGEN=OFF ^
    -DENABLE_TESTING=OFF ^
    ..
if errorlevel 1 exit 1

:: build
cmake --build . --config Release -- -j%CPU_COUNT%
if errorlevel 1 exit 1

:: install
cmake --build . --config Release --target install
if errorlevel 1 exit 1

:: test
::ctest --build-config Release --output-on-failure --timeout 120 -j%CPU_COUNT%
::if errorlevel 1 exit 1

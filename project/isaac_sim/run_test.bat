@echo off
REM SO-ARM101 Isaac Sim 테스트 실행 스크립트
REM 이 파일을 더블클릭하거나 CMD에서 실행하세요.

set ISAAC_DIR=D:\isaac-sim
set SCRIPT=%~dp0so_arm_test.py

echo ====================================================
echo  SO-ARM101 Isaac Sim 가상환경 테스트
echo ====================================================
echo.
echo Isaac Sim 디렉토리: %ISAAC_DIR%
echo 스크립트: %SCRIPT%
echo.

if not exist "%ISAAC_DIR%\python.bat" (
    echo [오류] %ISAAC_DIR%\python.bat 를 찾을 수 없습니다.
    echo        Isaac Sim 설치 경로를 확인하세요.
    pause
    exit /b 1
)

cd /d "%ISAAC_DIR%"
python.bat "%SCRIPT%"

echo.
echo 테스트 종료.
pause

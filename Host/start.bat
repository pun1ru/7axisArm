@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

rem ---- 找 Python：先 python，再 py 启动器
set "PY="
where python >nul 2>nul && set "PY=python"
if not defined PY (
    where py >nul 2>nul && set "PY=py -3"
)
if not defined PY (
    echo 没找到 Python，请先安装 Python 3.10 以上并勾选 Add to PATH
    pause
    exit /b 1
)

rem ---- 缺依赖就自动装
%PY% -c "import PySide6, pyqtgraph, serial, numpy" >nul 2>nul
if errorlevel 1 (
    echo 缺少依赖，正在安装 requirements.txt ...
    %PY% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo 依赖安装失败，请检查网络或手动执行: %PY% -m pip install -r requirements.txt
        pause
        exit /b 1
    )
)

rem ---- 启动；参数原样透传，例如 start.bat --mock / start.bat --port COM5
%PY% -m armhost.app %*
if errorlevel 1 (
    echo.
    echo 程序异常退出，日志见 %~dp0logs\armhost.log
    pause
)
endlocal

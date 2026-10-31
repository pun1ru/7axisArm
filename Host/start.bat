@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

rem ---- 找 Python：优先 3.11；3.14 + PySide6 6.11 存在原生崩溃问题
set "PY="
where py >nul 2>nul && py -3.11 -c "import sys" >nul 2>nul && set "PY=py -3.11"
if not defined PY (
    where python >nul 2>nul && python -c "import sys; sys.exit(0 if sys.version_info[:2]==(3,11) else 1)" >nul 2>nul && set "PY=python"
)
if not defined PY (
    echo 没找到 Python 3.11。请安装 Python 3.11，或确保 py -3.11 可用
    pause
    exit /b 1
)

rem ---- 缺依赖或版本不对就按锁文件安装
%PY% -c "import PySide6,pyqtgraph,serial,numpy; assert PySide6.__version__=='6.8.3'; assert pyqtgraph.__version__=='0.14.0'; assert numpy.__version__=='2.4.6'; assert serial.__version__=='3.5'" >nul 2>nul
if errorlevel 1 (
    echo 缺少依赖或版本不匹配，正在安装 requirements.lock ...
    %PY% -m pip install -r requirements.lock
    if errorlevel 1 (
        echo 依赖安装失败，请检查网络或手动执行: %PY% -m pip install -r requirements.lock
        pause
        exit /b 1
    )
)

rem ---- 无控制台启动；使用与依赖检查相同的解释器
set "PYW="
for /f "usebackq delims=" %%I in (`%PY% -c "import pathlib,sys;print(pathlib.Path(sys.executable).with_name('pythonw.exe'))"`) do set "PYW=%%I"
if defined PYW if exist "%PYW%" (
    start "" /D "%~dp0" "%PYW%" -m armhost.app %*
    exit /b 0
)

rem ---- 没有 pythonw 时退回控制台启动
%PY% -m armhost.app %*
if errorlevel 1 (
    echo.
    echo 程序异常退出，日志见 %~dp0logs\armhost.log
    pause
)
endlocal

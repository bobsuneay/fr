@echo off
REM Windows-only PyQt preview; ROS/MoveIt and real hardware require Ubuntu.
cd /d "%~dp0.."
python -m fr3_control_panel.app --demo
pause

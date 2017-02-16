@echo off
for /F "usebackq tokens=1,2 delims==" %%i in (`wmic os get LocalDateTime /VALUE 2^>NUL`) do if '.%%i.'=='.LocalDateTime.' set ldt=%%j
set ldt=%ldt:~0,4%-%ldt:~4,2%-%ldt:~6,2%_%ldt:~8,2%:%ldt:~10,2%:%ldt:~12,6%
echo Datetime: %ldt%

for /F "delims=" %%i in ('md5.exe -d%ldt%') do set hash=%%i
echo MD5 hash: %hash%
set uid=%hash:~-6%
echo UID:      %uid%
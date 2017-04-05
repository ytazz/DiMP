@echo on

rem
rem 配布用ファイルをpack/にコピー
rem

rmdir /S /Q pack

xcopy       Readme.txt  pack\
xcopy       DiMP10.sln  pack\
xcopy       SampleApp.h pack\
xcopy /E /Y DiMP2       pack\DiMP2\    /EXCLUDE:exclude.list
xcopy /E /Y Examples    pack\Examples\ /EXCLUDE:exclude.list

cls

pause
exit

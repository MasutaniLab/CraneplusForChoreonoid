@echo off
: CraneplusForChoreonoid�̃e�X�g

:�l�[�~���O�T�[�r�X�̊m�F
rtls /localhost > nul
if errorlevel 1 (
  echo �l�[�~���O�T�[�o��������܂���
  pause
  exit /b 1
  rem /b�I�v�V�����͐e���I��点�Ȃ����߂ɕK�{
)

:��ƃf�B���N�g�����o�b�`�t�@�C���̂���ꏊ�֕ύX
cd %~dp0

:Choreonoid�N��
start "Choreonoid" choreonoid craneplus.cnoid

:�R���|�[�l���g�̋N��
start "" TkSliderCommand.pyw
start "" TkSliderSpeed.pyw
start "" TkMonitorSliderPosition.pyw
start "" TkMonitorSliderMoving.pyw

:�R���|�[�l���g����ϐ���
set c=/localhost/TkSliderCommand0.rtc
set s=/localhost/TkSliderSpeed0.rtc
set p=/localhost/TkMonitorSliderPosition0.rtc
set m=/localhost/TkMonitorSliderMoving0.rtc
set d=/localhost/CRANE-AX12IoRTC.rtc

:���ԑ҂�
timeout 10

:�ڑ�
rtcon %c%:slider %d%:goalPosition
rtcon %s%:slider %d%:movingSpeed
rtcon %d%:presentPosition %p%:value
rtcon %d%:moving %m%:value

:�A�N�e�B�x�[�g
rtact %c% %s% %p% %m%

echo Choreonoid�ŃV�~�����[�V�������J�n���Ă��������D

:loop
set /p ans="�I�����܂����H (y/n)"
if not "%ans%"=="y" goto loop

rtdeact %c% %s% %p% %m%

:�I���irtexit�́C����������j
for %%i in (%c% %s% %p% %m%) do (
  rtexit %%i
)

:Choreonoid�I��
taskkill /im choreonoid.exe

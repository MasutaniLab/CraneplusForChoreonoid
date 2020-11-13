@echo off
: CraneplusForChoreonoidのテスト

:ネーミングサービスの確認
rtls /localhost > nul
if errorlevel 1 (
  echo ネーミングサーバが見つかりません
  pause
  exit /b 1
  rem /bオプションは親を終わらせないために必須
)

:作業ディレクトリをバッチファイルのある場所へ変更
cd %~dp0

:Choreonoid起動
start "Choreonoid" choreonoid craneplus.cnoid

:コンポーネントの起動
start "" TkSliderCommand.pyw
start "" TkSliderSpeed.pyw
start "" TkMonitorSliderPosition.pyw
start "" TkMonitorSliderMoving.pyw

:コンポーネント名を変数化
set c=/localhost/TkSliderCommand0.rtc
set s=/localhost/TkSliderSpeed0.rtc
set p=/localhost/TkMonitorSliderPosition0.rtc
set m=/localhost/TkMonitorSliderMoving0.rtc
set d=/localhost/CRANE-AX12IoRTC.rtc

:時間待ち
timeout 10

:接続
rtcon %c%:slider %d%:goalPosition
rtcon %s%:slider %d%:movingSpeed
rtcon %d%:presentPosition %p%:value
rtcon %d%:moving %m%:value

:アクティベート
rtact %c% %s% %p% %m%

echo Choreonoidでシミュレーションを開始してください．

:loop
set /p ans="終了しますか？ (y/n)"
if not "%ans%"=="y" goto loop

rtdeact %c% %s% %p% %m%

:終了（rtexitは，引数を一つずつ）
for %%i in (%c% %s% %p% %m%) do (
  rtexit %%i
)

:Choreonoid終了
taskkill /im choreonoid.exe

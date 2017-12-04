# ロボットアームCRANE+のChoreonoid用シミュレーションモデル

大阪電気通信大学  
升谷 保博  
2017年12月4日

## はじめに

- オープンソースのロボット用統合GUIソフトウェア[Choreonoid](http://choreonoid.org/ja/)を用いて，アールティ社の5軸ロボットアームCRANE+(販売終了？)の動力学シミュレーションを行うためのモデル一式です．
- 以下の環境で開発，動作確認しています．
  - Windows 10 64bit版
  - Visual Studio 2012
  - OpenRTM-aist 1.1.2 64bit版
  - Choreonoid 1.5
- VRMLモデルは名城大学の大原研究室が公開している[CRANE-simulation](https://github.com/rsdlab/CRANE-simulation)に含まれている[モデル](https://github.com/rsdlab/CRANE-simulation/tree/master/model_project/CRANE_Model)を基にしています．
- ChoreonoidのボディRTCのコントローラモジュールとして使うRTコンポーネント（DynamixelSim）は独自に作成したものです．
- 以上を利用するChorenoidのプロジェクトファイルも同梱しています．

## 内容物

- ロボットと環境のVRMLモデル
  - `model/*.wrl`
  - `floor.wrl`
  - `bottle.wrl`
- ボディRTCのコントローラモジュールのソースコード一式
  - ディレクトリ `DynamixelSim`
- ボディRTCの設定ファイル
  - `CRANE.conf`
- Choreonoidのプロジェクトファイル
  - `craneplus.cnoid`

## インストール

- [OpenRTM-aist 1.1.2](http://www.openrtm.org/openrtm/ja/node/6034)をインストール．
- [Choreonoid 1.5をインストール](http://choreonoid.org/ja/manuals/1.5/install/install.html)．
  - 2017年10月31日にバージョン1.6.0がリリースされましたが，まだ動作確認できていません．
  - Windowsの場合，Choreonoid本体をビルドしたVisual Stduioのバージョンや構成と，ボディRTCのコントローラモジュールをビルドするVisual Stduioのバージョンや構成が異なっていると，動作しません．配布されているWindows用のインストーラは`Choreonoid-1.5.0-win64.exe`は，Visual Studio 2013でビルドされています．
- [CraneplusForChoreonoid](https://github.com/MasutaniLab/CraneplusForChoreonoid)
をクローンかダウンロードする．
- CMake
  - ソースディレクトリはトップの下の`DynamixelSim`
  - ビルドディレクトリは`DynamixelSim/build`
  - Windowsの場合
    - ConfigureはVisual Studio 64bit
    - `DynamixelSim/build/DynamixelSim.sln`をVisual Studioで開く．
    - 構成をReleaseにしてビルド．
    - 正常終了したら，プロジェクト`INSTALL`をビルド．
  - Linuxの場合
    - `mkdir build; cd build; cmake ..; make; make install`
  - WindowsとLinuxでプロジェクトファイルを共通にするために，コントローラモジュールの動的ライブラリ（Windowsではdllファイル，Linuxではsoファイル）を`DynamixelSim/build/module`にインストールするようにしています．

## 使い方

- 適切なバージョンのChoreonoidを起動し，プロジェクトファイル`craneplus.cnoid`を読み込みます．
- メッセージビューにエラーが出ていないか確認します．
- `DynamixelSim`コンポーネントのポートを他のコンポーネントのポートに接続します．
- `DynamixelSim`コンポーネントの外部接続用のポートは以下の通りです．
  - 入力ポート
    - goalPosition
      - 型: RTC::TimedUShortSeq
      - 概要： 各アクチュエータへの位置指令．単位はAX-12の内部値と同じ．
    - movingSpeed
      - 型: RTC::TimedUShortSeq
      - 概要： 各アクチュエータへの速度指令．単位はAX-12の内部値と同じ．

  - 出力ポート
    - presentPosition
      - 型: RTC::TimedUShortSeq
      - 概要： 各アクチュエータの現在位置．単位はAX-12の内部値と同じ．
    - moving
      - 型: RTC::TimedUShortSeq
      - 概要： 各アクチュエータが動いているかどうかのフラグ．
- `DynamixelSim`のコンポーネントの仕様は，作者が公開している[Dynamixel
](https://github.com/MasutaniLab/Dynamixel)コンポーネントと互換になるようにしています．

## 既知の問題・TODO

- 位置と速度の指令に対して台形則による軌道を計算し，それに追従するようなPID制御を行っていますが，チューニングが十分ではありません．

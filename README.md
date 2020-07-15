# arduino-can
based on Seedstudio can-bus shield library

## MIT cheetahモーターHT-03を駆動するためのプログラム群

### 定常応答を調べるためのプログラム

#### HT-03_CAN_staytest0

サーボ制御のプロトタイプ．チュートリアルとしてこのプログラムを動かすところから始めるとよい

#### HT-03_CAN_staytest

サーボ制御のデータをCSVで保存することを想定したプログラム
HT-03_CAN_staytest0と出力のところだけが違う

### 周波数応答を調べるためのプログラム

#### HT-03_CAN_freqResp

位置入力を $\theta=A\sin(\omega t)$ として，振動させるプログラム

### シリアル通信を受け取るためのMATLABプログラム

#### SerialRead.m

指定された行数のシリアル通信を読み込み，Data配列に保存していくプログラム．主に周波数応答用

### 複数のモーターを同時に動かすためのプログラム

#### HT-03_CAN_DualTest

2個のモーターを同時に動かすためのプログラム


## RMD-X8を駆動するためのプログラム群

### RMD-X8_CAN_moveTest

サーボ制御のプロトタイプ
# ミヤムラメモ
次にやること
<!-- 1. Chassis.cpp の中身を変更する(一関数内で単位をそろえる) -->
<!-- 1. Gimbal.h Gimbal.cpp の単位をそろえる -->
1. タイヤ一個に対しての制御系を組む
1. 実際に足回りを動かす

# ミヤムラがやったこと
- M3508のMECを組んだ。\
    $`P(s) = \frac{4.3}{0.25s+1}`$ とした。\
    入力は電流値(-16384 ~ 16384)で、出力は rpmである。\
    補償器にはPD制御を用いた。\
    電流値(A)から電流値(-16384 ~ 16384)への変換は$`\frac{16384}{20}`$を掛ける。

# RM2020 歩兵ライブラリ
- Teensy4.0で動作する(予定の)制御プログラムです。
- ライブラリのインストールについては次を参照してください。
    - http://www.arduino.cc/en/Guide/Libraries
- ハードウェア構成と簡単な各マイコンの役割は以下のようになっています
    ![hard](..\out\plantUML\total\total.png)

# 制御ルール
- Yaw軸のgimbalの角度は、上から見て時計回りを正とする。
- Pitch軸のgimbalの角度は、上に向く方が正とする。
- 単位系は( m, sec, rad, Ampare)に統一
    - encoder8192とか、eularとかと混同しないように。

# 各ライブラリの関数
## Chassis
単位そろえた！

|関数|説明|
|:-|:-|
|void Chassis::doChassisTask(void)| ループ内で行う一連の足回りの計算|
|void setRobotVector(double x,double y,double r)| ロボットの各速度成分を入力する|
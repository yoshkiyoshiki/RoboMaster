# GM6020.hの説明
このライブラリを使うことでGM6020モーターの速度制御と位置制御が可能になります。\
また、GM6020クラスはMotorクラスを継承しています。\
クラスを継承しているので、元になるクラスの関数もすべて使えます。\
サンプルプログラムは同一フォルダ内のexampleに、速度制御と位置制御を行っているプログラムを置いています。

# Motor
|関数|説明|
|:-|:-|
|void setRPM(int16_t rpm)| ESCから返ってきたRPM値を保存する|
|void setPosition(int16_t position)|ESCから返ってきた絶対位置を保存する|
|void setCurrent(int16_t current)|ESCから返ってきた電流値を保存する|
|void setTemperature(int16_t temperature)|ESCから返ってきた温度を保存する|
|double getRPM()|保存されているRPM値を返す|
|double getPosition()|保存されている絶対位置を返す|
|double getCurrent()|保存されている電流値を返す|
|double getTemperature()|保存されている温度を返す|

# GM6020
|関数|説明|
|:-|:-|
|GM6020( double _deltaT, int16_t _neutral)| コンストラクタ。引数は制御間隔[sec],初期位置(単位はESCの分解能)を入れる。|
|void setPIDgain_mec( double _kp, double _ki, double _kd)| 制御系内にあるモデル誤差抑制補償器のPIDゲインを設定。ただしI制御は実装してない。。。|
|void setPIDgain_speed( double _kp, double _ki, double _kd)| 速度制御をするためのPIDゲインを設定。|
|void setPIDgain_position( double _kp, double _ki, double _kd)| 位置制御をするためのPIDゲインを設定。|
|void setNeutral(int16_t _neutral)| 原点の変更用関数。この関数を呼び出した場合、絶対位置の値が初期化される。|
|int getGeneralPosition()| 一回転以上したときの絶対位置を返す。|
|int getGeneralPosition_limited()| 一回転以上したときの原点からの偏差を返す。(-4096 ~ 4096)|
|int16_t culcAmpareFromRPM( double targetRPM)| 実際に計算を行う関数。必ず制御間隔ごとに呼び出すこと！引数は目標RPM値。返り値はESCへの入力値(-16384~16384)。|
|int16_t culcAmpareFromRPM_4Gimbal( double targetRPM, double gyro)| ジンバル用の実際に計算を行う関数。必ず制御間隔ごとに呼び出すこと！引数は目標RPM値,IMUの角速度[rad/sec]。返り値はESCへの入力値(-16384~16384)。|
|int16_t culcAmpareFromPosition( double targetPosition)| 実際に計算を行う関数。必ず制御間隔ごとに呼び出すこと！引数は目標位置(単位はESCの分解能)。返り値はESCへの入力値(-16384~16384)。|
|int16_t culcAmpareFromPosition_4Gimbal( double targetPosition, double position, double gyro)| ジンバル用の実際に計算を行う関数。引数は目標位置(単位はESCの分解能),IMUでの現在地(単位はESCの分解能),IMUの角速度[rad/sec]。返り値はESCへの入力値(-16384~16384)。|


# ライブラリ使用時の注意点
1. 必ずインスタンス生成時に入れた制御間隔でculcAmpareFrom~~関数を呼び出してください。内部でモーターの数式モデルを利用しており、適正に呼ばれないと計算がバグります。
1. 必ずESCから返ってくるデータの更新は密に行ってください。これらのデータをもとにモーターの数式モデルを更新しているので、適正に呼ばれないと計算がバグります。
1. 一定間隔で関数を呼び出し、かつデータの更新を密に行うために、**Metro**の利用を推奨します。
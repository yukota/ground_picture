# ground_picture
Google static mapから取得した画像をGround plane
上に描写するgazeboプラグインです。

![Demo](https://github.com/yukota/ground_picture/wiki/images/demo.jpg)

## Requirement
- gazebo 7.3.1+
- OpenCV 2.4+
- cpp-netlib 0.11.2+
- png++ 0.2.5+

## Usage
### Google Static Maps APIのAPI Key取得
[Google Static Maps](https://developers.google.com/maps/documentation/static-maps)からAPI Keyを取得してください。

### パラメータの設定
以下のパラメータをsdfに記述します。
- API Key(Required)
取得したGoogle Static MapsのAPI Keyを記述してください。
- 緯度(Required)
地図の中心の経度。
- 経度(Required)
地図の中心の緯度。
- 解像度(Optional)
1メートルあたりのピクセル数(整数値)で指定します。
解像度が高い場合、表示座標によってはGoogle Static Mapsが指定した解像度の画像を
持っていない場合があるため、そうした場合は解像度を下げてください。

models/ground_picture/model.sdf内の以下の場所を書き換えてください。
~~~xml
<plugin name="ground_picture" filename="libground_picture.so">
 <!-- Goole Static APIのAPI Key -->
 <apikey>Your Google Static API Key</apikey>
 <!-- 経度  -->
 <longitude>35.656025</longitude>
 <!-- 緯度  -->
 <latitude>139.7971489</latitude>
 <!-- 解像度 -->
 <pixels_per_meter>2</pixels_per_meter>
</plugin>
~~~

### worldへの組み込み
sample/sample.worldを参考に使用するworldを記述したsdfに組み込んでください。
~~~xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_picture</uri>
    </include>
  </world>
</sdf>
~~~

## Installation
ソースを取得します。
~~~sh
git clone git@github.com:yukota/ground_picture.git
~~~

ソースをビルドします。
~~~sh
cd ground_picture
mkdir build
cd build
cmake ..
make
~~~

ビルドしたプラグインをgazeboに認識させるため、環境変数を設定します。
~~~sh
# path/to/clonedの部分をソースをcloneした場所に合わせて書き換えてください。
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/path/to/cloned/gazebo_plugin_tutorial/build
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/path/to/cloned/learn/gazebo_plugin_tutorial/models
~~~

# ルンバ　＋　RPLIDAR　で地図を作成

・キーボード制御版
　　ルンバを　キーボード　で制御し、地図を作成

・オート版
　　ルンバが自動で障害物を避けながら移動し、地図を作成


# ルンバ　と　RaspberryPi　の接続

  [参考](https://goodfield55.blog.fc2.com/blog-entry-15.html)


# 操作方法

１．キーボード制御版

　１－１．ルンバ(RaspberryPi)

     roslaunch manual_roomba_remote_robot.launch

　１－２．PC

     端末１　roslaunch roomba_remote_desktop.launch

     端末２　rviz rviz

　１－３．キーボード制御方法

　　　'k' + 'return' :  前進

　　　'j' + 'return' :  後進

　　　'l' + 'rteurn' :  時計回り

　　　'h' + 'return' :  反時計回り

　　　'return'       :  停止


２．オート版

　２－１．ルンバ(RaspberryPi)

     roslaunch roomba_remote_robot.launch

　２－２．PC

     端末１　roslaunch roomba_remote_desktop.launch

     端末２　rviz rviz




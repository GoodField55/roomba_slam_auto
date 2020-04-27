# roomba_slam_auto

ルンバ　＋　RPLIDAR　で地図を作成

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



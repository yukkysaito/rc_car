# udev rules
98-serial.rules copy to `/etc/udev/rules.d/`. usb plug out and in.  

# vsec install
```
sudo apt-get install qtcreator qt-sdk libudev-dev libqt5serialport5-dev
cd bldc-tool
qmake -qt=qt5
make clean && make
sudo apt-get install ros-kinetic-ackermann-msgs ros-kinetic-serial
```

# imu
```
sudo apt-get install python-wxgtk3.0
sudo apt-get install ros-kinetic-razor-imu-9dof
roscd razor_imu_9dof/config
cp razor.yaml my_razor.yaml
```
in my_razor.yaml
 port: /dev/ttyUSB0 -> port: /dev/IMU

# roslaunch
```
roslaunch demo_launch demo.launch
```
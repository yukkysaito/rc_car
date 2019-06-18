# vsec install
sudo apt-get install qtcreator qt-sdk libudev-dev libqt5serialport5-dev
cd bldc-tool
qmake -qt=qt5
make clean && make
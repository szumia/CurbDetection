#!/usr/bin/env bash

COMPARE_STRING=""
OPENCV_VERSION="4.6.0"
A=`whoami`

if [[ $A != 'root' ]]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi

echo "script starting!!!!"

#Install GL/gl.h
apt-get install mesa-common-dev
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev

# Install Basic Package
apt update && apt install -y cmake g++ wget unzip git vim
apt install pkg-config libgtk2.0-dev libgl1-mesa-dev libavformat-dev libavcodec-dev libswscale-dev


#Install FFmpeg
apt-get install ffmpeg

str_exist() {
	if [[ -z "$COMPARE_STRING" ]]; then
	  echo 0
	elif [[ -n "$COMPARE_STRING" ]]; then
	  echo 1
	fi	
}

install_opencv() {
	mkdir opencv_install && cd opencv_install
	wget -O opencv.zip https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip
	unzip opencv.zip
	unzip opencv_contrib.zip
	mkdir -p build && cd build
	cmake -DWITH_FFMPEG=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-$OPENCV_VERSION/modules ../opencv-$OPENCV_VERSION
	cmake --build .
	make install
	cd ../../
	rm -rf opencv-install
}



install_fmt() {
	mkdir fmt_install && cd fmt_install
	wget https://github.com/fmtlib/fmt/archive/refs/tags/8.1.1.zip
	unzip 8.1.1.zip
	cd fmt-8.1.1/
	mkdir build && cd build
	cmake ..
	make -j8
	make install
	cd ../../..
  rm -rf fmt_install
}


install_glog() {
	mkdir glog_install && cd glog_install
	wget https://github.com/google/glog/archive/refs/tags/v0.6.0.zip
	unzip v0.6.0.zip
	cd glog-0.6.0/
	mkdir build && cd build
	cmake ..
	make -j8
	make install
	cd ../../..
  rm -rf glog_install
}


COMPARE_STRING=`ls /usr/local/lib/ | grep libopencv` 
EXIST=$(str_exist)
if [[ $EXIST == "0" ]]; then
  install_opencv
fi

COMPARE_STRING=`ls /usr/local/lib/ | grep libfmt` 
EXIST=$(str_exist)
if [[ $EXIST == "0" ]]; then
  install_fmt
fi

COMPARE_STRING=`ls /usr/local/lib/ | grep libglog` 
EXIST=$(str_exist)
if [[ $EXIST == "0" ]]; then
  install_glog
fi

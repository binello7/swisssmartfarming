#!/usr/bin/env bash

project_dir=$(pwd)

#-------------------------------------------------------------------------------
echo "Setting up python3 catkin_ws..."
catkin_ws="$HOME/catkin_ws_py3/"
mkdir -p $catkin_ws
cd $catkin_ws && mkdir src

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 \
-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install

cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd $catkin_ws
catkin build cv_bridge
source install/setup.bash --extend

echo -e "\n# Added by $project_dir/install.bash" >> ~/.bashrc
echo -e "source $catkin_ws/install/setup.bash --extend" >> ~/.bashrc

cd $project_dir
#-------------------------------------------------------------------------------

echo "Set up python2 virtual environment..."
virtualenv -p /usr/bin/python2 venv2
source venv2/bin/activate
pip install -r requirements_py2.txt
deactivate
#-------------------------------------------------------------------------------

echo "Set up python3 virtual environment..."
python3 -m venv venv3
source venv3/bin/activate
pip install -r requirements_py3.txt

echo -e "\nAll done!"

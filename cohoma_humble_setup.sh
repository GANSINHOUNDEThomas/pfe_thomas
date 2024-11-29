apt update
apt upgrade
apt install sudo curl git nano
sudo apt install mesa-utils

cd
#git clone https://github.com/PX4/PX4-Autopilot.git --recursive
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.15.0-beta1-703-gda8827883f

cd
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
pip install symforce
make px4_sitl -j$(nproc) --quiet
sudo apt remove gz-harmonic
#sudo apt install aptitude
sudo apt install gazebo libgazebo11 libgazebo-dev
#make px4_sitl gazebo-classic -j$(nproc)

cd 
apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio



cd
git clone https://github.com/ReconDronePilot/gazebo-classic_x500.git 
cd gazebo-classic_x500
chmod +x add_gazebo-classic_x500.sh
./add_gazebo-classic_x500.sh /root

cd
cp x500.sdf /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/x500 



cd
#git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

git clone --recursive --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/


cd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-eigen3-cmake-module


cd
mkdir ws
cd ws/


#git clone https://github.com/GANSINHOUNDEThomas/pfe_thomas/tree/pfe_thomas/px4_msgs.git
#git clone https://github.com/GANSINHOUNDEThomas/pfe_thomas/tree/pfe_thomas/joy_py.git
sudo apt install ros-humble-gazebo-msgs
sudo apt install ros-humble-gazebo-ros-pkgs
#git clone --recursive https://github.com/GANSINHOUNDEThomas/pfe_thomas/tree/pfe_thomas/px4-ros2-interface-lib.git
git clone https://github.com/GANSINHOUNDEThomas/pfe_thomas.git
mv pfe_thomas/ src/

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash


cd
cd PX4-Autopilot/
make px4_sitl gazebo-classic_x500








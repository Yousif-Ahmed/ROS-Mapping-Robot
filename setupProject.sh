prefix="ros-noetic"

# List of packages we need 
packages=("navigation" "gmapping" "robot-localization" "mavros-msgs" "velocity-controllers" "twist-mux" "teleop-twist-keyboard")

# Base install command
apt_command="sudo apt-get install "

# Build a long command that should look like 
# `sudo apt-get install ros-noetic-pkg1 ros-noetic-pkg2 ... ros-noetic-pkgn -y`
for pkg in ${packages[@]}; do
	apt_command+="$prefix-$pkg "
done

apt_command+=" -y"

# Run the command
eval $apt_command

cd ./src
git clone https://github.com/RobotnikAutomation/summit_xl_common.git
git clone https://github.com/RobotnikAutomation/summit_xl_sim.git
git clone https://github.com/RobotnikAutomation/robotnik_msgs.git
git clone https://github.com/RobotnikAutomation/robotnik_sensors.git

cd ..
catkin_make

source ./devel/setup.bash & echo 'sourced ./devel/setup.bash'

echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

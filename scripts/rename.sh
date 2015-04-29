user_name=$USER
host_name=$HOSTNAME

sed -i -e "s/jakob-think/$host_name/" /home/$user_name/workspace/catkin/src/finder/launch/*.launch
sed -i -e "s/jakob/$user_name/" /home/$user_name/workspace/catkin/src/finder/launch/*.launch

sed -i -e "s/jakob-think/$host_name/"  /home/$user_name/workspace/rosbuild/.rosinstall
sed -i -e "s/jakob/$user_name/" /home/$user_name/workspace/rosbuild/.rosinstall

sed -i -e "s/jakob-think/$host_name/"  /home/$user_name/workspace/rosbuild/setup.*
sed -i -e "s/jakob/$user_name/" /home/$user_name/workspace/rosbuild/setup.*
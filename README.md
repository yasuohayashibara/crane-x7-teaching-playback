# crane-x7-teaching-playback

### INSTALL

- Down load source code
```
cd catkin/src
git clone https://github.com/rt-net/crane_x7_ros
git clone https://github.com/rt-net/crane_x7_description
git clone https://github.com/yasuohayashibara/crane-x7-teaching-playback
```

- Install nessesarry softwares
```
rosdep install -r -y --from-paths . --ignore-src
```

- Build
```
catkin build
```

### Execute

- Simulator
```
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

- Real robot
```
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

- Keyboard capture (execute as a super user)
```
roscd crane-x7-teaching-playback
cd scrips
sudo python keyboard2command.py
```
- Execute teaching-playback node
```
rosrun crane-x7-teaching-playback teaching_playback.py
```

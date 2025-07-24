# crane-x7-teaching-playback

This is a node for teaching-playback using crane_x7.

[![IMAGE](http://img.youtube.com/vi/VvXfMvtXD0I/0.jpg)](https://youtu.be/VvXfMvtXD0I)

RT CORPORATION crane_x7  
https://rt-net.jp/products/crane-x7/

6 units used at open campus  
![230805_オープンキャンパス](https://github.com/yasuohayashibara/crane-x7-teaching-playback/assets/5755200/48b2afe1-c144-4d65-8d6c-7631a5c14d02)

### Key assign

2: x+, 8: x-, 6: y+, 4: y-, -: z+, +: z-  
0: hand open, .: hand close  
q: save position + increment index  
w: delete position + decrement index  
r: play positions  

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
sudo chmod 777 /dev/ttyUSB0
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

- Keyboard capture (execute as a super user)
```
roscd crane-x7-teaching-playback
cd scripts
sudo python keyboard2command.py
```
- Execute teaching-playback node
```
rosrun crane-x7-teaching-playback teaching_playback.py
```

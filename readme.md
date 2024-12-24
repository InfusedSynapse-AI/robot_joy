# robot_joy功能包使用说明

### 适用系统（ubuntu）——ros kinetic、ros melodic、ros noetic

### 编译安装

#### 依赖安装（除ros官方常用的功能包外）

```shell
$ sudo apt install ros-${ROS_DISTRO}-joy
```

dual_arm_msgs: 待上传

#### 下载

使用git clone将该功能包下载到工作空间中

#### 编译

```shell
$ catkin_make
```

### 运行

```shell
$ roslaunch robot_joy robot_joy.launch
```

### 手柄控制（Xbox360键位）
 
<u>**由于一些手柄键位不同，所以直接给出/joy中对应的原始数据位，括号中为我使用的手柄键位作为参考**</u>

模式切换：axes[6] (十字键左右)，顺序依次（移动，右臂关节运动，右臂末端平移，右臂末端旋转，左臂关节运动，左臂末端平移，左臂末端旋转）

唤醒(速度控制时需要唤醒，无操作30s后会自动休眠)：buttons[9] = 1（LS摇杆按压一下）

前进（机械臂关节2正转、末端x正方向平移、末端rx正方向旋转）：axes[1] = 1（LS摇杆上推）

后退（机械臂关节2反转、末端rx正方向旋转、末端rx负方向旋转）：axes[1] = -1（LS摇杆下推）

左移（机械臂关节1正转、末端y正方向平移、末端ry正方向旋转）：axes[0] = 1（LS摇杆左推）

右移（机械臂关节1反转、末端y负方向平移、末端ry负方向旋转）：axes[0] = -1（LS摇杆右推）

左转（机械臂关节3正转）：axes[3] = 1（RS摇杆左推）

右转（机械臂关节3反转）：axes[3] = -1（RS摇杆右推）

机械臂关节4正转、末端z正方向平移、末端rz正方向旋转：axes[4] = 1（RS摇杆上推）

机械臂关节4反转、末端z负方向平移、末端rz负方向旋转：axes[4] = -1（RS摇杆下推）

减速(机械臂关节5正转)：axes[2] = -1（LT键按下）

加速(机械臂关节5反转)：axes[5] = -1（RT键按下）

机械臂关节5正转：botton[4] = 1（LB键按下）

机械臂关节5反转：botton[5] = -1（RB键按下）

机械臂关节6正转：buttons[2] （X键按下）

机械臂关节6反转：buttons[1] （B键按下）


### 话题（topic）

#### 订阅

/joy（sensor_msgs/Joy）: 手柄控制话题

#### 发布

/joy_vel(geometry_msgs/Twist)：速度数据
/r_arm/rm_driver/Arm_PosTeach(dual_arm_msgs/Pos_Teach)：右臂末端位置示教
/r_arm/rm_driver/Arm_OrtTeach(dual_arm_msgs/Ort_Teach)：右臂末端姿态示教
/r_arm/rm_driver/Arm_JointTeach(dual_arm_msgs/Joint_Teach)：右臂关节位置示教
/r_arm/rm_driver/Arm_StopTeach(dual_arm_msgs/Stop_Teach): 右臂示教停止

/l_arm/rm_driver/Arm_PosTeach(dual_arm_msgs/Pos_Teach)：左臂末端位置示教
/l_arm/rm_driver/Arm_OrtTeach(dual_arm_msgs/Ort_Teach)：左臂末端姿态示教
/l_arm/rm_driver/Arm_JointTeach(dual_arm_msgs/Joint_Teach)：左臂关节位置示教
/l_arm/rm_driver/Arm_StopTeach(dual_arm_msgs/Stop_Teach): 左臂示教停止



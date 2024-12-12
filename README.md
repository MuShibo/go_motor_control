### Notice

support motor: GO-M8010-6 motor

ROS version: Noetic

### Build
```bash
catkin build go_motor_control
```

### Run
```bash
roslaunch go_motor_control motor_test.launch
```

### Tip

Typically, for kp and kd, assuming the motor's gear ratio is r, when calculating kp and kd on the rotor side, we need to convert kp and kd on the output side by dividing them by the square of r. 

$$kp_{\text{rotor}} = \frac{kp_{\text{output}}}{r^2}$$

$$kd_{\text{rotor}} = \frac{kd_{\text{output}}}{r^2}$$


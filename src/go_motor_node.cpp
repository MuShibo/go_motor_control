//
// Created by Mu Shibo on 12/2024
//

#include <ros/ros.h>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "go_motor_node");
  ros::NodeHandle nh;

  SerialPort serial("/dev/ttyUSB0");
  MotorCmd cmd;
  MotorData data;

  float sin_counter = 0.0;

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    sin_counter += 0.001;
    float output_angle_d = 90 * sin(2 * 3.14 * sin_counter);
    float rotor_angle_d = (output_angle_d * (3.14 / 180));

    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = 10 * (1 / queryGearRatio(MotorType::GO_M8010_6) / queryGearRatio(MotorType::GO_M8010_6));
    cmd.kd = 1 * (1 / queryGearRatio(MotorType::GO_M8010_6) / queryGearRatio(MotorType::GO_M8010_6));
    cmd.q = rotor_angle_d * queryGearRatio(MotorType::GO_M8010_6);
    cmd.dq = -0 * queryGearRatio(MotorType::GO_M8010_6);
    cmd.tau = 0.0 / queryGearRatio(MotorType::GO_M8010_6);
    serial.sendRecv(&cmd, &data);

    std::cout << std::endl;
    std::cout << "motor.q: " << data.q / queryGearRatio(MotorType::GO_M8010_6) << std::endl;
    std::cout << "motor.W: " << data.dq / queryGearRatio(MotorType::GO_M8010_6) << std::endl;
    std::cout << "motor.tau: " << data.tau * queryGearRatio(MotorType::GO_M8010_6) << std::endl;
    std::cout << std::endl;

    loop_rate.sleep();
  }

  return 0;
}

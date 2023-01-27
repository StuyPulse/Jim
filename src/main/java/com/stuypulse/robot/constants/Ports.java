/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Intake {
        int FRONT_MOTOR_PORT = 30;
        int BACK_MOTOR_PORT = 31;
        int FRONT_RIGHT_SENSOR = 0;
        int FRONT_LEFT_SENSOR = 1;
        int BACK_LEFT_SENSOR = 2;
        int BACK_RIGHT_SENSOR = 3;
    }

    public interface Arm {
        int SHOULDER_LEFT = 20;
        int SHOULDER_RIGHT =  21;
        int WRIST = 22;
    }
}

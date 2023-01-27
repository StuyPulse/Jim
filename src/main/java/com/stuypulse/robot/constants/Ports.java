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

    public interface LEDController {
        int PORT = 694; // PWM
    }
    
    public interface Plant {
        int FORWARD = 0;
        int REVERSE = 1;
    }
    
    public interface Wings {
        int LEFT_DEPLOY_FORWARD = 2;
        int LEFT_DEPLOY_REVERSE = 3;
        int RIGHT_DEPLOY_FORWARD = 4;
        int RIGHT_DEPLOY_REVERSE = 5;
        int LEFT_LATCH = 7;
        int RIGHT_LATCH = 8;
    }
}

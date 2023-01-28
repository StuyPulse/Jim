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
    
    public interface Swerve {
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 11;
        }

        public interface FrontLeft {
            int DRIVE = 12;
            int TURN = 13;
        }

        public interface BackRight{
            int DRIVE = 14;
            int TURN = 15;
        }

        public interface BackLeft{
            int DRIVE = 16;
            int TURN = 17; 
        }
    }

    public interface Arm {
        int SHOULDER_LEFT = 20;
        int SHOULDER_RIGHT =  21;
        int WRIST = 22;
    }

    public interface LEDController {
        int PORT = 0; // PWM
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
        int LEFT_LATCH = 6;
        int RIGHT_LATCH = 7;
    }
}

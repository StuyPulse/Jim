/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.05;

    //shut up amber
    public interface Intake{
        SmartNumber STALL_TIME = new SmartNumber("Settings/Intake/Stall Time", 0.2);
        SmartNumber STALL_CURRENT = new SmartNumber("Settings/Intake/Stall Current", 20);

        SmartNumber CONE_FRONT_ROLLER = new SmartNumber("Settings/Intake/Cone Front Roller Speed", 0.3);
        SmartNumber CONE_BACK_ROLLER = new SmartNumber("Settings/Intake/Cone Front Roller Speed", 0);

        SmartNumber CUBE_FRONT_ROLLER = new SmartNumber("Settings/Intake/Cube Front Roller Speed", 0.3);
        SmartNumber CUBE_BACK_ROLLER = new SmartNumber("Settings/Intake/Cube Back Roller Speed", 0.2);

    }

    public interface Arm {
        public interface ShoulderFeedback {
            double P = 1;
            double I = 0;
            double D = 0;
        }

        public interface WristFeedback {
            double P = 1;
            double I = 0;
            double D = 0;
        }

        double SHOULDER_GEAR_RATIO = 1;
        double WRIST_GEAR_RATIO = 1;
        double SHOULDER_CONVERSION = 360 / SHOULDER_GEAR_RATIO;
        double WRIST_CONVERSION = 360 / WRIST_GEAR_RATIO;
    }
}

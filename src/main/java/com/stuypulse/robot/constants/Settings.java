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

    public interface Intake{
        double debounceTime = 0.3;
        double STALLING_THRESHOLD = 0.2;

        SmartNumber frontMotorSpeed = new SmartNumber("Settings/Intake/frontMotorSpeed", 0.3);
        SmartNumber coneBackMotorSpeed = new SmartNumber("Settings/Intake/coneBackMotorSpeed", 0.3);
        SmartNumber cubeBackMotorSpeed = new SmartNumber("Settings/Intake/cubeBackMotorSpeed", 0.2);

    }
}

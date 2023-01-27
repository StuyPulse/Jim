/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface LED {
        double MANUAL_UPDATE_TIME = 0.75;
        double BLINK_TIME = 0.5;
    }
        
    public interface Wings {
        SmartNumber LEFT_LATCH_DELAY = new SmartNumber("Wings/Left Latch Delay", 1.0);
        SmartNumber RIGHT_LATCH_DELAY = new SmartNumber("Wings/Right Latch Delay", 1.0);
        SmartNumber LEFT_RETRACT_DELAY = new SmartNumber("Wings/Left Retract Delay", 1.0);
        SmartNumber RIGHT_RETRACT_DELAY = new SmartNumber("Wings/Right Retract Delay", 1.0);
    }
}

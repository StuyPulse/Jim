/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.02;

    public interface Arm {
    
        public interface Shoulder {
            double GEARING = 80;
            double LENGTH = Units.inchesToMeters(42);
            double MAX_ANGLE = 180; 
            double MIN_ANGLE = -180;
            double MASS = 0.01; 
            double WEIGHT = MASS * 9.81; 
            double JKG = 0.33 * MASS * (Math.pow(LENGTH, 2));

            double VEL_LIMIT = 0.5;
            double ACCEL_LIMIT = 0.4;

            double ANGLE_OFFSET = 0;

            SmartBoolean DEADZONE_ENABLED = new SmartBoolean("Arm/Deadzone Enabled", true);
            double ANGLE_DEADZONE = 36;
            double ANGLE_DEADZONE_HIGH = 90 + ANGLE_DEADZONE;
            double ANGLE_DEADZONE_LOW = 90 - ANGLE_DEADZONE;
    
            public interface PID {
                SmartNumber kP = new SmartNumber("Shoulder/kP", 16);
                SmartNumber kI = new SmartNumber ("Shoulder/kI", 0);
                SmartNumber kD = new SmartNumber("Shoulder/kD", 0);
            }
            
            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Shoulder/kS", 0.1);
                SmartNumber kA = new SmartNumber("Shoulder/kA", 0.06);
                SmartNumber kG = new SmartNumber("Shoulder/kG", 0.24);
                SmartNumber kV = new SmartNumber("Shoulder/kV", 0.3);
            }
        }
    
        public interface Wrist {
            double GEARING = 50;
            double LENGTH = Units.inchesToMeters(17);
            double MAX_ANGLE = Double.POSITIVE_INFINITY; 
            double MIN_ANGLE = Double.NEGATIVE_INFINITY;
            double MASS = 0.001;
            double WEIGHT = MASS * 9.81;
            double JKG = 0.33 * MASS * (Math.pow(LENGTH, 2));
            
            double VEL_LIMIT = 1.0;
            double ACCEL_LIMIT = 0.8;

            double ANGLE_OFFSET = 0;
    
            public interface PID {
                SmartNumber kP = new SmartNumber("Wrist/kP", 8);
                SmartNumber kI = new SmartNumber ("Wrist/kI", 0);
                SmartNumber kD = new SmartNumber("Wrist/kD", 0);
            }
    
            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Wrist/kS", 0.1);
                SmartNumber kA = new SmartNumber("Wrist/kA", 0.05);
                SmartNumber kG = new SmartNumber("Shoulder/kG", 0.0);
                SmartNumber kV = new SmartNumber("Shoulder/kV", 0.1);
            }
        }
    }

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

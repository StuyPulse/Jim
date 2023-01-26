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
    public interface Arm {
        public interface Simulation {
            double ROBOT_HEIGHT = 0;
            double ROBOT_WIDTH = 0;
            double MID_PEG_HEIGHT = 0;
            double TOP_PEG_HEIGHT = 0;

            public interface Shoulder {
                double GEARING = 80;
                double LENGTH = 42; // inches
                
                double MAXANGLE = Units.degreesToRadians(30); 
                double MINANGLE = Units.degreesToRadians(-210);
                double MASS = 7; 
                double WEIGHT = MASS * 9.81; 
                double JKG = 0.33 * MASS * (Math.pow(Units.inchesToMeters(LENGTH), 2));
    
                double VEL_LIMIT = 30;
                double ACCEL_LIMIT = 10;
    
                double MIDGOAL = 0.5; // meters
                double HIGHGOAL = 0.5; // meters
    
                public interface PID {
                    SmartNumber kP = new SmartNumber("Shoulder/kP", 1);
                    SmartNumber kI = new SmartNumber ("Shoulder/kI", 0);
                    SmartNumber kD = new SmartNumber("Shoulder/kD", 0.1);
                }
            
    
                public interface Feedforward {
                    double kS = 0.1;
                    double kA = 0.06;
                    double kG = 3.382;
                    double kV = 0.3;
                }
            }
    
            public interface Wrist {
                
                double GEARING = 80;
                double LENGTH = 17; // inches
                double MAXANGLE = 90; 
                double MINANGLE = -90;
                double MASS = 4;
                double WEIGHT = MASS * 9.81;
                double JKG = 0.33 * MASS * (Math.pow(Units.inchesToMeters(LENGTH), 2));
    
                double VEL_LIMIT = 4;
                double ACCEL_LIMIT = 2;
    
                double MIDGOAL = 0.5; // meters
                double HIGHGOAL = 0.5; // meters
            
                public interface PID {
                    SmartNumber kP = new SmartNumber("Wrist/kP", 0.6);
                    SmartNumber kI = new SmartNumber ("Wrist/kI", 0);
                    SmartNumber kD = new SmartNumber("Wrist/kD", 0);
                }
    
                public interface Feedforward {
                    double kS = 0.1;
                    double kA = 0.05;
                    double kG = 0.7;
                    double kV = 0.1;
                }
            }
        }
        public interface ShoulderFeedForward {
            double kG = -1;
            double kS = 1;
            double kA = 1;
            double kV = 1;
        }

        public interface WristFeedForward {
            double kG = -1;
            double kS = 1;
            double kA = 1;
            double kV = 1;
        }
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

        double SHOULDER_VEL_LIMIT = 1;
        double SHOULDER_ACC_LIMIT = 1;
        double WRIST_VEL_LIMIT = 1;
        double WRIST_ACC_LIMIT = 1;

        double SHOULDER_MAX_VOLTAGE = 3;
        double WRIST_MAX_VOLTAGE = 3;
        double SHOULDER_GEAR_RATIO = 1;
        double WRIST_GEAR_RATIO = 1;
        double SHOULDER_CONVERSION = 360 / SHOULDER_GEAR_RATIO;
        double WRIST_CONVERSION = 360 / WRIST_GEAR_RATIO;
    }
}

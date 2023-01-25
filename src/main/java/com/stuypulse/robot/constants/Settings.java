/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Vision {
        double TOLERANCE = -1;
        Pose2d[] TAGS = {
            
        };
    }

    
    
    double DT = 0.02;

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(30.0);
        double HEIGHT = Units.inchesToMeters(24.0);
        double MAX_SPEED = 4.2;

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0);
        }
        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0); 

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 0);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0);               
        }

        public interface FrontRight {
            String ID = "Front Right";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, HEIGHT * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, HEIGHT * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, HEIGHT * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, HEIGHT * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(3);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 4.71;
                
                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double GEAR_RATIO = -1;
                double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        } 
    } 
}

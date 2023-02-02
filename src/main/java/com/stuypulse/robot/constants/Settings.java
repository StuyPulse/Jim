/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double DT = 0.02;

    public interface Intake{
        SmartNumber STALL_TIME = new SmartNumber("Settings/Intake/Stall Time", 0.2);
        SmartNumber STALL_CURRENT = new SmartNumber("Settings/Intake/Stall Current", 20);

        SmartNumber CONE_FRONT_ROLLER = new SmartNumber("Settings/Intake/Cone Front Roller Speed", 1);
        SmartNumber CONE_BACK_ROLLER = new SmartNumber("Settings/Intake/Cone Back Roller Speed", 1);

        SmartNumber CUBE_FRONT_ROLLER = new SmartNumber("Settings/Intake/Cube Front Roller Speed", 1);
        SmartNumber CUBE_BACK_ROLLER = new SmartNumber("Settings/Intake/Cube Back Roller Speed", 0.8);

    }

    public interface Vision {
        double TOLERANCE = -1;
        double COMMUNITY_DISTANCE = Field.PEG_TO_CHARGING_STATION_EDGE - Swerve.LENGTH/2;

        public interface Limelight {
            String [] LIMELIGHTS = {"limelight-front","limelight-back"};
            int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
            double CAMERA_TO_CENTER = Units.inchesToMeters(-1);
            Angle CAMERA_PITCH = Angle.fromDegrees(-1);
            double CAMERA_HEIGHT = Units.inchesToMeters(-1);
        }
    }

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(30.0);
        double LENGTH = Units.inchesToMeters(24.0);
        double MAX_SPEED = 4.2;

        public interface Turn {
            double kP = 0;
            double kI = 0;
            double kD = 0;

            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 1);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 1);
        }
        public interface Drive {
            double kP = 0;
            double kI = 0;
            double kD = 0; 

            double kS = 0;
            double kV = 1;
            double kA = 1;
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
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
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        } 
    } 

    public interface Arm {
    
        public interface Shoulder {
            double GEARING = 80;
            double LENGTH = Units.inchesToMeters(43.75);
            double MAX_ANGLE = 0; 
            double MIN_ANGLE = -180;
            double MASS = 0.01; 
            double WEIGHT = MASS * 9.81; 
            double JKG = 0.33 * MASS * (Math.pow(LENGTH, 2));

            double VEL_LIMIT = 0.5;
            double ACCEL_LIMIT = 0.4;

            double ANGLE_OFFSET = 0;

            SmartBoolean DEADZONE_ENABLED = new SmartBoolean("Arm/Deadzone Enabled", true);
            double ANGLE_DEADZONE = 30;
            double ANGLE_DEADZONE_HIGH = 90 + ANGLE_DEADZONE;
            double ANGLE_DEADZONE_LOW = 90 - ANGLE_DEADZONE;

            double TOLERANCE = 3;
    
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

            double TOLERANCE = 5;
    
            public interface PID {
                SmartNumber kP = new SmartNumber("Wrist/kP", 10);
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

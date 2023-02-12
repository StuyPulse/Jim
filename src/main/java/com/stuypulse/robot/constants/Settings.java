/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.stuylib.math.Angle;

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
    
    public enum Robot {
        JIM,
        SACROD
    }

    Robot ROBOT = Robot.JIM;

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
        double USABLE_DISTANCE = Units.feetToMeters(10);
        double TRUST_DISTANCE = Units.feetToMeters(5);
        double TRUST_ANGLE = 50;

        public interface Limelight {
            String [] LIMELIGHTS = {"limelight"};
            int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
        }
    }

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(26.504);
        double LENGTH = Units.inchesToMeters(20.508);
        
        double MAX_SPEED = 4.2;
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity (rad/s)", 3.0);


        public interface Motion {
            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
        
        public interface Turn {
            double kP = 3.5;
            double kI = 0.0;
            double kD = 0.1;
            
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.007);
        }

        public interface Drive {
            double kP = 1.3;
            double kI = 0.0;
            double kD = 0.0; 

            double kS = 0.17335;
            double kV = 2.7274;
            double kA = 0.456;
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

                double MIN_PID_INPUT = 0;
                double MAX_PID_INPUT = POSITION_CONVERSION;
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
                SmartNumber kP = new SmartNumber("Arm/Shoulder/kP", 16);
                SmartNumber kI = new SmartNumber ("Arm/Shoulder/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Shoulder/kD", 0);
            }
            
            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Shoulder/kS", 0.1);
                SmartNumber kA = new SmartNumber("Arm/Shoulder/kA", 0.06);
                SmartNumber kG = new SmartNumber("Arm/Shoulder/kG", 0.24);
                SmartNumber kV = new SmartNumber("Arm/Shoulder/kV", 0.3);
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
            SmartNumber wristSpeedDegrees = new SmartNumber("Arm/Wrist/Speed", 15); // degrees per second
            
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
                SmartNumber kS = new SmartNumber("Arm/Wrist/kS", 0.1);
                SmartNumber kA = new SmartNumber("Arm/Wrist/kA", 0.05);
                SmartNumber kG = new SmartNumber("Arm/Wrist/kG", 0.0);
                SmartNumber kV = new SmartNumber("Arm/Wrist/kV", 0.1);
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

    public interface Operator {
        SmartNumber DEADBAND = new SmartNumber("Operator Settings/Deadband", 0.05);

        SmartNumber WRIST_TELEOP_SPEED = new SmartNumber("Operator Settings/Wrist Adjust Speed", 25); // deg per second
        SmartNumber WRIST_FILTERING = new SmartNumber("Operator Settings/Wrist Filtering", 0.1);
        
        SmartNumber SHOULDER_TELEOP_SPEED = new SmartNumber("Operator Settings/Shoulder Adjust Speed", 25); // deg per second
        SmartNumber SHOULDER_FILTERING = new SmartNumber("Operator Settings/Shoulder Filtering", 0.1);
    }

    public interface Driver {
        
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.45);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.00);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.00);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 2.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("DriPver Settings/Speed Filtering", 0.125);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.005);
    
        
        SmartNumber DEADBAND = new SmartNumber("Driver Settings/Deadband", 0.05);
        SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Max Speed", 4.2);
        SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Max Accleration", 7);
        SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Max Turning", 6.1);

        SmartNumber MAX_SLOW_SPEED = new SmartNumber("Driver Settings/Max Slow Speed", Units.feetToMeters(1));
        SmartNumber MAX_SLOW_TURNING = new SmartNumber("Driver Setings/Max Slow Turning", Units.degreesToRadians(10));

        public interface Drive {
            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.25);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);
        }

        public interface Turn {
            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.15);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);
        }

    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }


    public interface Alignment {

        SmartNumber DEBOUNCE_TIME = new SmartNumber("Alignment/Debounce Time", 0.3);

        SmartNumber ALIGNED_THRESHOLD_X = new SmartNumber("Alignment/X Threshold", 0.1);
        SmartNumber ALIGNED_THRESHOLD_Y = new SmartNumber("Alignment/Y Threshold", 0.1);
        SmartNumber ALIGNED_THRESHOLD_ANGLE = new SmartNumber("Alignment/Angle Threshold", 5);
        
        public interface Translation {
            SmartNumber P = new SmartNumber("Alignment/Translation/kP", 2);
            SmartNumber I = new SmartNumber("Alignment/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Translation/kD", 0.0);
        }
        public interface Rotation {
            SmartNumber P = new SmartNumber("Alignment/Rotation/kP", 2);
            SmartNumber I = new SmartNumber("Alignment/Rotation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Rotation/kD", 0);
        }
    }
    
}

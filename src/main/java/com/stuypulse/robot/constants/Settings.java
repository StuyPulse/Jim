/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.robot.util.ArmJoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    
    public enum Robot {
        JIM,
        SACROD,


        // runs voltage control project
        BLAY_MODE
    }

    Robot ROBOT = Robot.JIM;

    double DT = 0.02;

    // SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", false);
    boolean DEBUG_MODE = true;

    public static boolean isDebug() {
        return DEBUG_MODE; // DEBUG_MODE.get();// || RobotBase.isSimulation();
    }

    public static void putNumber(String key, double value) {
        if (isDebug())
            SmartDashboard.putNumber(key, value);
    }

    public static void putBoolean(String key, boolean value) {
        if (isDebug())
            SmartDashboard.putBoolean(key, value);
    }

    public static void putString(String key, String value) {
        if (isDebug())
            SmartDashboard.putString(key, value);
    }

    public interface Intake{
        SmartNumber STALL_TIME = new SmartNumber("Intake/Stall Time (Rising)", 0.05);
        SmartNumber STALL_CURRENT = new SmartNumber("Intake/Stall Current", 60);

        SmartNumber CUBE_ACQUIRE_TIME = new SmartNumber("Intake/Cube Acquire Time", 0.5);

        SmartNumber INTAKE_CONE_ROLLER_FRONT = new SmartNumber("Intake/Intake Cone Roller Front Speed", 1);
        SmartNumber INTAKE_CONE_ROLLER_BACK = new SmartNumber("Intake/Intake Cone Roller Back Speed", 1);

        SmartNumber INTAKE_CUBE_ROLLER_FRONT = new SmartNumber("Intake/Intake Cube Roller Front Speed", 0.5);
        SmartNumber INTAKE_CUBE_ROLLER_BACK = new SmartNumber("Intake/Intake Cube Roller Back Speed", 0.5);

        SmartNumber OUTTAKE_CONE_ROLLER_FRONT = new SmartNumber("Intake/Outtake Cone Roller Front Speed", 0.5);
        SmartNumber OUTTAKE_CONE_ROLLER_BACK = new SmartNumber("Intake/Outtake Cone Roller Back Speed", 0.3);

        SmartNumber OUTTAKE_CUBE_ROLLER_FRONT = new SmartNumber("Intake/Outtake Cube Roller Front Speed", 1);
        SmartNumber OUTTAKE_CUBE_ROLLER_BACK = new SmartNumber("Intake/Outtake Cube Roller Back Speed", 1);


        SmartNumber NEW_GAMEPIECE_TIME = new SmartNumber("Intake/New Gamepiece Time (Falling)", 0.5);

        SmartNumber IR_SENSOR_TIME = new SmartNumber("Intake/IR Sensor Debounce Time (Rising)", 0.1);
    }

    public interface Vision {
        double USABLE_DISTANCE = Units.feetToMeters(10);
        double TRUST_DISTANCE = Units.feetToMeters(5);
        double TRUST_ANGLE = 50;

        public interface Limelight {
            String [] LIMELIGHTS = {"limelight-front", "limelight-back"};
            int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
        }
    }

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(26.504);
        double LENGTH = Units.inchesToMeters(20.508);
        
        double MAX_SPEED = Units.feetToMeters(15.76);
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity (rad/s)", 6.28);


        public interface Motion {
            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
        
        public interface Turn {
            double kP = 2.0;
            double kI = 0.0;
            double kD = 0.1;
            
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.007);
        }

        public interface Drive {
            double kP = 2.38;
            double kI = 0.0;
            double kD = 0.0; 

            double kS = 0.17459;
            double kV = 2.4561;
            double kA = 0.40442;
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(174).plus(Rotation2d.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-131).plus(Rotation2d.fromDegrees(270));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(119).plus(Rotation2d.fromDegrees(180));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-2).plus(Rotation2d.fromDegrees(90));
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

        public SmartNumber BFS_FIELD_LEAD = new SmartNumber("Arm/Field Lead", 30 );

        public interface Shoulder {
            int MOTORS = 2;
            double REDUCTION = 63.0;
            double MASS = 3.054; // kg
            double LENGTH = 1.1557; // m, length
            double MOI = 0.369; // kg m^2
            double RADIUS = 0.305; // m, radius to cg

            ArmJoint JOINT = 
                new ArmJoint(
                    DCMotor.getNEO(MOTORS).withReduction(REDUCTION),
                    MASS, 
                    LENGTH, 
                    MOI, 
                    RADIUS);

            Rotation2d ZERO_ANGLE = Rotation2d.fromDegrees(129.6 + 90);

            SmartNumber MAX_VELOCITY = new SmartNumber("Arm/Shoulder/Max Velocity (deg)", 120.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Arm/Shoulder/Max Acceleration (deg)", 720.0);

            SmartNumber TOLERANCE = new SmartNumber("Arm/Shoulder/Tolerance (deg)", 10.0);
    
            public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Shoulder/kP", 5.0);
                SmartNumber kI = new SmartNumber("Arm/Shoulder/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Shoulder/kD", 0.6);
            }
            
            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Shoulder/kS", 0.0);
                SmartNumber kA = new SmartNumber("Arm/Shoulder/kA", 0.07);
                // empty kG - 0.275
                // cone  kG - 0.35
                SmartNumber kG = new SmartNumber("Arm/Shoulder/kG", 0.7);
                SmartNumber kV = new SmartNumber("Arm/Shoulder/kV", 0.28);
            }
        }
    
        public interface Wrist {
            
            int MOTORS = 1;
            double REDUCTION = 70.0;
            double MASS = 1.317; // kg
            double LENGTH = 0.44298; // m, length
            double MOI = 0.033; // kg m^2
            double RADIUS = 0.2443 + 1.065; // m, radius to cg

            ArmJoint JOINT = 
                new ArmJoint(
                    DCMotor.getNEO(MOTORS).withReduction(REDUCTION),
                    MASS, 
                    LENGTH, 
                    MOI, 
                    RADIUS);

            Rotation2d ZERO_ANGLE = Rotation2d.fromDegrees(8);

            SmartNumber MAX_VELOCITY = new SmartNumber("Arm/Wrist/Max Velocity (deg)", 240.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Arm/Wrist/Max Acceleration (deg)", 1440.0);

            SmartNumber TOLERANCE = new SmartNumber("Arm/Wrist/Tolerance (deg)", 6.0);
    
            public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Wrist/kP", 5.0);
                SmartNumber kI = new SmartNumber("Arm/Wrist/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Wrist/kD", 0.7);
            }
    
            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Wrist/kS", 0);
                SmartNumber kA = new SmartNumber("Arm/Wrist/kA", 0.06);
                SmartNumber kG = new SmartNumber("Arm/Wrist/kG", 0.6);
                SmartNumber kV = new SmartNumber("Arm/Wrist/kV", 0.24);
            }
        }
    }
    
    public interface LED {
        double MANUAL_UPDATE_TIME = 0.75;
        double BLINK_TIME = 0.5;
    }
    public interface Wings {
        SmartNumber LATCH_DELAY = new SmartNumber("Wings/Red Latch Delay", 0.5);
        SmartNumber RETRACT_DELAY = new SmartNumber("Wings/Red Retract Delay", 0.5);
    }

    public interface AutoBalance {
        SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Distance Threshold", 0.05);
        SmartNumber ANGLE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Angle Thrshold", 6);

        SmartNumber MAX_TILT = new SmartNumber("Auto Balance/Max Tilt (deg)", 15.0); 
        SmartNumber MAX_SPEED = new SmartNumber("Auto Balance/Max Engage Speed (m per s)", 0.5);

        SmartNumber kT_u = new SmartNumber("Auto Balance/With Plant/Tu", 0.2);  // from Zieger-Nichols tuning method

        public interface Translation {
            SmartNumber P = new SmartNumber("Auto Balance/Translation/kP", 0.05);
            SmartNumber I = new SmartNumber("Auto Balance/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Auto Balance/Translation/kD", 0);
        }
    
        public interface Tilt {
            SmartNumber P = new SmartNumber("Auto Balance/Tilt/kP", 0.05);
            SmartNumber I = new SmartNumber("Auto Balance/Tilt/kI", 0);
            SmartNumber D = new SmartNumber("Auto Balance/Tilt/kD", 0);
        }

        public interface Gyro {
            SmartNumber kT_u = new SmartNumber("Auto Engage/Tu", 0.2);  // from Zieger-Nichols tuning method
            Number kK_u = IStream.create(() -> MAX_SPEED.get() / MAX_TILT.get()).number();  // from Zieger-Nichols tuning method

            Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
            SmartNumber kI = new SmartNumber("", 0);
            Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method
        }
    }

    public interface Operator {
        SmartNumber DEADBAND = new SmartNumber("Operator Settings/Deadband", 0.2);

        SmartNumber WRIST_TELEOP_SPEED = new SmartNumber("Operator Settings/Wrist Adjust Speed", 90); // deg per second
        
        SmartNumber SHOULDER_TELEOP_SPEED = new SmartNumber("Operator Settings/Shoulder Adjust Speed", 60); // deg per second
    }

    public interface Driver {
        SmartNumber PLANT_DEBOUNCE = new SmartNumber("Driver Settings/Plant Drive Rising Debounce", 0.5);

        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.1);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.25);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Units.feetToMeters(15.67));
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 6.5);

            SmartNumber MAX_SLOW_SPEED = new SmartNumber("Driver Settings/Drive/Max Slow Speed", Units.feetToMeters(1));
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.08);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.15);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 7.0);

            SmartNumber MAX_SLOW_TURNING = new SmartNumber("Driver Settings/Turn/Max Slow Turning", Units.degreesToRadians(10));
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

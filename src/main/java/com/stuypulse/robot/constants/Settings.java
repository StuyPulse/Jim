/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import com.stuypulse.robot.util.ArmJoint;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.PIDConstants;

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

    public interface Intake{
        SmartNumber STALL_TIME = new SmartNumber("Intake/Stall Time (Rising)", 0.05);
        SmartNumber STALL_CURRENT = new SmartNumber("Intake/Stall Current", 35);

        public interface Acquire {
            SmartNumber CONE_FRONT = new SmartNumber("Intake/Cone Acquire Front", 1.0);
            SmartNumber CONE_BACK = new SmartNumber("Intake/Cone Acquire Back", 1.0);

            SmartNumber CUBE_FRONT = new SmartNumber("Intake/Cube Acquire Front", 0.8);
            SmartNumber CUBE_BACK = new SmartNumber("Intake/Cube Acquire Back", 0.8);
        }

        public interface Deacquire {
            SmartNumber CONE_FRONT = new SmartNumber("Intake/Cone Deacquire Front", 0.5);
            SmartNumber CONE_BACK = new SmartNumber("Intake/Cone Deacquire Back", 0.5);

            SmartNumber CONE_UP_FRONT = new SmartNumber("Intake/Cone Up Deacquire Front", 0.75);
            SmartNumber CONE_UP_BACK = new SmartNumber("Intake/Cone Up Deacquire Back", 0.75);

            SmartNumber CUBE_FRONT = new SmartNumber("Intake/Cube Deacquire Front", 0.5);
            SmartNumber CUBE_BACK = new SmartNumber("Intake/Cube Deacquire Back", 0.5);
        }
    }

    public interface Vision {
        double MAX_USE_DISTANCE = 3;
        double MIN_USE_DISTANCE  = Units.inchesToMeters(5);
        double MIN_USE_ANGLE = 0;
        double MAX_USE_ANGLE = 50;

        public interface Limelight {
            String [] LIMELIGHTS = {
                "limelight-back",
                "limelight-front"
            };
            int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
            Pose3d [] POSITIONS = new Pose3d[] {
                new Pose3d(new Translation3d(0.1, 0, 1.29032), new Rotation3d(0, Math.toRadians(-30), Math.PI)),
                new Pose3d(new Translation3d(0.1, 0, 1.29032), new Rotation3d(0, Math.toRadians(-30), 0))
            };
        }
    }

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(26.504);
        double LENGTH = Units.inchesToMeters(20.508);

        SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Max Module Speed (meter per s)", 5.0);
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity (rad per s)", 6.28);

        SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module Velocity Deadband (m per s)", 0.02);

        public interface Motion {
            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
            double kI = 0.0;
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);

            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.007);
        }

        public interface Drive {
            double kP = 0.8;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.22304;
            double kV = 2.4899;
            double kA = 0.41763;
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(357.213206) // recalibrated 4/21                
                .plus(Rotation2d.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(249.731491) // recalibrated 3/24
                .plus(Rotation2d.fromDegrees(270));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(125.371964) // recalibrated 4/6
                .plus(Rotation2d.fromDegrees(180));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(7.748473) // recalibrated 5/24
                .plus(Rotation2d.fromDegrees(90));
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
            SmartNumber MAX_SHOULDER_ANGLE = new SmartNumber("Arm/Shoulder/Max Angle (deg)", 15.0);
            SmartNumber OVER_BUMPER_ANGLE = new SmartNumber("Arm/Shoulder/Over Bumper Angle (deg)", 25.0);

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

            Rotation2d ZERO_ANGLE = Rotation2d.fromRotations(0.025333).plus(Rotation2d.fromDegrees(+90));

            SmartNumber TELEOP_MAX_VELOCITY = new SmartNumber("Arm/Shoulder/Teleop Max Velocity (deg)", 315);
            SmartNumber TELEOP_MAX_ACCELERATION = new SmartNumber("Arm/Shoulder/Teleop Max Acceleration (deg)", 420);

            SmartNumber AUTON_MAX_VELOCITY = new SmartNumber("Arm/Shoulder/Auton Max Velocity (deg)", 360);
            SmartNumber AUTON_MAX_ACCELERATION = new SmartNumber("Arm/Shoulder/Auton Max Acceleration (deg)", 480);

            SmartNumber STALLING_VOLTAGE = new SmartNumber("Arm/Shoulder/Stalling Voltage", 12.0);
            SmartNumber STALLING_VELOCITY = new SmartNumber("Arm/Shoulder/Stalling Velocity", 0.2);
            SmartNumber STALLING_CURRENT = new SmartNumber("Arm/Shoulder/Stalling Current", 100.0);

            SmartNumber TOLERANCE = new SmartNumber("Arm/Shoulder/Tolerance (deg)", 10.0);

            SmartNumber INTAKE_VOLTAGE = new SmartNumber("Arm/Shoulder/Intake Voltage", 0);

            public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Shoulder/kP", 1.3);
                SmartNumber kI = new SmartNumber("Arm/Shoulder/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Shoulder/kD", 0.25);
            }

            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Shoulder/kS", 0.061);
                SmartNumber kV = new SmartNumber("Arm/Shoulder/kV", 1.2);
                SmartNumber kA = new SmartNumber("Arm/Shoulder/kA", 0.038097);
                SmartNumber kGEmpty = new SmartNumber("Arm/Shoulder/kG Empty", 0.6);

                SmartNumber kGCube = new SmartNumber("Arm/Shoulder/kG Cube", 0.6);
                SmartNumber kGCone = new SmartNumber("Arm/Shoulder/kG Cone", 0.71);
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

            Rotation2d ZERO_ANGLE = Rotation2d.fromRotations(0.662482).plus(Rotation2d.fromDegrees(120));

            SmartNumber TELEOP_MAX_VELOCITY = new SmartNumber("Arm/Wrist/Teleop Max Velocity (deg)", 480.0);
            SmartNumber TELEOP_MAX_ACCELERATION = new SmartNumber("Arm/Wrist/Teleop Max Acceleration (deg)", 480.0);

            SmartNumber AUTON_SHOULDER_VELOCITY_FEEDBACK_CUTOFF = new SmartNumber("Arm/Wrist/Auton Shoulder Velocity Feedback Cutoff (deg per s)", 5.0);
            SmartNumber AUTON_SHOULDER_VELOCITY_FEEDBACK_DEBOUNCE = new SmartNumber("Arm/Wrist/Auton Feedback Enabled Debounce", 0.15);

            SmartNumber TELEOP_SHOULDER_VELOCITY_FEEDBACK_CUTOFF = new SmartNumber("Arm/Wrist/Teleop Shoulder Velocity Feedback Cutoff (deg per s)", 17.5);
            SmartNumber TELEOP_SHOULDER_VELOCITY_FEEDBACK_DEBOUNCE = new SmartNumber("Arm/Wrist/Teleop Feedback Enabled Debounce", 0.0);

            SmartNumber WRIST_SAFE_ANGLE = new SmartNumber("Arm/Wrist/Safe Angle (deg)", 80);

            SmartNumber TOLERANCE = new SmartNumber("Arm/Wrist/Tolerance (deg)", 7.0);

            SmartNumber INTAKE_VOLTAGE = new SmartNumber("Arm/Wrist/Intake Voltage", 0);

            public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Wrist/kP", 6.0);
                SmartNumber kI = new SmartNumber("Arm/Wrist/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Wrist/kD", 1);
            }

            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Wrist/kS", 0.0);
                SmartNumber kA = new SmartNumber("Arm/Wrist/kA", 0.01);
                SmartNumber kG = new SmartNumber("Arm/Wrist/kG", 0.0);
                SmartNumber kV = new SmartNumber("Arm/Wrist/kV", 1.0);
            }
        }
    }

    public interface LED {
        double MANUAL_UPDATE_TIME = 0.75;
        double BLINK_TIME = 0.5;
        int LED_LENGTH = 55;
    }
    public interface Wings {
        SmartNumber LATCH_DELAY = new SmartNumber("Wings/Red Latch Delay", 0.25);
        SmartNumber RETRACT_DELAY = new SmartNumber("Wings/Red Retract Delay", 0.25);
    }

    public interface AutoBalance {
        SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Distance Threshold", 0.05);
        SmartNumber ANGLE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Angle Thrshold", 12);

        SmartNumber MAX_TILT = new SmartNumber("Auto Balance/Max Tilt (deg)", 15.0);
        SmartNumber MAX_SPEED = new SmartNumber("Auto Balance/Max Engage Speed (m per s)", 0.8);

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

        SmartNumber WRIST_TELEOP_SPEED = new SmartNumber("Operator Settings/Wrist Adjust Speed", 120); // deg per second
        SmartNumber SHOULDER_TELEOP_SPEED = new SmartNumber("Operator Settings/Shoulder Adjust Speed", 120); // deg per second

        SmartNumber VOLTAGE_DEADBAND = new SmartNumber("Operator Settings/Voltage Deadband", 0.05);
        SmartNumber SHOULDER_DRIVE_VOLTAGE = new SmartNumber("Operator Settings/Shoulder Drive Voltage", 9.0);
        SmartNumber WRIST_DRIVE_VOLTAGE = new SmartNumber("Operator Settings/Wrist Drive Voltage", 9.0);
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.10);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.125);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 20);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 7.0);

            public interface GyroFeedback {
                SmartBoolean GYRO_FEEDBACK_ENABLED = new SmartBoolean("Driver Settings/Gyro Feedback/Enabled", true);

                SmartNumber P = new SmartNumber("Driver Settings/Gyro Feedback/kP", 0.5);
                SmartNumber I = new SmartNumber("Driver Settings/Gyro Feedback/kI", 0.0);
                SmartNumber D = new SmartNumber("Driver Settings/Gyro Feedback/kD", 0.1);
            }
        }

    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }


    public interface Alignment {

        SmartNumber DEBOUNCE_TIME = new SmartNumber("Alignment/Debounce Time", 0.15);

        SmartNumber ALIGNED_CUBE_THRESHOLD_X = new SmartNumber("Alignment/X Cube Threshold", 0.08);
        SmartNumber ALIGNED_CUBE_THRESHOLD_Y = new SmartNumber("Alignment/Y Cube Threshold", 0.1);
        SmartNumber ALIGNED_CUBE_THRESHOLD_ANGLE = new SmartNumber("Alignment/Angle Cube Threshold", 5);

        SmartNumber ALIGNED_CONE_THRESHOLD_X = new SmartNumber("Alignment/X Cone Threshold", 0.08);
        SmartNumber ALIGNED_CONE_THRESHOLD_Y = new SmartNumber("Alignment/Y Cone Threshold", 0.05);
        SmartNumber ALIGNED_CONE_THRESHOLD_ANGLE = new SmartNumber("Alignment/Angle Cone Threshold", 1);

        SmartNumber AGAINST_GRID_VEL_X = new SmartNumber("Alignment/Against Grid X Velocity (m per s)", 0.02);
        SmartNumber AGAINST_GRID_DEBOUNCE = new SmartNumber("Alignment/Against Grid Debounce", 0.3);

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

    public interface Score {

        // Tip in scoring
        SmartNumber kForwardSpeed = new SmartNumber("Robot Score/Forward Speed (in per s)", 4);
        SmartNumber kWristVoltage = new SmartNumber("Robot Score/Wrist Voltage", 3);

        // Tip in release
        SmartNumber kBackwardsTipInSpeed = new SmartNumber("Robot Score/Tip In Backwards Speed (in per s)", 16);

        // Tip out releasing
        SmartNumber kBackwardsTipOutSpeed = new SmartNumber("Robot Score/Tip Out Backwards Speed (in per s)", 24);

        SmartNumber kShoulderTipOutTolerance = new SmartNumber("Robot Score/Tip Out Shoulder Tolerance (degrees)", 5);
    }
}

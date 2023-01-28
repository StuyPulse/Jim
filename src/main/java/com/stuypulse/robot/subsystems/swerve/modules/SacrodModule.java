package com.stuypulse.robot.subsystems.swerve.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SacrodModule extends ISwerveModule {

    private interface Turn {
        double kP = 3.5;
        double kI = 0.0;
        double kD = 0.1;
    }

    private interface Drive {
        double kP = 1.3;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.17335;
        double kV = 2.7274;
        double kA = 0.456;
    }
    
    public interface Encoder {
        public interface Drive {
            double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            public interface Stages {
                // input / output
                double FIRST = 16.0 / 48.0;
                double SECOND = 28.0 / 16.0;
                double THIRD = 15.0 / 60.0;
            }

            double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public interface Turn {
            double GEAR_RATIO = 1.0 / 12.8;
            double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }

    private interface Chassis {
        double WIDTH = Units.inchesToMeters(29.0);
        double HEIGHT = Units.inchesToMeters(29.0);
    }

    public static SacrodModule createFrontRight() {
        return new SacrodModule(
            "Front Right",
            new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5),
            11, 1, Angle.fromDegrees(143), 10);
    }

    public static SacrodModule createFrontLeft() {
        return new SacrodModule(
            "Front Left",
            new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5),
            13, 3, Angle.fromDegrees(36), 12);
    }

    public static SacrodModule createBackLeft() {
        return new SacrodModule(
            "Back Left",
            new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5),
            15, 2, Angle.fromDegrees(-80.5), 14);
    }

    public static SacrodModule createBackRight() {
        return new SacrodModule(
            "Back Right",
            new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5),
            17, 0, Angle.fromDegrees(142.3), 16);
    }

    // module data

    private final String id;
    private final Translation2d location;

    private SwerveModuleState targetState;

    // turn

    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final Angle angleOffset;

    private final AngleController turnController;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final Controller driveController;

    public SacrodModule(String id, Translation2d location, int turnCANId, int absoluteEncoderChannel,
            Angle angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState();

        // turn

        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        Motors.Swerve.TURN.configure(turnMotor);

        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);
        this.angleOffset = angleOffset;
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        // drive

        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        Motors.Swerve.DRIVE.configure(driveMotor);

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public Translation2d getOffset() {
        return location;
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getRotation2d());
    }

    private double getSpeed() {
        return driveEncoder.getVelocity();
    }

    private double getDistance() {
        return driveEncoder.getPosition();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, absoluteEncoder.getAbsolutePosition()));
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset.getRotation2d());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation2d());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getRotation2d());
    }

    @Override
    public void periodic() {
        turnMotor.setVoltage(turnController.update(
                Angle.fromRotation2d(targetState.angle),
                Angle.fromRotation2d(getRotation2d())));
        driveMotor.setVoltage(driveController.update(targetState.speedMetersPerSecond, getSpeed()));

        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber(id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber(id + "/Absolute Angle", getAbsolutePosition().getDegrees());

        SmartDashboard.putNumber(id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Speed", getSpeed());
        SmartDashboard.putNumber(id + "/Speed Error", driveController.getError());
        SmartDashboard.putNumber(id + "/Speed Voltage", driveController.getOutput());

    }
}

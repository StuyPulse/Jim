/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;



public class MAX_SwerveModule extends SwerveModule {

    // module data
    private String id;
    private Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private CANSparkMax turnMotor;
    private SparkAbsoluteEncoder absoluteEncoder;

    // drive
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    // controller
    private SparkPIDController drivePID;
    private SimpleMotorFeedforward driveFF;
    private SparkPIDController turnPID;


    private final SlewRateLimiter turnRateLimit;

    private double prevVelocity;

    public MAX_SwerveModule(String id, Translation2d location, int turnCANId, Rotation2d angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        // turn
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        configureTurnMotor(angleOffset);

        turnRateLimit = new SlewRateLimiter(Swerve.MAX_TURNING.get());

        // drive
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        configureDriveMotor();

        driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

        targetState = new SwerveModuleState();

        prevVelocity = 0;
    }

    private void configureTurnMotor(Rotation2d angleOffset) {

        turnMotor.restoreFactoryDefaults();

        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        absoluteEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        absoluteEncoder.setZeroOffset(angleOffset.getRotations());
        absoluteEncoder.setInverted(true);

        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(absoluteEncoder);

        turnPID.setP(Turn.kP.get());
        turnPID.setI(Turn.kI);
        turnPID.setD(Turn.kD.get());
        turnPID.setOutputRange(-1, 1);


        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Encoder.Turn.MIN_PID_INPUT);
        turnPID.setPositionPIDWrappingMaxInput(Encoder.Turn.MAX_PID_INPUT);

        turnMotor.enableVoltageCompensation(12.0);

        Motors.Swerve.TURN.configure(turnMotor);
    }

    private void configureDriveMotor() {

        driveMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        drivePID.setP(Drive.kP);
        drivePID.setI(Drive.kI);
        drivePID.setD(Drive.kD);
        drivePID.setOutputRange(-1, 1);

        driveMotor.enableVoltageCompensation(12.0);
        Motors.Swerve.DRIVE.configure(driveMotor);
        driveEncoder.setPosition(0);
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
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition());
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }



    @Override
    public void periodic() {
        // turn
        turnPID.setReference(
            turnRateLimit.calculate(targetState.angle.getRadians()),
            ControlType.kPosition);

        // drive
        double vel = getVelocity();
        double ffVoltage = driveFF.calculate(prevVelocity, vel, Settings.DT);
        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity, 0, ffVoltage, ArbFFUnits.kVoltage);

        prevVelocity = vel;

        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", vel);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveMotor.getOutputCurrent());
    }
}

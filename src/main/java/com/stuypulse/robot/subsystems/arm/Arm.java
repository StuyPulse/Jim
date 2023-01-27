package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
* X absolute encoder: conversion is not 360, this puts the range in 0, 360
* X the range should be -180, +180, we can handle this manually and cleanly through a series of helper functions
* X MathUtil.java is your best friend here
**
* X the angle of the joints are no longer linear, and we have to deal with the jump from 180 to -180 with rotation2d's
* I think. 
* 
* because of that , we need to make sure the arm goes in the exact direction we expect every single time. we also need
* to be careful of physical limits that consider the correct angle (e..g once you go past one full rotation you're at -180 and that's
* probably a valid angle)
* 
* X also you need to consider zeroing the absolute encoder stuff 
* also the starting wrist and shoulder angle must match the angle of the initial target values
*
* also we don't have constants right now that make the simulation work
* X also can you put the sim mechanism2d stuff in its own class so we can use it for the actual robot. also add duplicate
* X ligament so that the real vs target can be logged. 
*//*
* X absolute encoder: conversion is not 360, this puts the range in 0, 360
* X the range should be -180, +180, we can handle this manually and cleanly through a series of helper functions
* X MathUtil.java is your best friend here
**
* X the angle of the joints are no longer linear, and we have to deal with the jump from 180 to -180 with rotation2d's
* I think. 
* 
* because of that , we need to make sure the arm goes in the exact direction we expect every single time. we also need
* to be careful of physical limits that consider the correct angle (e..g once you go past one full rotation you're at -180 and that's
* probably a valid angle)
* 
* X also you need to consider zeroing the absolute encoder stuff 
* also the starting wrist and shoulder angle must match the angle of the initial target values
*
* also we don't have constants right now that make the simulation work
* X also can you put the sim mechanism2d stuff in its own class so we can use it for the actual robot. also add duplicate
* X ligament so that the real vs target can be logged. 
*/
public class Arm extends IArm {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final Controller shoulderController;
    private final Controller wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle; 

    public Arm() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        shoulderEncoder.setZeroOffset(shoulderEncoder.getPosition());
        wristEncoder.setZeroOffset(wristEncoder.getPosition());

        configureMotors();

        shoulderEncoder.setZeroOffset(shoulderEncoder.getPosition());
        wristEncoder.setZeroOffset(wristEncoder.getPosition());

        configureMotors();

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kA, Shoulder.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    .setSetpointFilter(new MotionProfile(SHOULDER_VEL_LIMIT, SHOULDER_ACC_LIMIT));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kA, Wrist.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    .setSetpointFilter(new MotionProfile(WRIST_VEL_LIMIT, WRIST_ACC_LIMIT));

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target Angle", 0);
    }

    private void configureMotors() {
        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(SLMath.map(shoulderEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(SLMath.map(wristEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public double getShoulderTargetAngle() {
        return shoulderTargetAngle.get();
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(SLMath.map(shoulderEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public double getWristTargetAngle() {
        return wristTargetAngle.get();
    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(SLMath.map(wristEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public void setTargetWristAngle(double angle) {
        wristTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Shoulder.MIN_ANGLE), Math.toDegrees(Shoulder.MAX_ANGLE)));
    } 

    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Math.toDegrees(Shoulder.MINANGLE), Math.toDegrees(Shoulder.MAX_ANGLE)));
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle, boolean longPath) {
        wristTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Math.toDegrees(Wrist.MIN_ANGLE), Math.toDegrees(Wrist.MAX_ANGLE)));
    }

    @Override
    public boolean isShoulderAtAngle(double maxError) {
        return Math.abs(getShoulderAngle().getDegrees() - shoulderTargetAngle.get()) < maxError;
    }

    @Override
    public boolean isWristAtAngle(double maxError) {
        return Math.abs(getWristAngle().getDegrees() - wristTargetAngle.get()) < maxError;
    }

    public void moveShoulder(double angle) {
        shoulderTargetAngle.set(shoulderTargetAngle.get() + angle);
    }

    public void moveWrist(double angle) {
        wristTargetAngle.set(wristTargetAngle.get() + angle);
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    public void execute() {
        double shoulderOutput = shoulderController.update(shoulderTargetAngle.get(), getShoulderAngle().getDegrees());
        double wristOutput = wristController.update(wristTargetAngle.get(), getWristAngle().getDegrees());

        runShoulder(shoulderOutput);
        runWrist(wristOutput);

        SmartDashboard.putNumber("Arm/Shoulder/Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle", getWristAngle().getDegrees());
        
        SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
    }
}
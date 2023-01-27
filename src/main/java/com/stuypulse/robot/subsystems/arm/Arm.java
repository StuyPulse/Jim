package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDCalculator;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
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
*/
public class Arm extends IArm {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle; 

    public Arm() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        shoulderEncoder.setZeroOffset(Shoulder.ANGLE_OFFSET);
        wristEncoder.setZeroOffset(Wrist.ANGLE_OFFSET);

        configureMotors();

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    .setSetpointFilter(new AMotionProfile(Shoulder.VEL_LIMIT, Shoulder.ACCEL_LIMIT));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    .setSetpointFilter(new AMotionProfile(Wrist.VEL_LIMIT, Wrist.ACCEL_LIMIT));

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target Angle (deg)", 0);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target Angle (deg)", 0);
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
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Shoulder.MIN_ANGLE, Shoulder.MAX_ANGLE));
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        wristTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Wrist.MIN_ANGLE, Wrist.MAX_ANGLE));
    }

    @Override
    public boolean isShoulderAtAngle(Rotation2d maxError) {
        return Math.abs(getShoulderAngle().minus(Rotation2d.fromDegrees(shoulderTargetAngle.get())).getDegrees()) < maxError.getDegrees();
    }

    @Override
    public boolean isWristAtAngle(Rotation2d maxError) {
        return Math.abs(getWristAngle().minus(Rotation2d.fromDegrees(wristTargetAngle.get())).getDegrees()) < maxError.getDegrees();
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    } 

    @Override
    public void periodic() {

        double shoulderOutput = shoulderController.update(shoulderTargetAngle.get(), getShoulderAngle().getDegrees());
        double wristOutput;

        if (Shoulder.DEADZONE_ENABLED.get() & Math.abs(shoulderTargetAngle.get()) < Shoulder.ANGLE_DEADZONE_HIGH & Math.abs(shoulderTargetAngle.get()) > Shoulder.ANGLE_DEADZONE_LOW) {
            wristOutput = wristController.update(90, getWristAngle().getDegrees());
        } else {
            wristOutput = wristController.update(wristTargetAngle.get(), getWristAngle().getDegrees());
        }

        runShoulder(shoulderOutput);
        runWrist(wristOutput);

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        
        SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
    }
}

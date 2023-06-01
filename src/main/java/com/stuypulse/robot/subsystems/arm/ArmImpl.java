/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.arm;

import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_LEFT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_RIGHT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.WRIST_CONFIG;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_LEFT;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_RIGHT;
import static com.stuypulse.robot.constants.Ports.Arm.WRIST;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.TimedMovingAverage;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmImpl extends Arm {

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final IFilter wristVelocityFilter;
    private final IFilter shoulderVelocityFilter;

    private final BStream wristStalling;

    private double wristVolts;

    protected ArmImpl() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        // Probably helps?
        wristVelocityFilter = new TimedMovingAverage(0.1);
        shoulderVelocityFilter = new TimedMovingAverage(0.1);

        wristStalling = BStream.create(this::isWristStalling)
            .filtered(new BDebounce.Rising(Wrist.STALL_TIME));

        configureMotors();
    }

    private void configureMotors() {
        shoulderEncoder.setZeroOffset(0);
        wristEncoder.setZeroOffset(0);

        shoulderEncoder.setInverted(true);
        shoulderEncoder.setVelocityConversionFactor(Units.rotationsToRadians(1));
        Motors.disableStatusFrames(shoulderRight, 3, 4);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        Motors.disableStatusFrames(shoulderLeft, 3, 4, 5, 6);

        wristEncoder.setInverted(true);
        wristEncoder.setVelocityConversionFactor(Units.rotationsToRadians(1));
        Motors.disableStatusFrames(wrist, 3, 4);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    @Override
    public Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE);
    }

    @Override
    protected void setShoulderVoltageImpl(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    @Override
    protected void setWristVoltageImpl(double voltage) {
        wristVolts = voltage;
        wrist.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean wristCoast, boolean shoulderCoast) {
        shoulderLeft.setIdleMode(shoulderCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        shoulderRight.setIdleMode(shoulderCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        wrist.setIdleMode(wristCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public double getShoulderVelocityRadiansPerSecond() {
        return shoulderVelocityFilter.get(shoulderEncoder.getVelocity());
    }

    @Override
    public double getWristVelocityRadiansPerSecond() {
        return wristVelocityFilter.get(wristEncoder.getVelocity());
    }

    private boolean isWristStalling() {
        return wrist.getOutputCurrent() > Wrist.STALL_CURRENT.get()
            && wristEncoder.getVelocity() < Wrist.STALL_VELOCITY.doubleValue()
            && wristVolts > Wrist.STALL_VOLTAGE.doubleValue();
    }

    @Override
    public void periodicallyCalled() {
        SmartDashboard.putNumber("Arm/Shoulder/Let Bus Voltage (V)", shoulderLeft.getBusVoltage());
        SmartDashboard.putNumber("Arm/Shoulder/Right Bus Voltage (V)", shoulderRight.getBusVoltage());
        SmartDashboard.putNumber("Arm/Wrist/Bus Voltage (V)", wrist.getBusVoltage());

        SmartDashboard.putNumber("Arm/Shoulder/Left Current (amps)", shoulderLeft.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Shoulder/Right Current (amps)", shoulderRight.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Wrist/Current (amps)", wrist.getOutputCurrent());

        if (wristStalling.get()) {
            wrist.setVoltage(0);
            wristVolts = 0;
        }

        SmartDashboard.putNumber("Arm/Shoulder/Raw Encoder Angle (rot)", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist/Raw Encoder Angle (rot)", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder/Left Duty Cycle", shoulderLeft.get());
        SmartDashboard.putNumber("Arm/Shoulder/Right Duty Cycle", shoulderRight.get());
        SmartDashboard.putNumber("Arm/Wrist/Duty Cycle", wrist.get());

        SmartDashboard.putBoolean("Arm/Wrist/Stalling", wristStalling.get());
    }
}

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.wpilibj.DriverStation;

public class ArmConstraints {

    private SmartNumber shoulderMaxVelocity;
    private SmartNumber shoulderMaxAcceleration;

    private SmartNumber wristMaxVelocity;
    private SmartNumber wristMaxAcceleration;

    private SmartNumber autonShoulderMaxVelocity;
    private SmartNumber autonShoulderMaxAcceleration;

    private SmartNumber autonWristMaxVelocity;
    private SmartNumber autonWristMaxAcceleration;

    public ArmConstraints(SmartNumber shoulderMaxVelocity, SmartNumber shoulderMaxAcceleration, SmartNumber wristMaxVelocity, SmartNumber wristMaxAcceleration,
                          SmartNumber autonShoulderMaxVelocity, SmartNumber autonShoulderMaxAcceleration, SmartNumber autonWristMaxVelocity, SmartNumber autonWristMaxAcceleration) {
        this.shoulderMaxVelocity = shoulderMaxVelocity;
        this.shoulderMaxAcceleration = shoulderMaxAcceleration;
        
        this.wristMaxVelocity = wristMaxVelocity;
        this.wristMaxAcceleration = wristMaxAcceleration;

        this.autonShoulderMaxVelocity = autonShoulderMaxVelocity;
        this.autonShoulderMaxAcceleration = autonShoulderMaxAcceleration;
        
        this.autonWristMaxVelocity = autonWristMaxVelocity;
        this.autonWristMaxAcceleration = autonWristMaxAcceleration;
    }

    public IStream getShoulderMaxVelocity() {
        return shoulderMaxVelocity.filtered(Math::toRadians, (x) -> DriverStation.isAutonomous() ? autonShoulderMaxVelocity.get() : x);
    }

    public IStream getShoulderMaxAcceleration() {
        return shoulderMaxAcceleration.filtered(Math::toRadians, (x) -> DriverStation.isAutonomous() ? autonShoulderMaxAcceleration.get() : x);
    }

    public IStream getWristMaxVelocity() {
        return wristMaxVelocity.filtered(Math::toRadians, (x) -> DriverStation.isAutonomous() ? autonWristMaxVelocity.get() : x);
    }

    public IStream getWristMaxAcceleration() {
        return wristMaxAcceleration.filtered(Math::toRadians, (x) -> DriverStation.isAutonomous() ? autonWristMaxAcceleration.get() : x);
    }

    public void update(double shoulderMaxVel, double shoulderMaxAccel, double wristMaxVel, double wristMaxAccel) {
        this.shoulderMaxVelocity.set(shoulderMaxVel);;
        this.shoulderMaxAcceleration.set(shoulderMaxAccel);;
        this.wristMaxVelocity.set(wristMaxVel);
        this.wristMaxAcceleration.set(wristMaxAccel);
    }
}

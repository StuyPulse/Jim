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

    public ArmConstraints(double shoulderMaxVelocity, double shoulderMaxAcceleration, double wristMaxVelocity, double wristMaxAcceleration,
                            double autonShoulderMaxVelocity, double autonShoulderMaxAcceleration, double autonWristMaxVelocity, double autonWristMaxAcceleration) {
        this.shoulderMaxVelocity = new SmartNumber("Arm/Constraints/Shoulder Max Velocity", shoulderMaxVelocity);
        this.shoulderMaxAcceleration = new SmartNumber("Arm/Constraints/Shoulder Max Acceleration", shoulderMaxAcceleration);
        
        this.wristMaxVelocity = new SmartNumber("Arm/Constraints/Wrist Max Velocity", wristMaxVelocity);
        this.wristMaxAcceleration = new SmartNumber("Arm/Constraints/Wrist Max Acceleration", wristMaxAcceleration);

        this.autonShoulderMaxVelocity = new SmartNumber("Arm/Constraints/Auto/Shoulder Max Velocity", autonShoulderMaxVelocity);
        this.autonShoulderMaxAcceleration = new SmartNumber("Arm/Constraints/Auto/Shoulder Max Acceleration", autonShoulderMaxAcceleration);
        
        this.autonWristMaxVelocity = new SmartNumber("Arm/Constraints/Auto/Wrist Max Velocity", autonWristMaxVelocity);
        this.autonWristMaxAcceleration = new SmartNumber("Arm/Constraints/Auto/Wrist Max Acceleration", autonWristMaxAcceleration);
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

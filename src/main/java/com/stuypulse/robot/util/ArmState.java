package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {

    private final double shoulderDegrees, wristDegrees;

    public ArmState(double shoulderDegrees, double wristDegrees) {
        this.shoulderDegrees = shoulderDegrees;
        this.wristDegrees = wristDegrees;
    }

    public Rotation2d getShoulderRotation() {
        return Rotation2d.fromDegrees(shoulderDegrees);
    }

    public Rotation2d getWristRotation() {
        return Rotation2d.fromDegrees(wristDegrees);
    }
}

package com.stuypulse.robot.util;

public class ArmAngles {

    private final double shoulderDegrees, wristDegrees;

    public ArmAngles(double shoulderDegrees, double wristDegrees) {
        this.shoulderDegrees = shoulderDegrees;
        this.wristDegrees = wristDegrees;
    }

    public double getShoulderDegrees() {
        return shoulderDegrees;
    }

    public double getWristDegrees() {
        return wristDegrees;
    }
}

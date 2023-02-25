package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {

    private final Number shoulder; 
    private final Number wrist;

    public ArmState(Number shoulderDegrees, Number wristDegrees) {
        this.shoulder = shoulderDegrees;
        this.wrist = wristDegrees;
    }

    public ArmState(Rotation2d shoulder, Rotation2d wrist) {
        this(shoulder.getDegrees(), wrist.getDegrees());
    }

    public Rotation2d getShoulderState() {
        return Rotation2d.fromDegrees(shoulder.doubleValue());
    }

    public Rotation2d getWristState() {
        return Rotation2d.fromDegrees(wrist.doubleValue());
    }

}

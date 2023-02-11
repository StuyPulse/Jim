package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Motors.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public class ArmState implements Interpolatable<ArmState> {

    public static final ArmState fromDegrees(double shoulderDegrees, double wristDegrees) {
        return new ArmState(Rotation2d.fromDegrees(shoulderDegrees), Rotation2d.fromDegrees(wristDegrees));
    }

    private final Rotation2d shoulder; 
    private final Rotation2d wrist;

    public ArmState(Rotation2d shoulder, Rotation2d wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    public Rotation2d getShoulderState() {
        return shoulder;
    }

    public Rotation2d getWristState() {
        return wrist;
    }

    public ArmState flip() {
        return fromDegrees(-180 - shoulder.getDegrees(), -180 - wrist.getDegrees());
    }

    public ArmState interpolate(ArmState other, double t) {
        return new ArmState(
            shoulder.interpolate(other.shoulder, t),
            wrist.interpolate(other.wrist, t)
        );
    }
}

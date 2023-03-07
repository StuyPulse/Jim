package com.stuypulse.robot.util;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {
 
    private final Number shoulder; 
    private final Number wrist;

    private Optional<Number> shoulderToleranceDegrees;
    private Optional<Number> wristToleranceDegrees;

    private boolean wristLimp;

    public ArmState(Number shoulderDegrees, Number wristDegrees) {
        this.shoulder = shoulderDegrees;
        this.wrist = wristDegrees;

        shoulderToleranceDegrees = Optional.empty();
        wristToleranceDegrees = Optional.empty();

        wristLimp = false;
    }

    public ArmState setWristLimp(boolean wristLimp) {
        this.wristLimp = wristLimp;
        return this;
    }

    public boolean isWristLimp() {
        return wristLimp;
    }

    public ArmState(Rotation2d shoulder, Rotation2d wrist) {
        this(shoulder.getDegrees(), wrist.getDegrees());
    }

    public double getShoulderDegrees() {
        return shoulder.doubleValue();
    }

    public double getWristDegrees() {
        return wrist.doubleValue();
    }

    public Rotation2d getShoulderState() {
        return Rotation2d.fromDegrees(getShoulderDegrees());
    }


    public Rotation2d getWristState() {
        return Rotation2d.fromDegrees(getWristDegrees());
    }

    // Will mutate arm state
    public ArmState setWristTolerance(Number toleranceDegrees) {
        wristToleranceDegrees = Optional.of(toleranceDegrees);
        return this;
    }

    public ArmState setShoulderTolerance(Number toleranceDegrees) {
        shoulderToleranceDegrees = Optional.of(toleranceDegrees);
        return this;
    }

    public Optional<Number> getWristTolerance() {
        return wristToleranceDegrees;
    }

    public Optional<Number> getShoulderTolerance() {
        return shoulderToleranceDegrees;
    }

    public boolean isOverBumper() {
        return Math.abs(getShoulderDegrees() + 90) < Settings.Arm.Shoulder.OVER_BUMPER_ANGLE.get();
    }

    public boolean isOnSameSide(ArmState lhs) {
        return (getShoulderDegrees() < -90) == (lhs.getShoulderDegrees() < -90);
    }

}

package com.stuypulse.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {

    private final Number shoulder; 
    private final Number wrist;

    private Optional<Number> shoulderToleranceDegrees;
    private Optional<Number> wristToleranceDegrees;

    public ArmState(Number shoulderDegrees, Number wristDegrees) {
        this.shoulder = shoulderDegrees;
        this.wrist = wristDegrees;

        shoulderToleranceDegrees = Optional.empty();
        wristToleranceDegrees = Optional.empty();
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

}

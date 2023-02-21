package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmBFSField.Constraint;

import edu.wpi.first.math.util.Units;

public interface Constraints {
    
    double INFLATION = Units.inchesToMeters(0.0);

    double SHOULDER_HEIGHT = Units.inchesToMeters(48);
    double WRIST_LENGTH = Units.inchesToMeters(17);
    double SHOULDER_LENGTH = Units.inchesToMeters(42.25);

    double BUMPER_WIDTH = Units.inchesToMeters(32) + 2 * INFLATION;
    double BUMPER_HEIGHT = Units.inchesToMeters(5.5) + INFLATION;
    double FLOOR_HEIGHT = Units.inchesToMeters(0) + INFLATION;

    Constraint BUMPER_CONSTRAINT = (s, w) -> {
        final double x = SHOULDER_LENGTH * Math.cos(Math.toRadians(s)) + WRIST_LENGTH * Math.cos(Math.toRadians(w));
        final double y = SHOULDER_HEIGHT + SHOULDER_LENGTH * Math.sin(Math.toRadians(s)) + WRIST_LENGTH * Math.sin(Math.toRadians(w));
        
        if (Math.abs(x) < BUMPER_WIDTH / 2) {
            return y < BUMPER_HEIGHT;
        } else {
            return y < FLOOR_HEIGHT;
        }
    };

    Constraint FLIP_CONSTRAINT = (s, w) -> {
        double shoulderTolerance = 13;
        double wristTolerance = 10;
        return (Math.abs(s - (-90)) < shoulderTolerance) && (Math.abs(w - 90) > wristTolerance);
    };

    double MAX_SHOULDER_ANGLE = 25.0; // degrees

    Constraint SHOULDER_CONSTRAINT = (s, w) -> {
        return MAX_SHOULDER_ANGLE < s && s < (180 - MAX_SHOULDER_ANGLE);
    };

    Constraint CONSTRAINT = BUMPER_CONSTRAINT.add(SHOULDER_CONSTRAINT).add(FLIP_CONSTRAINT);
}

package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmBFSField.Constraint;

import edu.wpi.first.math.util.Units;

public interface Constraints {
    
    double INFLATION = Units.inchesToMeters(0.0);

    double SHOULDER_HEIGHT = Units.inchesToMeters(48);
    double SHOULDER_LENGTH = Units.inchesToMeters(42.25);

    double WRIST_LENGTH = Units.inchesToMeters(17);
    double CUBE_SUPPORT_LENGTH = Units.inchesToMeters(6);
    double CUBE_SUPPORT_ANGLE = 35;

    double BUMPER_WIDTH = Units.inchesToMeters(32) + 2 * INFLATION;
    double BUMPER_HEIGHT = Units.inchesToMeters(5.5) + INFLATION;
    double FLOOR_HEIGHT = Units.inchesToMeters(0) + INFLATION;

    public static boolean isInvalid(double s, double w, double wLen) {
        final double x = SHOULDER_LENGTH * Math.cos(Math.toRadians(s)) + wLen * Math.cos(Math.toRadians(w));
        final double y = SHOULDER_HEIGHT + SHOULDER_LENGTH * Math.sin(Math.toRadians(s)) + wLen * Math.sin(Math.toRadians(w));
        
        if (Math.abs(x) < BUMPER_WIDTH / 2) {
            return y < BUMPER_HEIGHT;
        } else {
            return y < FLOOR_HEIGHT;
        }
    }

    Constraint BUMPER_CONSTRAINT = (s, w) -> {
        return isInvalid(s, w, WRIST_LENGTH) ||
        isInvalid(s, w + CUBE_SUPPORT_ANGLE, CUBE_SUPPORT_LENGTH) ||
        isInvalid(s, w + CUBE_SUPPORT_ANGLE, CUBE_SUPPORT_LENGTH);

    };

    Constraint FLIP_CONSTRAINT = (s, w) -> {
        final boolean isNotUp = Math.abs(w - 90) > 10;
        final boolean isNotSideways = Math.abs(w - 90) < 30;
        
        final boolean isOverMiddle = Math.abs(s - (-90)) < 10;
        final boolean isOverBumper = Math.abs(s - (-90)) < 30;

        return (
            (isOverMiddle && isNotUp) ||
            (isOverBumper && isNotUp && isNotSideways)
        );
    };

    double MAX_SHOULDER_ANGLE = 25.0; // degrees

    Constraint SHOULDER_CONSTRAINT = (s, w) -> {
        return MAX_SHOULDER_ANGLE < s && s < (180 - MAX_SHOULDER_ANGLE);
    };

    Constraint CONSTRAINT = BUMPER_CONSTRAINT.add(SHOULDER_CONSTRAINT).add(FLIP_CONSTRAINT);
}

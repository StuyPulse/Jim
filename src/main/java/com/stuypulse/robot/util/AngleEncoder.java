package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Absolute to relative angle
 * Takes absolute angle, calculates deltas and integrates
 */
public class AngleEncoder {
    private Rotation2d previousAngle;
    private double accumulatedRadians;

    public void reset(Rotation2d angle) {
        previousAngle = angle;
        accumulatedRadians = angle.getRadians();
    }

    public void update(Rotation2d angle) {
        if (previousAngle == null) {
            reset(angle);
        } else {
            accumulatedRadians += angle.minus(previousAngle).getRadians();
            previousAngle = angle;
        }

        SmartDashboard.putNumber("BOOM/prev angle", previousAngle.getRadians());
        SmartDashboard.putNumber("BOOM/difference", angle.minus(previousAngle).getDegrees());
    }

    public Rotation2d getAngle() {
        return new Rotation2d(accumulatedRadians);
    }

    public double getRadians() {
        return accumulatedRadians;
    }

    public double getDegrees() {
        return Units.radiansToDegrees(accumulatedRadians);
    }
}

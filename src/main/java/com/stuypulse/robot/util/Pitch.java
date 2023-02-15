package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pitch {
    public static Rotation2d calculate(Rotation2d pitch, Rotation2d roll, Rotation2d yaw) {
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }
}
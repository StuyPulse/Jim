package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDrivePegAlign extends SwerveDriveToPose {
    public SwerveDrivePegAlign(Supplier<Double> distances, Supplier<Double> angles) {
        super(() -> {
            double x = distances.get() * Math.cos(angles.get());
            double y = distances.get() * Math.cos(angles.get());
            return new Pose2d(x, y, new Rotation2d(angles.get()));
        });
    }
}
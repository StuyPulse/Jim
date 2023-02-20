package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveSequentialBalance extends SequentialCommandGroup {

    private double angle;

    public SwerveDriveSequentialBalance() {

        double currentAngle = Odometry.getInstance().getRotation().getDegrees();
        // angle
        if (currentAngle > -45 && currentAngle <= 45) {
            angle = 0;
        } else if (currentAngle > 45 && currentAngle <= 135) {
            angle = 90;
        } else if (currentAngle < -45 && currentAngle >= -135) {
            angle = -90;
        } else if (currentAngle < -135 && currentAngle >= 135 ) {
            angle = 180;
        }


        addCommands(
            new SwerveDriveToPose(() -> new Pose2d(Odometry.getInstance().getTranslation(), new Rotation2d(angle))),
            new SwerveDriveBlayBalance(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90))
        );
    }
}
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveSequentialBalance extends SequentialCommandGroup {


    public SwerveDriveSequentialBalance() {
        addCommands(
            new SwerveDriveToPose(() -> new Pose2d(Odometry.getInstance().getTranslation(), new Rotation2d())),
            new SwerveDriveBlayBalance(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90))
        );
    }
}
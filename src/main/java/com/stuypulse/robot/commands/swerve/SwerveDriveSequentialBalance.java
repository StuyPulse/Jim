package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveSequentialBalance extends SequentialCommandGroup {

    public SwerveDriveSequentialBalance() {


        addCommands(
            new SwerveDriveBalanceAlign(),
            new SwerveDriveBlayBalance(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90))
        );
    }
}
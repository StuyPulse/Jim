package com.stuypulse.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveAlignThenBalance extends SequentialCommandGroup {

    public SwerveDriveAlignThenBalance() {


        addCommands(
            new SwerveDriveBalanceAlign(),
            new SwerveDriveBalanceBlay(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90))
        );
    }
}
/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.commands.swerve.SwerveDrivePointWheels;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveAlignThenBalance extends SequentialCommandGroup {

    public SwerveDriveAlignThenBalance() {


        addCommands(
            // new SwerveDriveBalanceAlign(),
            new SwerveDriveBalanceBlay(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90))
        );
    }
}

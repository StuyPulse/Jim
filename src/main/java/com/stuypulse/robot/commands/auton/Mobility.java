/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class Mobility extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public Mobility() {
        addCommands(
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Mobility", CONSTRAINTS)
            ).robotRelative()
        );
    }

}

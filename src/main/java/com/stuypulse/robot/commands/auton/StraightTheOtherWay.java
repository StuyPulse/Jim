/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class StraightTheOtherWay extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public StraightTheOtherWay() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("Straight the other way", CONSTRAINTS),
            "There", "Back"
        );

        addCommands(
            new SwerveDriveFollowTrajectory(paths.get("There")).robotRelative().withStop(),

            new WaitCommand(2),

            new SwerveDriveFollowTrajectory(paths.get("Back")).fieldRelative()
        );
    }

}

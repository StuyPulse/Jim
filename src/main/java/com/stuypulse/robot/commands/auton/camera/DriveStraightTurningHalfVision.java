/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton.camera;

import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class DriveStraightTurningHalfVision extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    private static final AbstractOdometry odometry = AbstractOdometry.getInstance();

    public DriveStraightTurningHalfVision() {
        addCommands(
            new InstantCommand(() -> odometry.setActive(false)),

            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Drive Straight Turning", CONSTRAINTS)
            ).robotRelative(),
            
            new InstantCommand(() -> odometry.setActive(true)),

            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Drive Straight Turning Back", CONSTRAINTS)
            ).fieldRelative()
        );
    }

}

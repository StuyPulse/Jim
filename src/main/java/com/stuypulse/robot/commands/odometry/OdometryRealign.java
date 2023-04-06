/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.odometry;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class OdometryRealign extends InstantCommand {
    private final Odometry odometry;
    private final Supplier<Rotation2d> references;

    public OdometryRealign(Supplier<Rotation2d> references) {
        odometry = Odometry.getInstance();
        this.references = references;

        // addRequirements(odometry);
    }

    public OdometryRealign(Rotation2d reference) {
        this(() -> reference);
    }

    public OdometryRealign() {
        this(Rotation2d.fromDegrees(0));
    }

    @Override
    public void initialize() {
        Pose2d pose = odometry.getPose();
        odometry.reset(new Pose2d(pose.getTranslation(), references.get()));
    }

}

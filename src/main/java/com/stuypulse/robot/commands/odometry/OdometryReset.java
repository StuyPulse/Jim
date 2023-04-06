/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.odometry;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class OdometryReset extends InstantCommand {
    private final Odometry odometry;
    private final Supplier<Pose2d> references;

    public OdometryReset(Supplier<Pose2d> references) {
        odometry = Odometry.getInstance();
        this.references = references;

        // addRequirements(null);
    }

    public OdometryReset(Pose2d reference) {
        this(() -> reference);
    }

    @Override
    public void initialize() {
        odometry.reset(references.get());
    }

}

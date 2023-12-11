/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractOdometry extends SubsystemBase {

    private static final AbstractOdometry instance;

    static {
        instance = new Odometry();
    }

    public static AbstractOdometry getInstance() {
        return instance;
    }

    protected AbstractOdometry() {}

    public abstract Field2d getField();

    public abstract Pose2d getPose();

    public abstract void reset(Pose2d pose2d);

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }
}

/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ReflectiveTapeData {
    public final Pose2d pose;
    public final double latency;
    public final int id;
    public final Pose3d cameraPose;

    public ReflectiveTapeData(Pose2d robotPose, double latency, int id, Pose3d cameraPose) {
        this.pose = robotPose;
        this.latency = latency;
        this.id = id;
        this.cameraPose = cameraPose;
    }

    public boolean hasAnyTarget() {
        return limelight.getValidTarget();
    }

    public Angle getXAngle() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getXAngle() was called]");
            return Angle.kZero;
        }

        return Angle.fromDegrees(
                limelight.getTargetXAngle());
    }

    public double getDistanceToTape() {
        return pose.getTranslation().getNorm();
    }
}
/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ReflectiveTapeData extends VisionData {

    public ReflectiveTapeData(Pose2d robotPose, double latency, int id, Pose3d cameraPose) {
        super(robotPose, latency, cameraPose);
    }

    public boolean hasAnyTarget() {
        return Limelight.getInstance().getValidTarget();
    }
    
    private Angle getYAngle() {
        if (!hasAnyTarget()) {
            // Settings.reportWarning("Unable To Find Target! [getYAngle() was called]");
            return Angle.kZero;
        }

        // 330.0 (-30.0 degrees) is the pitch of the limelight
        return Angle.fromDegrees(Limelight.getInstance().getTargetYAngle() - 330.0);
    }

    public double getDistance() {
        return Double.NaN;
    }
    
    @Override
    public double getDegrees() {
        if (!hasAnyTarget()) {
            return 0.0;
        }

        return Limelight.getInstance().getTargetXAngle();
    }
}
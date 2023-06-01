/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReflectiveTapeData extends VisionData {

    public ReflectiveTapeData(double distance, double xAngle, double latency, Pose3d cameraPose) {
        super(new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN)), latency, cameraPose);
        this.distance = distance;
        this.xAngle = xAngle;
    }
}
/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ReflectiveTapeData extends VisionData {
    double distance = 

    public ReflectiveTapeData(Pose2d robotPose, double distance, double xAngle, double latency, int id, Pose3d cameraPose) {
        super(robotPose, latency, cameraPose);
        this.distance = distance;
        this.xAngle = xAngle;
    }
}
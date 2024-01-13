/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {

    public final long[] ids;
    public final Pose3d cameraLocation;
    public Pose3d robotPose;
    public final double timestamp;

    public double calculateDistanceToTag(Fiducial tag) {
        return robotPose.getTranslation().getDistance(tag.getPose().getTranslation());
    }

    private int getPrimaryID() {
        if (ids.length == 0) return -1;
        return (int) ids[0];
    }

    public Fiducial getPrimaryTag() {
        return Field.getTag(getPrimaryID());
    }

    public VisionData(long[] ids, Pose3d cameraLocation, Pose3d robotPose, double timestamp) {
        this.ids = ids;
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.timestamp = timestamp;
    }
}

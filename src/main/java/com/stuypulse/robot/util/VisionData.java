/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionData {

    public final long[] ids;
    public final Translation3d[] tvecs;
    public final Pose3d cameraLocation;
    public Pose3d robotPose;
    public final double latency;

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

    public VisionData(long[] ids, Translation3d[] tvecs, Pose3d cameraLocation, Pose3d robotPose, double latency) {
        this.ids = ids;
        this.tvecs = tvecs;
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.latency = latency;
    }
}

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagData {
    public final Pose2d pose;
    public final double latency;
    public final int id;

    /**
     * Record april tag data, usually from a vision system
     * 
     * @param pose robot pose information, in meters with blue/red corner as origin
     * @param latency total latency, in seconds
     * @param id tag id
     */
    public AprilTagData(Pose2d pose, double latency, int id) {
        this.pose = pose;
        this.latency = latency;
        this.id = id;
    }
}
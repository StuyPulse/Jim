package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagData {
    public final Pose2d pose;
    public final double latency;
    public final int id;
    public final Limelight limelight;

    public double getDegreesToTag() {
        return limelight.getDegreesToTag(pose, id);
    }

    public double getDistanceToTag() {
        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.getAprilTagFromId(id).getTranslation();
        
        return robot.getDistance(tag);
    }

    /**
     * Record april tag data, usually from a vision system
     * 
     * @param pose robot pose information, in meters with blue/red corner as origin
     * @param latency total latency, in seconds
     * @param id tag id
     * @param limelight associated limelight
     */
    public AprilTagData(Pose2d pose, double latency, int id, Limelight limelight) {
        this.pose = pose;
        this.latency = latency;
        this.id = id;
        this.limelight = limelight;
    }
}
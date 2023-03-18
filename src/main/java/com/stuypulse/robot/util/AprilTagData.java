package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagData {
    public final Pose2d pose;
    public final double latency;
    public final int id;
    public final Limelight limelight;

    private static double getDegreesBetween(Rotation2d a, Rotation2d b) {
        double c = a.getCos() * b.getCos() + a.getSin() * b.getSin();
        double d = (1 - c) * 180;

        return d;
    }

    public double getDegreesToTag() {
        Rotation2d tag = Field.getAprilTagFromId(id).getRotation();

        return getDegreesBetween(tag.plus(Rotation2d.fromDegrees(180)), pose.getRotation().minus(limelight.getRobotRelativeRotation()));
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
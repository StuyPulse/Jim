package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class VisionData {
    public final Pose2d pose; 
    public final double latency;
    public final Pose3d cameraPose;
    public final int id; 
    
    protected static double getDegreesBetween(Rotation2d a, Rotation2d b) {
        double c = a.getCos() * b.getCos() + a.getSin() * b.getSin();
        double d = (1 - c) * 180;

        return d;
    }

    public abstract double getDegrees();
    public abstract double getDistance();

    /**
     * Record vision data, usually from a vision system
     *
     * @param robotPose robot pose information, in meters with blue/red corner as origin
     * @param latency total latency, in seconds
     * @param cameraPose associated camera's robot relative pose
     * @param id id of the target, -1 if not applicable (e.g. for vision tape)
    */
    public VisionData(Pose2d robotPose, double latency, int id, Pose3d cameraPose) {
        this.pose = robotPose;
        this.latency = latency;
        this.id = id;
        this.cameraPose = cameraPose;
    }
    
    public VisionData(Pose2d robotPose, double latency, Pose3d cameraPose) {
        this(robotPose, latency, -1, cameraPose);
    }
}



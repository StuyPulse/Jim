package com.stuypulse.robot.subsystems;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.LimelightTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static Vision instance;

    public static Vision getInstance() {
        if(instance == null) {
            instance = new Vision();
        } 
        return instance; 
    }


    // [] TODO: change names
    public enum Error {
        LOW,
        MID,
        HIGH;
    }
    
    public class Result {
        private final Pose2d pose;
        private final double latency;
        private final Error error;

        public Result(Pose2d pose, double latency, Error error) {
            this.pose = pose;
            this.latency = latency;
            this.error = error;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getLatency() {
            return latency;
        }

        public Error getError() {
            return error;
        }
    }

    private final Limelight [] limelights;

    public Vision() {
        limelights = new Limelight[2];
        limelights[0] = Limelight.getInstance(Settings.Vision.LL_FRONT);
        limelights[1] = Limelight.getInstance(Settings.Vision.LL_FRONT);
    }

    public List<Result> getResults() {
        var results = new ArrayList<Result>();

        for (Limelight camera: limelights){
            if(camera.getValidTarget()){
                results.add(getResult(camera));
            }
        }
        return results;
    }

    private Result getResult(Limelight limelight) {
        double robotDistance = distanceToTarget(getPose2d(limelight), getAprilTagPose2d(limelight));
        boolean inZone = robotDistance <= Settings.Vision.STATION_DISTANCE;
        boolean inTolerance = robotDistance <= Settings.Vision.TOLERANCE;
        Error error = Error.HIGH;
        if(inZone){
            error = Error.LOW;
        }
        else if(!inZone && inTolerance){
            error = Error.MID;
        }
        else if(!inTolerance){
            error = Error.HIGH;
        }
        return new Result(getPose2d(limelight), limelight.getLatencyMs(), error);
    }

    private double distanceToTarget(Pose2d aprilTag, Pose2d robot) {
        return Math.sqrt(Math.pow(aprilTag.getX() - robot.getX(), 2) + Math.pow(aprilTag.getY() - robot.getY(), 2));
    }
    
    private Pose2d getPose2d(Limelight limelight) {
        return getPose3d(limelight).toPose2d();
    }

    private Pose3d getPose3d(Limelight limelight){
        if (limelight.getRobotPose() == null) {
			return new Pose3d();
		}
        return limelight.getRobotPose();
    }

    private Pose2d getAprilTagPose2d(Limelight limelight){
        int id = (int) limelight.getTagID();
        return Field.aprilTags[id-1].toPose2d();
    }

}
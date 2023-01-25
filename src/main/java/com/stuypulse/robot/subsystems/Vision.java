package com.stuypulse.robot.subsystems;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.Solve3DResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static Vision instance;

    public static Vision getInstance() {
        if(instance == null) {
            instance = new Vision();
        } 
        return instance; 
    }

    public enum ResultStatus {
        TRUST(0),
        TRUST_WITH_ERROR(1),
        UNTRUST(2);

        private final int value;

        private ResultStatus(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }

        public static ResultStatus fromValue(double distance) {
            if (distance > Settings.Vision.TOLERANCE)
                return TRUST;
            return UNTRUST;
        }

    }
    
    public class Result {
        private final Pose2d pose;
        private double latency;
        private ResultStatus status;

        public Result() {
            pose = new Pose2d();
            latency = 0;
            status = ResultStatus.UNTRUST;
        }

        public Result(Pose2d pose, double latency, ResultStatus status) {
            this.pose = pose;
            this.latency = latency;
            this.status = status;
        }
        
        public Pose2d getPose() {
            return pose;
        }

        public double getLatency() {
            return latency;
        }

        public ResultStatus getStatus() {
            return status;
        }


    }

    private final Limelight front;
    private final Limelight back;
    private List<Result> results;

    public Vision() {
        front = Limelight.getInstance("front");
        back = Limelight.getInstance("back");
    }

    public List<Result> getResult() {
        var results = new ArrayList<Result>();
        updateResult();
        return results;
    }

    private void updateResult() {
        if(front.getValidTarget()) {
            Pose2d frontTag = Field.aprilTags[(int)front.getTagID()-1].toPose2d();
            double frontDistance = distanceToTarget(frontTag, getPose2d(front));
            results.add(new Result(frontTag, frontDistance, ResultStatus.fromValue(frontDistance)));
        }
        if (back.getValidTarget()) {
            Pose2d backTag = Field.aprilTags[(int)back.getTagID()-1].toPose2d();
            double backDistance = distanceToTarget(backTag, getPose2d(back));
            results.add(new Result(backTag, backDistance, ResultStatus.fromValue(backDistance)));
        } 
    }

    private double distanceToTarget(Pose2d aprilTag, Pose2d robot) {
        return Math.sqrt(Math.pow(aprilTag.getX() - robot.getX(), 2) + Math.pow(aprilTag.getY() - robot.getY(), 2));
    }
    
    private Pose2d getPose2d(Limelight limelight) {
        Pose3d pose = getPose3d(limelight);
        return pose.toPose2d();
    }

    private Pose3d getPose3d(Limelight limelight){
        if (limelight.getRobotPose() == null) {
			return new Pose3d();
		}
        return limelight.getRobotPose();
    }
}
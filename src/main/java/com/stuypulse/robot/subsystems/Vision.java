package com.stuypulse.robot.subsystems;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.Solve3DResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

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
            if (distance > 2.0)
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


    }

    private final Limelight front;
    private final Limelight back;
    private Result result;

    public Vision() {
        front = Limelight.getInstance("front");
        back = Limelight.getInstance("back");
        result = new Result();
    }

    Result getResult(Limelight limelight) {
        return result;      
    }

    public void updateResult() {
        if (front.getValidTarget() && back.getValidTarget()) {
            
        } else if (front.getValidTarget()) {
            result = new Result(
                getPose2d(front),
                front.getLatencyMs(),
                ResultStatus.fromValue(
                    
                ));
        } else if (back.getValidTarget()) {
            result = new Result(
                getPose2d(front),
                front.getLatencyMs(),
                ResultStatus.fromValue(
                    
                ));
        }
    }

    private double distanceToTarget(Pose2d aprilTag, Pose2d robot) {
        return Math.sqrt(Math.pow(aprilTag.getX() - robot.getX(), 2) + Math.pow(aprilTag.getY() - robot.getY(), 2));
    }
    
    private Pose2d getPose2d(Limelight limelight) {
        Pose3d pose = getPose3d(limelight);
        return new Pose2d(pose.getTranslation().getX(),pose.getTranslation().getY(),pose.getRotation().toRotation2d());
    }

    private Pose3d getPose3d(Limelight limelight){
        if (limelight.getRobotPose() == null) {
			return new Pose3d();
		}
        return limelight.getRobotPose();
    }
}
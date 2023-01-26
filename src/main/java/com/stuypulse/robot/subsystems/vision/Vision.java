package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends IVision {

    private static Vision instance;

    public static Vision getInstance() {
        if(instance == null) {
            instance = new Vision();
        } 
        return instance; 
    }
    
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
        String [] hostNames = Settings.Vision.Limelight.LIMELIGHTS;
        limelights = new Limelight[hostNames.length];

        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = Limelight.getInstance(hostNames[i]);
            for (int port : Settings.Vision.Limelight.PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }
        CameraServer.startAutomaticCapture();
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
        double robotDistance = distToTarget(getPose2d(limelight), getTagPose2d(limelight));
        boolean inZone = robotDistance <= Settings.Vision.COMMUNITY_DISTANCE;
        boolean inTolerance = robotDistance <= Settings.Vision.TOLERANCE;

        // defaults to high error
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

        return new Result(getPose2d(limelight), getLatency(limelight), error);
    }

    public double getLatency(Limelight limelight){
        return limelight.getLatencyMs()/1000.0;
    }

    private double distToTarget(Pose2d aprilTag, Pose2d robot) {
        return Math.hypot(aprilTag.getX()- robot.getX(), aprilTag.getY() - robot.getY());
        // return Math.sqrt(Math.pow(aprilTag.getX() - robot.getX(), 2) + Math.pow(aprilTag.getY() - robot.getY(), 2));
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

    private Pose2d getTagPose2d(Limelight limelight){
        if(!limelight.getValidTarget()){
            return new Pose2d();
        }
        int id = (int) limelight.getTagID();
        return Field.aprilTags[id-1].toPose2d();
    }

    @Override
    public void periodic(){
        for (int i = 0; i < limelights.length; i ++){
            Limelight camera = limelights[i];
            Pose2d pose = getPose2d(limelights[i]);
            String limelight = Settings.Vision.Limelight.LIMELIGHTS[i];

            if (!camera.isConnected()){
                System.out.println("[WARNING] Limelight " + limelight + "is disconected.");
            }

            SmartDashboard.putNumber("Vision/" + limelight +  "/Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/" + limelight +  "/Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/" + limelight +"Pose Rotation", pose.getRotation().getDegrees());   
        }
    }
}
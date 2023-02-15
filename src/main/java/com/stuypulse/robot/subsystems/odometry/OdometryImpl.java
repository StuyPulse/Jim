package com.stuypulse.robot.subsystems.odometry;

import java.util.List;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionImpl;
import com.stuypulse.robot.subsystems.vision.Vision.Noise;
import com.stuypulse.robot.subsystems.vision.Vision.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OdometryImpl extends Odometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private boolean overrideNoise;

    public OdometryImpl() {   
        var swerve = SwerveDrive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        odometry = new SwerveDriveOdometry(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, Units.degreesToRadians(10)));
        field = new Field2d();

        overrideNoise = false;

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.resetPosition(
                    drive.getGyroAngle(), 
                    drive.getModulePositions(), 
                    pose
        );
        odometry.resetPosition(drive.getGyroAngle(), 
        drive.getModulePositions(), 
        pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    public void overrideNoise(boolean overrideNoise) {
        this.overrideNoise = overrideNoise;
    }

    private void processResults(List<Result> results, SwerveDrive drive, Vision vision){  
        for (Result result : vision.getResults()) {

            if (overrideNoise) {
                poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(1, 1, Math.toRadians(5)));
                return;
            }
            
            switch (result.getNoise()) {
                case LOW:
                    // pose estimator add vision measurement
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(3, 3, Math.toRadians(10)));
                        // TODO: Fill in constants
                    // pose estimator reset
                    // poseEstimator.resetPosition(
                    //     drive.getGyroAngle(), 
                    //     drive.getModulePositions(),
                    //     result.getPose());

                    
                    break;

                case MID:
                    // pose estimator add vision measurement
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(10, 10, Math.toRadians(15)));
                        // TODO: Fill in constants
                    break;
                
                case HIGH:
                    break; // DO NOT DO ANYTHING
            }
        }  
    }

    @Override
    public void periodic() {

        SwerveDrive drive = SwerveDrive.getInstance();
        Vision vision = VisionImpl.getInstance();
        List<Result> results = vision.getResults();
        
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
        processResults(results, drive, vision);
        

        // logging from pose estimator
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
    }

}
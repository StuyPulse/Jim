package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.constants.Motors.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.IVision;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.IVision.Noise;
import com.stuypulse.robot.subsystems.vision.IVision.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Odometry extends IOdometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    public Odometry() {   
        var swerve = SwerveDrive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        odometry = new SwerveDriveOdometry(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, Units.degreesToRadians(10)));
        field = new Field2d();

        SmartDashboard.putData("Field", field); 
        swerve.initFieldObjects(field);
    }

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

    

    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        IVision vision = Vision.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
        odometry.update(drive.getGyroAngle(), drive.getModulePositions());

        for (Result result : vision.getResults()) {
            // result.noise = Noise.HIGH;
            switch (result.getNoise()) {
                case LOW:
                    SmartDashboard.putString("Odometry/Noise", "LOW");
                    // pose estimator add vision measurement
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(0.1, 0.1, Math.toRadians(5)));
                        // TODO: Fill in constants
                    // pose estimator reset
                    // poseEstimator.resetPosition(
                    //     drive.getGyroAngle(), 
                    //     drive.getModulePositions(),
                    //     result.getPose());[]\[]\

                    
                    break;

                case MID:
                    SmartDashboard.putString("Odometry/Noise", "MID");
                    // pose estimator add vision measurement
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(5, 5, Math.toRadians(10)));
                        // TODO: Fill in constants
                    break;
                
                case HIGH:
                    SmartDashboard.putString("Odometry/Noise", "HIGH");
                    break; // DO NOT DO ANYTHING
            }
        }    
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
    }

}
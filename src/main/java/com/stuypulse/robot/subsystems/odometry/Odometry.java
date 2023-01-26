package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.Vision.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Odometry extends IOdometry {

    private static Odometry instance;

    public static Odometry getInstance(){
        if(instance == null){
            instance = new Odometry();
        }
        return instance;
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    public Odometry() {   
        var swerve = SwerveDrive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));
        field = new Field2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        Vision vision = Vision.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());

        for (Result result : vision.getResults()) {
            switch (result.getError()) {
                case LOW:
                    // pose estimator reset
                    poseEstimator.resetPosition(
                        drive.getGyroAngle(), 
                        drive.getModulePositions(),
                        result.getPose());
                    break;

                case MID:
                    // pose estimator add vision measurement
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency());
                    break;
                
                case HIGH:
                    break; // DO NOT DO ANYTHING
            }
        }    
        field.setRobotPose(getPose());
    }

}
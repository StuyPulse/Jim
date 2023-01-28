package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.constants.Motors.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.IVision;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.IVision.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Odometry extends IOdometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    public Odometry() {   
        var swerve = SwerveDrive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));
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
    }

    

    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        IVision vision = Vision.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());

        for (Result result : vision.getResults()) {
            switch (result.getNoise()) {
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
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VecBuilder.fill(0, 0, 0));
                        // TODO: Fill in constants
                    break;
                
                case HIGH:
                    break; // DO NOT DO ANYTHING
            }
        }    
        field.setRobotPose(getPose());
    }

}
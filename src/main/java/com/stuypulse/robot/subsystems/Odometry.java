package com.stuypulse.robot.subsystems;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.Solve3DResult;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Odometry extends SubsystemBase {

    

    private final SwerveDrivePoseEstimator poseEstimator;

    public Odometry() {   
        var swerve = SwerveDrive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(drive.getKinematics(), drive.getGyroAngle(), drive.getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    

    @Override
    public void periodic() {
        drive = SwerveDrive.getInstance();
        vision = Vision.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
    }

}
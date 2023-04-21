/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.AprilTagData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;


public class OdometryImpl extends Odometry {

    public static final SmartBoolean DISABLE_APRIL_TAGS = new SmartBoolean("Odometry/Disable April Tags", false);

    private interface VisionStdDevs {
        Vector<N3> AUTO = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    Vector<N3> TELEOP = VecBuilder.fill(0.3 - Units.inchesToMeters(5.0), 0.3 - Units.inchesToMeters(5.0), Units.degreesToRadians(30));
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private final FieldObject2d odometryPose2d;
    private final FieldObject2d poseEstimatorPose2d;

    protected OdometryImpl() {
        var swerve = SwerveDrive.getInstance();
        var startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        poseEstimator =
            new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                startingPose,

                VecBuilder.fill(
                    0.1,
                    0.1,
                    0.1),

                VisionStdDevs.TELEOP);

        odometry =
            new SwerveDriveOdometry(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                startingPose);

        field = new Field2d();

        odometryPose2d = field.getObject("Odometry Pose2d");
        poseEstimatorPose2d = field.getObject("Pose Estimator Pose2d");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        if (USE_VISION_ANGLE.get())
            return poseEstimator.getEstimatedPosition();
        else
            return new Pose2d(
                poseEstimator.getEstimatedPosition().getTranslation(),
                odometry.getPoseMeters().getRotation());
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();

        poseEstimator.resetPosition(
            drive.getGyroAngle(),
            drive.getModulePositions(),
            pose);

        odometry.resetPosition(
            drive.getGyroAngle(),
            drive.getModulePositions(),
            pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    private void processResults(List<AprilTagData> results, SwerveDrive drive, Vision vision){
        if (DISABLE_APRIL_TAGS.get()) {
            return;
        }

        for (AprilTagData result : results) {
            if (Robot.getMatchState() == MatchState.AUTO) {
                // poseEstimator.addVisionMeasurement(
                //     new Pose2d(result.pose.getTranslation(), getRotation()),
                //     Timer.getFPGATimestamp() - result.latency,
                //     VisionStdDevs.AUTO);
            } else {
                poseEstimator.addVisionMeasurement(
                    result.pose,
                    Timer.getFPGATimestamp() - result.latency,
                    VisionStdDevs.TELEOP);
            }
        }
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
        odometry.update(drive.getGyroAngle(), drive.getModulePositions());

        poseEstimatorPose2d.setPose(poseEstimator.getEstimatedPosition());

        Vision vision = Vision.getInstance();
        List<AprilTagData> results = Vision.getInstance().getResults();
        processResults(results, drive, vision);

        odometryPose2d.setPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}

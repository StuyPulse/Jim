/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.VisionData;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry extends AbstractOdometry {

    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;

    private final SmartBoolean VISION_ACTIVE;

    protected Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();

        this.odometry =
                new SwerveDriveOdometry(
                        swerve.getKinematics(),
                        swerve.getGyroAngle(),
                        swerve.getModulePositions(),
                        new Pose2d());

        this.estimator =
                new SwerveDrivePoseEstimator(
                        swerve.getKinematics(),
                        swerve.getGyroAngle(),
                        swerve.getModulePositions(),
                        new Pose2d());

        this.field = new Field2d();
        this.odometryPose2D = field.getObject("Odometry Pose");
        this.estimatorPose2D = field.getObject("Estimator Pose");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
        VISION_ACTIVE = new SmartBoolean("Odometry/VISION ACTIVE", true);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(
                estimator.getEstimatedPosition().getTranslation(),
                estimator.getEstimatedPosition().getRotation());
    }

    @Override
    public void reset(Pose2d pose2d) {
        SwerveDrive swerve = SwerveDrive.getInstance();

        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose2d);
        estimator.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose2d);
    }

    @Override
    public void setActive(boolean active) {
        VISION_ACTIVE.set(active);
    }

    @Override
    public void periodic() {
        SwerveDrive swerve = SwerveDrive.getInstance();

        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());

        if (VISION_ACTIVE.getAsBoolean()) {
            for (VisionData result : AbstractVision.getInstance().getOutput()) {

                Fiducial primaryTag = result.getPrimaryTag();
                double distance = result.calculateDistanceToTag(primaryTag);

                SmartDashboard.putNumber("Odometry/Primary Tag/Distance", distance);

                estimator.addVisionMeasurement(
                    result.robotPose.toPose2d(),
                    result.timestamp);
            }
        }

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());

        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
    }
}

/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.LinearRegression;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class Odometry extends AbstractOdometry {

    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDriveOdometry odometry;

    private final LinearRegression xyRegression;
    private final LinearRegression thetaRegression;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;

    Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0));

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
                        new Pose2d(),
                        VecBuilder.fill(100000, 100000, 100000),
                        visionStdDevs);

        xyRegression = new LinearRegression(Field.xyStdDevs);
        thetaRegression = new LinearRegression(Field.thetaStdDevs);

        this.field = new Field2d();
        this.odometryPose2D = field.getObject("Odometry Pose");
        this.estimatorPose2D = field.getObject("Estimator Pose");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
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

    private Vector<N3> getStdDevs(double distance) {
        double xyStdDev = xyRegression.calculatePoint(distance);
        double thetaStdDev = thetaRegression.calculatePoint(distance);

        SmartDashboard.putNumber("Odometry/StdDevs/XY", xyStdDev);
        SmartDashboard.putNumber("Odometry/StdDevs/Theta", thetaStdDev);

        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev);
    }

    private void updateWithVision(List<VisionData> visionData) {
        for (VisionData result : visionData) {
            // if (Math.abs(result.robotPose.getRotation().getZ() - lastAngle) > 30) {
            //     SmartDashboard.putBoolean("Odometry/Rotation Flip Blocked", true);
            //     result.robotPose = new Pose3d(result.robotPose.getTranslation(), new Rotation3d(result.robotPose.getRotation().getX(), result.robotPose.getRotation().getY(), -result.robotPose.getRotation().getZ()));
            // } else {
            //     SmartDashboard.putBoolean("Odometry/Rotation Flip Blocked", false);
            // }

            Fiducial primaryTag = result.getPrimaryTag();
            double distance = result.calculateDistanceToTag(primaryTag);

            SmartDashboard.putNumber("Odometry/Primary Tag/Distance", distance);

            // estimator.addVisionMeasurement(
            //     result.robotPose.toPose2d(),
            //     Timer.getFPGATimestamp() - result.latency, 
            //     getStdDevs(distance));
            
            estimator.addVisionMeasurement(
                result.robotPose.toPose2d(), 
                Timer.getFPGATimestamp() - result.latency);
        }
    }

    @Override
    public void periodic() {
        SwerveDrive swerve = SwerveDrive.getInstance();

        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());

        List<VisionData> output = AbstractVision.getInstance().getOutput();
        
        if (!output.isEmpty()) updateWithVision(output);

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());

        SmartDashboard.putBoolean("Vision/Is Empty", output.isEmpty());

        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
    }
}

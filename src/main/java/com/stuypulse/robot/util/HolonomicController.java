/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class HolonomicController implements Sendable {
    private final Controller xController;
    private final Controller yController;
    private final AngleController angleController;

    public HolonomicController(Controller xController, Controller yController, AngleController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;
    }

    public ChassisSpeeds update(Pose2d setpoint, Pose2d measurement) {
        xController.update(setpoint.getX(), measurement.getX());
        yController.update(setpoint.getY(), measurement.getY());
        angleController.update(
            Angle.fromRotation2d(setpoint.getRotation()),
            Angle.fromRotation2d(measurement.getRotation()));


        return getOutput();
    }

    public ChassisSpeeds getOutput() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.getOutput(),
            yController.getOutput(),
            angleController.getOutput(),
            angleController.getMeasurement().getRotation2d());
    }

    public boolean isDone(double xToleranceMeters, double yToleranceMeters, double angleToleranceDegrees) {
        return xController.isDone(xToleranceMeters) && yController.isDone(yToleranceMeters) && angleController.isDoneDegrees(angleToleranceDegrees);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Holonomic Controller");
        builder.addDoubleProperty("Angle Setpoint (degrees)", () -> angleController.getSetpoint().toDegrees(), null);
        builder.addDoubleProperty("Angle Measurement (degrees)", () -> angleController.getMeasurement().toDegrees(), null);
        builder.addDoubleProperty("X Setpoint (meters)", () -> xController.getSetpoint(), null);
        builder.addDoubleProperty("X Measurement (meters)", () -> xController.getMeasurement(), null);
        builder.addDoubleProperty("Y Setpoint (meters)", () -> yController.getSetpoint(), null);
        builder.addDoubleProperty("Y Measurement (meters)", () -> yController.getMeasurement(), null);

    }



}

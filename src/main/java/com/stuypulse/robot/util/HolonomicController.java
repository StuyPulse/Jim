/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class HolonomicController implements Sendable {

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController angleController;

    public HolonomicController(PIDController xController, PIDController yController, PIDController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;

        angleController.enableContinuousInput(-Math.PI, +Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d setpoint, Pose2d measurement) {
        double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
        double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
        double angleOutput = angleController.calculate(
            measurement.getRotation().getRadians(),
            setpoint.getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput,
            yOutput,
            angleOutput,
            measurement.getRotation());
    }

    public boolean isDone(double xToleranceMeters, double yToleranceMeters, double angleToleranceDegrees) {
        xController.setTolerance(xToleranceMeters);
        yController.setTolerance(yToleranceMeters);
        angleController.setTolerance(Math.toRadians(angleToleranceDegrees));

        return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Holonomic Controller");

        builder.addCloseable(xController);
        builder.addCloseable(yController);
        builder.addCloseable(angleController);
    }



}

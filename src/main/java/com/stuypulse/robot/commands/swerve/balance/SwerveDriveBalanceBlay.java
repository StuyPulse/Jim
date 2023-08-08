/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceBlay extends CommandBase {

    private final Controller control;

    private double angleThreshold;

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Plant plant;

    public SwerveDriveBalanceBlay() {
        this(AutoBalance.MAX_SPEED.doubleValue());
    }

    public SwerveDriveBalanceBlay(double maxSpeed) {
        double kK_u = maxSpeed / AutoBalance.MAX_TILT.doubleValue();
        double kP = 0.8 * kK_u;
        double kD = 0.1 * kK_u * AutoBalance.kT_u.doubleValue();

        angleThreshold = AutoBalance.ANGLE_THRESHOLD.doubleValue();

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        plant = Plant.getInstance();
        control = new PIDController(kP, 0, kD).setOutputFilter(x -> -x);

        addRequirements(swerve, plant);
    }

    @Override
    public void execute() {
        control.update(0, swerve.getBalanceAngle().getDegrees());

        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            control.getOutput(), 0, 0, odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/Speed", control.getOutput());
    }

    private boolean timedOut = false;

    @Override
    public boolean isFinished() {
        if (Robot.getMatchState() == MatchState.AUTO && Timer.getMatchTime() < 0.1) {
            timedOut = true;
            return true;
        }
        return control.isDone(angleThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        swerve.setXMode();
        plant.engage();

        if (timedOut) {
            LEDController.getInstance().setColor(LEDColor.YELLOW, 1);
        }
    }

    public SwerveDriveBalanceBlay withAngleThreshold(double degreesThreshold) {
        this.angleThreshold = degreesThreshold;
        return this;
    }
}

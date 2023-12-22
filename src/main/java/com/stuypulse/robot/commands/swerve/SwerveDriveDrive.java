/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Driver.Turn.GyroFeedback;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Optional;

public class SwerveDriveDrive extends CommandBase {

    private final SwerveDrive swerve;
    private final Plant plant;

    private VStream speed;
    private IStream turn;

    private final Gamepad driver;

    private final AngleController gyroFeedback;
    private Optional<Rotation2d> holdAngle;

    public SwerveDriveDrive(Gamepad driver) {
        this.driver = driver;

        swerve = SwerveDrive.getInstance();
        plant = Plant.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC)
            );


        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Turn.RC)
            );

        holdAngle = Optional.empty();
        gyroFeedback = new AnglePIDController(GyroFeedback.P, GyroFeedback.I, GyroFeedback.D);

        addRequirements(swerve, plant);
    }

    @Override
    public void initialize() {
        holdAngle = Optional.empty();
    }

    private boolean isTurnInDeadband() {
        return Math.abs(turn.get()) < Turn.DEADBAND.get();
    }

    private boolean isDriveInDeadband() {
        return driver.getLeftStick().magnitude() < Drive.DEADBAND.get();
    }

    @Override
    public void execute() {
        double angularVel = turn.get();

        // if turn in deadband, save the current angle and calculate small adjustments
        if (isTurnInDeadband()) {
            if (holdAngle.isEmpty()) {
                holdAngle = Optional.of(swerve.getGyroAngle());
            }

            if (GyroFeedback.GYRO_FEEDBACK_ENABLED.get() && !isDriveInDeadband()) {
                angularVel = -gyroFeedback.update(
                    Angle.fromRotation2d(holdAngle.get()),
                    Angle.fromRotation2d(swerve.getGyroAngle()));
            }
        }

        // if turn outside deadband, clear the saved angle
        else {
            holdAngle = Optional.empty();
        }

        // unplant if driving in endgame
        if (Timer.getMatchTime() < 30) {
            if ((driver.getLeftStick().magnitude() > 0.5) ||
                (driver.getRightStick().magnitude() > 0.5)) {

                plant.disengage();
            }
        }

        // if planted then X
        if (plant.isEngaged() || driver.getRawStartButton() || driver.getRawSelectButton()) {
            swerve.setXMode();
        }

        // if in robot relative mode, set raw chassis speed
        else if (driver.getRawLeftBumper()) {
            swerve.setChassisSpeeds(new ChassisSpeeds(speed.get().y, -speed.get().x, -angularVel));
        }

        // use the angularVelocity for drive otherwise
        else {
            swerve.drive(speed.get(), angularVel);
        }
    }
}

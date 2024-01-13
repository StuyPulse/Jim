/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn.GyroFeedback;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveAimbot extends CommandBase {

    private final SwerveDrive swerve;
    private final Plant plant;

    private VStream speed;

    private final AngleController gyroFeedback;

    public SwerveDriveAimbot(Gamepad driver) {
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

        gyroFeedback = new AnglePIDController(GyroFeedback.P, GyroFeedback.I, GyroFeedback.D);
        addRequirements(swerve, plant);
    }

    @Override
    public void execute() {
        
        Pose2d robotPose = Odometry.getInstance().getPose();

        Rotation2d target = new Rotation2d(
                robotPose.getX() - Field.WIDTH/2.0, 
                robotPose.getY() - Field.HEIGHT/2.0
            ).minus(Rotation2d.fromDegrees(180));
            
        double angularVel = -gyroFeedback.update(
                    Angle.fromRotation2d(target.plus(Rotation2d.fromDegrees(180))),
                    Angle.fromRotation2d(swerve.getGyroAngle()));

        swerve.drive(speed.get(), angularVel);
    }
}

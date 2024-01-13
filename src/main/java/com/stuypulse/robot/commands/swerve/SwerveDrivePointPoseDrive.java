/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrivePointPoseDrive extends CommandBase {

    private final SwerveDrive swerve;
    private VStream speed;
    private final AnglePIDController turnController;
    private final Pose2d target;

    public SwerveDrivePointPoseDrive(Pose2d target, Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();

        speed = VStream.create(driver::getLeftStick)
                .filtered(
                    new VDeadZone(Drive.DEADBAND),
                    x -> x.clamp(1.0),
                    x -> Settings.vpow(x, Drive.POWER.get()),
                    x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                    new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                    new VLowPassFilter(Drive.RC)
                );

        this.turnController = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);

        this.target = target;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        AbstractOdometry odometry = Odometry.getInstance();
        Pose2d difference = new Pose2d(odometry.getPose().getX() - target.getX(), odometry.getPose().getY() - target.getY(), odometry.getRotation());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan(difference.getX() / difference.getY()));

        turnController.update(Angle.fromRotation2d(angle), Angle.fromRotation2d(odometry.getRotation()));

        swerve.drive(speed.get(), turnController.getOutput());

        SmartDashboard.putNumber("Point At Pose/Error (deg)", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Point At Pose/Output", turnController.getOutput());
        SmartDashboard.putNumber("Point At Pose/Difference X", difference.getX());
        SmartDashboard.putNumber("Point At Pose/Difference Y", difference.getY());
    }
}

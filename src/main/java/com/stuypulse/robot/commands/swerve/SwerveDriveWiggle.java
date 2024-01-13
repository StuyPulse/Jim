/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.util.StopWatch;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWiggle extends Command {

    private StopWatch timer;

    private final IStream rotation;

    private final SwerveDrive swerve;

    public SwerveDriveWiggle(double period, double radPerSecondAmplitude) {
        timer = new StopWatch();

        rotation = IStream.create(() -> {
            return Math.sin((timer.getTime() * 2 * Math.PI) / period);
        });

        this.swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, rotation.get()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}

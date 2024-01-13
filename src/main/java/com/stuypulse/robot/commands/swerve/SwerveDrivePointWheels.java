/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePointWheels extends Command {

    private final SwerveDrive swerve;
    private final Rotation2d angle;

    public SwerveDrivePointWheels(Rotation2d angle) {
        this.swerve = SwerveDrive.getInstance();
        this.angle = angle;

        addRequirements(swerve);
    }

    public SwerveDrivePointWheels() {
        this(Rotation2d.fromDegrees(0));
    }

    @Override
    public void execute(){
        SwerveModuleState state = new SwerveModuleState(
            0,
            angle.minus(Odometry.getInstance().getRotation()));
        swerve.setModuleStates(state, state, state, state);
    }

}

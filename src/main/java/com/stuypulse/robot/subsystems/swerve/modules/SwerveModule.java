/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {
    public abstract String getID();
    public abstract Translation2d getOffset();
    public abstract SwerveModuleState getState();

    public abstract SwerveModulePosition getModulePosition();
    public abstract void setTargetState(SwerveModuleState swerveModuleState);

}

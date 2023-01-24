package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ISwerveModule extends SubsystemBase {
    public abstract String getID();
    public abstract Translation2d getLocation();
    public abstract SwerveModuleState getState();

    public abstract SwerveModulePosition getModulePosition();
    public abstract void setTargetState(SwerveModuleState swerveModuleState);
    
}
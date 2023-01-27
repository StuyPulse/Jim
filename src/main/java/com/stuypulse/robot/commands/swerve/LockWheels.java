package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockWheels extends CommandBase {
    
    private SwerveDrive swerve;
    
    public LockWheels(SwerveDrive swerve){
        this.swerve = swerve;
        
        addRequirements(swerve);
    }
    
    @Override
    public void execute(){
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        swerve.setModuleStates(state, state, state, state);
    }

}
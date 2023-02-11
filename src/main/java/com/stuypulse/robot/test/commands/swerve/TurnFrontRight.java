package com.stuypulse.robot.test.commands.swerve;

import com.stuypulse.robot.TestRobotContainer;
import com.stuypulse.robot.test.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnFrontRight extends CommandBase{
    private SwerveDrive swerve;

    public TurnFrontRight(SwerveDrive swerve){
    this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        swerve.turnMotorFR(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }
    
}


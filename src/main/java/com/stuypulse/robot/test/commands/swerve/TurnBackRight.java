package com.stuypulse.robot.test.commands.swerve;

import com.stuypulse.robot.TestRobotContainer;
import com.stuypulse.robot.test.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnBackRight extends CommandBase{
    private SwerveDrive swerve;

    public TurnBackRight(SwerveDrive swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        swerve.turnMotorBR(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }
    
}


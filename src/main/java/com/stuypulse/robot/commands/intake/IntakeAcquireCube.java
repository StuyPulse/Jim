package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeAcquireCube extends CommandBase{
    private IIntake intake;

    public IntakeAcquireCube(){
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.acquireCube();
    }
}

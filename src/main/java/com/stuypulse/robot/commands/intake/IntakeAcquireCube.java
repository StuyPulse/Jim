package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeAcquireCube extends CommandBase{
    private Intake intake;

    public IntakeAcquireCube(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.acquireCube();
    }
}

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireCube extends InstantCommand {
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

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeDeacquireCone extends InstantCommand {
    private Intake intake;

    public IntakeDeacquireCone(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.deacquireCone();
    }    
}
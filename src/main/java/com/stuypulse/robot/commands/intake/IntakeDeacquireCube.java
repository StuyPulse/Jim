package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeDeacquireCube extends InstantCommand {
    private IIntake intake;
    public IntakeDeacquireCube() {
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquireCube();
    }
}

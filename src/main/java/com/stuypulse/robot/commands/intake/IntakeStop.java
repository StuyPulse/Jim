package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeStop extends InstantCommand {
    private final IIntake intake;

    public IntakeStop() {
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }
}

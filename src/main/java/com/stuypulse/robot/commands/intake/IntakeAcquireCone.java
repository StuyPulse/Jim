package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireCone extends InstantCommand {
    private IIntake intake;

    public IntakeAcquireCone(){
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.acquireCone();
    }
}

package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireCone extends InstantCommand {
    private Intake intake;

    public IntakeAcquireCone(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.acquireCone();
    }
}

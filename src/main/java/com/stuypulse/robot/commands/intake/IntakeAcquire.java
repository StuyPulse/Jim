package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquire extends InstantCommand {
    private Intake intake;

    public IntakeAcquire(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.acquire();
    }
}

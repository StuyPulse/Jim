package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeScore extends InstantCommand {
    
    private final Intake intake;

    public IntakeScore() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (Manager.getInstance().getGamePiece().isCone()) {
            intake.deacquireCone();
        } else {
            intake.deacquireCube();
        }
    }

}

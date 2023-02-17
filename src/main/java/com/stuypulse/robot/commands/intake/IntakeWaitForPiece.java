package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeWaitForPiece extends CommandBase {
    
    public IntakeWaitForPiece() {
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().hasNewGamePiece();
    }
}

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
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
        var manager = Manager.getInstance();

        if (manager.getNodeLevel() != NodeLevel.LOW && manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {
            return;
        }

        if (manager.getGamePiece().isCone()) {
            intake.deacquireCone();
        } else {
            intake.deacquireCube();
        }
    }

}

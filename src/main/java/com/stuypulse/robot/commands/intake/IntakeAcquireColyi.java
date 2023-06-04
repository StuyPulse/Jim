package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireColyi extends InstantCommand {
    private Intake intake;
    private Manager manager;

    public IntakeAcquireColyi(){
        intake = Intake.getInstance();
        manager = Manager.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        boolean isCubeMid = manager.getScoreSide() == ScoreSide.BACK && manager.getGamePiece().isCube();

        if (manager.getGamePiece().isCone() || isCubeMid) {
            intake.acquire();
        }
    }
}

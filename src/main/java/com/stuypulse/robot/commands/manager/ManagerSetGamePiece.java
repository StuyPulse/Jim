package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetGamePiece extends InstantCommand {

    public ManagerSetGamePiece(GamePiece gamePiece) {
        super(() -> Manager.getInstance().setGamePiece(gamePiece));
    }

}
package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetGamePiece extends InstantCommand {
    private final Manager manager;
    private final GamePiece gamePiece; 

    public ManagerSetGamePiece(GamePiece gamePiece) {
        manager = Manager.getInstance();        
        this.gamePiece = gamePiece;
        // addRequirements(manager); Myles think about this later
    }

    @Override
    public void initialize() {
        manager.setGamePiece(gamePiece);
    }
}
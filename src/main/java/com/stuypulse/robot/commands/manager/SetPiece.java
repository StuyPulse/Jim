package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Piece;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetPiece extends InstantCommand {
    private final Piece piece;
    private final Manager manager;
    
    public SetPiece(Piece piece) {
        this.piece = piece;
        this.manager = Manager.getInstance();
    }

    @Override
    public void initialize() {
        manager.setPiece(piece);
    }
}
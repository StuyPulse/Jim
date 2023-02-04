package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Side;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetSide extends InstantCommand {
    private final Side side;
    private final Manager manager;
    
    public SetSide(Side side) {
        this.side = side;
        this.manager = Manager.getInstance();
    }

    @Override
    public void initialize() {
        manager.setSide(side);
    }
}
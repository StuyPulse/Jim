package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Mode;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetMode extends InstantCommand {
    private final Mode mode;
    private final Manager manager;
    
    public SetMode(Mode mode) {
        this.mode = mode;
        this.manager = Manager.getInstance();
    }

    @Override
    public void initialize() {
        manager.setMode(mode);
    }
}
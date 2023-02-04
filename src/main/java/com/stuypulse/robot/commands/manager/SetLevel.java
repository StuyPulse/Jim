package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetLevel extends InstantCommand {
    private final Level level;
    private final Manager manager;
    
    public SetLevel(Level level) {
        this.level = level;
        this.manager = Manager.getInstance();
    }

    @Override
    public void initialize() {
        manager.setLevel(level);
    }
}
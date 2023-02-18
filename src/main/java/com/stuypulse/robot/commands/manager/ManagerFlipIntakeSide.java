package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerFlipIntakeSide extends InstantCommand {
    public ManagerFlipIntakeSide() {
        super(() -> {
            var manager = Manager.getInstance();
            manager.setIntakeSide(manager.getIntakeSide().getOpposite());
        });
    }
}
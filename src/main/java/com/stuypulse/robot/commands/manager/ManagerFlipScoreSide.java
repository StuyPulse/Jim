package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerFlipScoreSide extends InstantCommand {
    public ManagerFlipScoreSide() {
        super(() -> {
            var manager = Manager.getInstance();
            manager.setScoreSide(manager.getScoreSide().getOpposite());
        });
    }
}
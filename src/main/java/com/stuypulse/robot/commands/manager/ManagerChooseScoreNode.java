package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerChooseScoreNode extends InstantCommand {
    
    public ManagerChooseScoreNode() {
        super(() -> {
            var manager = Manager.getInstance();

            manager.setGridNode(manager.getNearestScoreIndex());
        });
    }

}

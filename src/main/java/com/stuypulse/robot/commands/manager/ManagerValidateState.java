package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerValidateState extends InstantCommand {

    public ManagerValidateState() {
        super(() -> {
            var manager = Manager.getInstance();

            manager.setScoreSide(ScoreSide.BACK);

            if (manager.getNodeLevel() == NodeLevel.LOW) {
                manager.setScoreSide(ScoreSide.FRONT);
            }
        });
    }
}

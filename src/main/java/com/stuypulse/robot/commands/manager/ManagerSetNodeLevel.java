package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetNodeLevel extends InstantCommand {

    public ManagerSetNodeLevel(NodeLevel nodeLevel) {
        super(() -> Manager.getInstance().setNodeLevel(nodeLevel));
    }

}
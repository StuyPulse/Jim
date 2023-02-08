package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetNodeLevel extends InstantCommand {
    private final Manager manager;
    private final NodeLevel nodeLevel; 

    public ManagerSetNodeLevel(NodeLevel nodeLevel) {
        manager = Manager.getInstance();        
        this.nodeLevel = nodeLevel;
        // addRequirements(manager);
    }

    @Override
    public void initialize() {
        manager.setNodeLevel(nodeLevel);
    }
}
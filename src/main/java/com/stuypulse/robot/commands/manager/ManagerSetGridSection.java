package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Direction;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetGridSection extends InstantCommand {
    
    public ManagerSetGridSection(Direction gridSection) {
        super(() -> Manager.getInstance().setGridSection(gridSection));
    }

}

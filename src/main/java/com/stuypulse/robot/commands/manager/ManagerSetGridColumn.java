package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Direction;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetGridColumn extends InstantCommand {

    public ManagerSetGridColumn(Direction gridColumn) {
        super(() -> Manager.getInstance().setGridColumn(gridColumn));
    }

}

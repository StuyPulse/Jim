package com.stuypulse.robot.commands.manager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.Manager;

public class ManagerSetGridNode extends InstantCommand {
    
    public ManagerSetGridNode(int index) {
        super(() -> {
            var manager = Manager.getInstance();

            if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
                manager.setGridNode(index);
            } else {
                manager.setGridNode(8 - index);
            }
        });
    }
}

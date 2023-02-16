package com.stuypulse.robot.commands.manager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Direction;

public class ManagerSetScoreIndex extends InstantCommand {
    
    public ManagerSetScoreIndex(int index) {
        super(() -> {
            var manager = Manager.getInstance();

            Direction[] dirs = 
                // Direction.values(); // is this in the right order?
                { Direction.LEFT, Direction.CENTER, Direction.RIGHT };

            if (DriverStation.getAlliance() == Alliance.Blue) {
                manager.setGridSection(dirs[index / 3]);
                manager.setGridColumn(dirs[index % 3]);
            } else {
                manager.setGridSection(dirs[(8 - index) / 3]);
                manager.setGridColumn(dirs[(8 - index) % 3]);
            }
        });
    }
}

package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerFlipIntakeSide extends InstantCommand {
    public ManagerFlipIntakeSide() {
        super(() -> {
            var manager = Manager.getInstance();
            if (manager.getIntakeSide() == IntakeSide.FRONT) {
                manager.setIntakeSide(IntakeSide.BACK);
            } else {
                manager.setIntakeSide(IntakeSide.FRONT);
            }
        });
    }
}
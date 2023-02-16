package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingsToggleRight extends InstantCommand {
    public WingsToggleRight() {
        super(() -> {
            var wings = Wings.getInstance();
            if (wings.isRightExtended()) {
                wings.retractRight();
            } else {
                wings.extendRight();
            }
        });
    }
}

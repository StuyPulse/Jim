package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingsToggleLeft extends InstantCommand {
    public WingsToggleLeft() {
        super(() -> {
            var wings = Wings.getInstance();
            if (wings.isLeftExtended()) {
                wings.retractLeft();
            } else {
                wings.extendLeft();
            }
        });
    }
}

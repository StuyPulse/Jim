package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingsToggleRed extends InstantCommand {
    public WingsToggleRed() {
        super(() -> {
            var wings = Wings.getInstance();
            if (wings.isRedExtended()) {
                wings.retractRed();
            } else {
                wings.extendRed();
            }
        });
    }
}

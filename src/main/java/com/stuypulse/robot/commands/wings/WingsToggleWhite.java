package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingsToggleWhite extends InstantCommand {
    public WingsToggleWhite() {
        super(() -> {
            var wings = Wings.getInstance();
            if (wings.isWhiteExtended()) {
                wings.retractWhite();
            } else {
                wings.extendWhite();
            }
        });
    }
}

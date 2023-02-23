package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class TeleopButton extends Trigger {

    public TeleopButton(BooleanSupplier pressed) {
        super(() -> DriverStation.isTeleop() && pressed.getAsBoolean());
    }
}
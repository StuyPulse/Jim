package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSetPulseRed extends InstantCommand {
    private double updateTime;
    private LEDController controller;

    public LEDSetPulseRed(double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
    }

    public LEDSetPulseRed() {
        this(Settings.LED.MANUAL_UPDATE_TIME);
    }

    @Override
    public void initialize() {
        controller.forceSetLED(LEDColor.PULSE_RED);
    }
}

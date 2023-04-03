package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.LEDControllerImpl;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSetRainbow extends InstantCommand{
    private double updateTime;
    private LEDController controller;

    public LEDSetRainbow(double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
    }

    public LEDSetRainbow() {
        this(Settings.LED.MANUAL_UPDATE_TIME);
    }

    @Override
    public void initialize() {
        System.out.println("RAINBOW SET WAS CALLED!!!! IT BETTER WORK?");
        controller.forceSetLED(LEDColor.RAINBOW);
        
    }
}

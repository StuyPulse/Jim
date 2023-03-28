/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 */
public class LEDController extends SubsystemBase {

    private static AddressableLED instance;
    private static AddressableLEDBuffer instanceBuffer;

    public static AddressableLED getInstance() {
        return instance;
    }

    // Motor that controlls the LEDs
    private final AddressableLED controller;

    // Stopwatch to check when to start overriding manual updates
    private final StopWatch lastUpdate;
    private double manualTime;

    // The current color to set the LEDs to
    private LEDColor manualColor;

    protected LEDController() {
        instance = new AddressableLED(Ports.LEDController.PORT);
        instanceBuffer = new AddressableLEDBuffer(Ports.LEDController.PORT) // get length of led strip ?
        instance.setLength(instanceBuffer.getLength());

        // set data
        instance.setData(instanceBuffer);
        instance.start();

        this.controller = new PWMSparkMax(Ports.LEDController.PORT);
        this.lastUpdate = new StopWatch();
    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();
    }

    public LEDColor setColor(LEDColor color) {
        // setColor(color, Settings.LED.MANUAL_UPDATE_TIME);
        int red = 0;
        int green = 0;
        int blue = 0;
        
        if (color == LEDColor.PURPLE) {
            red = 102;
            blue = 204;
        }
        
        if (color == LEDColor.YELLOW) {
            red = 255;
            green = 255;
        }
        
        if (color == LEDColor.GREEN) {
            green = 204;
        }

        instance.setData(instanceBuffer);

        return color;
    }

    private void setLEDs() {
        for (int i = 0; i < instanceBuffer.getLength(); i++) {
            instanceBuffer.setRGB(i, red, green, blue);
        }
    }

    private void setLEDConditions() {
    }

    public LEDColor getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: setColor(LEDColor.PURPLE);
            case CONE_TIP_IN: setColor(LEDColor.YELLOW);
            case CONE_TIP_UP: setColor(LEDColor.GREEN);
            case CONE_TIP_OUT: setColor(LEDColor.YELLOW).pulse();
            default: return LEDColor.OFF;
        }
    }

    @Override
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (DriverStation.isAutonomous() || lastUpdate.getTime() < manualTime) {
            controller.set(manualColor.get());
        }

        // Otherwise use the default color
        else {
            controller.set(getDefaultColor().get());
        }
    }
}

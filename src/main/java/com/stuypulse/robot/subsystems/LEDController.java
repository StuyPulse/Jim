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

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 */
public class LEDController extends SubsystemBase {

    private static LEDController instance;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    // Motor that controlls the LEDs
    private final PWMSparkMax controller;

    // Stopwatch to check when to start overriding manual updates
    private final StopWatch lastUpdate;
    private double manualTime;

    // The current color to set the LEDs to
    private LEDColor manualColor;

    protected LEDController() {
        this.controller = new PWMSparkMax(Ports.LEDController.PORT);
        this.lastUpdate = new StopWatch();

        setLEDConditions();
        setColor(LEDColor.OFF);
    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();
    }

    public void setColor(LEDColor color) {
        setColor(color, Settings.LED.MANUAL_UPDATE_TIME);
    }

    private void setLEDConditions() {
    }

    public LEDColor getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: return LEDColor.PURPLE;
            case CONE_TIP_IN: return LEDColor.YELLOW;
            case CONE_TIP_UP: return LEDColor.GREEN;
            case CONE_TIP_OUT: return LEDColor.ORANGE;
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

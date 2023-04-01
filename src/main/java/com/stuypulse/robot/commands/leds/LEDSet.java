/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

/*-
 * Contains:
 *      - commands for LEDs
 * @author Richie Xue
 * @author Jo Walkup
 */

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.util.LEDColor;

import com.stuypulse.robot.constants.Settings;
//import com.stuypulse.robot.constants.Ports.LEDController;
import com.stuypulse.robot.subsystems.leds.*;
import com.stuypulse.robot.subsystems.leds.LEDControllerImpl;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSet extends InstantCommand {

    private LEDColor color;
    private double updateTime;
    private LEDController controller;

    public LEDSet(LEDColor color, double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
        this.color = color;
    }

    public LEDSet(LEDColor color) {
        this(color, Settings.LED.MANUAL_UPDATE_TIME);
    }

    @Override
    public void initialize() {
        controller.setColor(color, updateTime);
    }
}
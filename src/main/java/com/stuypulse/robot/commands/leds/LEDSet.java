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
    private LEDInstruction instruction;

    public LEDSet(LEDColor color, double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
        this.color = color;
    }

    public LEDSet(LEDInstruction instruction, double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
        this.instruction = instruction;
    }

    public LEDSet(LEDColor color) {
        this(color, Settings.LED.MANUAL_UPDATE_TIME);
        //System.out.println("LEDSet was called!!!");
    }

    public LEDSet(LEDInstruction instruction) {
        this(instruction, Settings.LED.MANUAL_UPDATE_TIME);
        //System.out.println("LEDSet was called!!!");
    }

    @Override
    public void initialize() {
        System.out.println("LEDSet was called!!!");
        controller.setColor(color, updateTime);
    }
}
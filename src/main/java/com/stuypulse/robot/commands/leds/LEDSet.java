/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

/*-
 * Contains:
 *      - commands for LEDs
 * @author Richie Xue
 * @author Jo Walkup
 */

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
//import com.stuypulse.robot.constants.Ports.LEDController;
import com.stuypulse.robot.subsystems.leds.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSet extends InstantCommand {

    private double updateTime;
    private LEDController controller;
    private LEDInstruction instruction;

    public LEDSet(LEDInstruction instruction, double updateTime) {
        this.controller = LEDController.getInstance();
        this.updateTime = updateTime;
        this.instruction = instruction;
    }

    public LEDSet(LEDInstruction instruction) {
        this(instruction, Settings.LED.MANUAL_UPDATE_TIME);
    }

    @Override
    public void initialize() {
        controller.forceSetLED(instruction);
    }
}

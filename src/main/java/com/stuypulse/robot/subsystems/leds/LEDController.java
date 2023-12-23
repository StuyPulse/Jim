/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.constants.LEDColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 * @author Richie Xue
 * @author Jo Walkup
 */
public class LEDController extends SubsystemBase {

// singleton
    private static LEDController instance = new LEDController();
    
    public static LEDController getInstance() {
        return instance;
    }

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    public LEDController() {
        leds = new AddressableLED(Ports.LEDController.PORT);
        ledsBuffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH); 

        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();

        SmartDashboard.putData(instance);
    }


    public void forceSetLED(LEDInstruction instruction) {
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }

    public LEDInstruction getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: return LEDColor.RED;
            case CONE_TIP_IN: return LEDColor.YELLOW;
            case CONE_TIP_UP: return LEDColor.GREEN;
            case CONE_TIP_OUT: return LEDColor.ORANGE;
            default: return LEDColor.RED;
        }
    }

    @Override
    public void periodic() {
        if (Robot.getMatchState() == MatchState.TELEOP) {
            forceSetLED(getDefaultColor());
        }
    }
}

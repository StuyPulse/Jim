/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
*  @author Sam Belliveau
 * @author Andrew Liu
 * @author Richie Xue
 * @author Jo Walkup
 */
public class LEDControllerImpl extends LEDController {

    // Motor that controlls the LEDs
    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    public LEDControllerImpl() {
        leds = new AddressableLED(Ports.LEDController.PORT);
        ledsBuffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH); // get length of led strip ?

        // set data
        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();
    }

    @Override
    public void forceSetLED(LEDInstruction instruction) {
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }

}

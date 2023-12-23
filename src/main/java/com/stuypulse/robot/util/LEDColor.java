/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;
import com.stuypulse.robot.subsystems.leds.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.LEDRainbow;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;
  
/**
 * Class that stores all of the different RGB values for the LED Controller.
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 * @author Reya Miller
 * @author Colyi Chen
 * @author Richie Xue
 * @author Jo Walkup
 * @author Naowal Rahman
 * @author Souvik Basak
 */
public class LEDColor implements LEDInstruction {
    private final int red;
    private final int green;
    private final int blue;

    public LEDColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public LEDColor(Color color) {
        this(color.getRed(), color.getGreen(), color.getBlue());
    }

    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }

    public static Color getAWTColor(int red, int green, int blue) {
        return new Color(red, green, blue);
    }
    
    public Color getAWTColor() {
        return new Color(this.red, this.green, this.blue);
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setRGB(i, getRed(), getGreen(), getBlue());
        }
    }


    /***********************/
    /*** COLOR CONSTANTS ***/
    /***********************/

    public static final LEDColor AQUA = new LEDColor(getAWTColor(0, 255, 255));
    public static final LEDColor BLACK = new LEDColor(getAWTColor(0, 0, 0));
    public static final LEDColor BLUE = new LEDColor(getAWTColor(0, 128, 255));
    public static final LEDColor BLUE_GREEN = new LEDColor(getAWTColor(0, 255, 128));
    public static final LEDColor BLUE_VIOLET = new LEDColor(getAWTColor(51, 51, 255));
    public static final LEDColor DARK_BLUE = new LEDColor(getAWTColor(0, 0, 204));
    public static final LEDColor DARK_GRAY = new LEDColor(getAWTColor(64, 64, 64));
    public static final LEDColor DARK_GREEN = new LEDColor(getAWTColor(0, 153, 0));
    public static final LEDColor DARK_RED = new LEDColor(getAWTColor(204, 0, 0));
    public static final LEDColor GOLD = new LEDColor(getAWTColor(218, 165, 32));
    public static final LEDColor GRAY = new LEDColor(getAWTColor(128, 128, 128));
    public static final LEDColor GREEN = new LEDColor(getAWTColor(0, 255, 0));
    public static final LEDColor HOT_PINK = new LEDColor(getAWTColor(255, 105, 180));
    public static final LEDColor LAWN_GREEN = new LEDColor(getAWTColor(102, 204, 0));
    public static final LEDColor LIME = new LEDColor(getAWTColor(191, 255, 0));
    public static final LEDColor ORANGE = new LEDColor(getAWTColor(255, 128, 0));
    public static final LEDColor PINK = new LEDColor(getAWTColor(255, 192, 203));
    public static final LEDColor PURPLE = new LEDColor(getAWTColor(160, 32, 240));
    public static final LEDColor RED = new LEDColor(getAWTColor(255, 0 , 0));
    public static final LEDColor RED_ORANGE = new LEDColor(getAWTColor(255, 83, 73));
    public static final LEDColor VIOLET = new LEDColor(getAWTColor(127, 0, 255));
    public static final LEDColor WHITE = new LEDColor(getAWTColor(255, 255, 255));
    public static final LEDColor YELLOW = new LEDColor(getAWTColor(255, 255, 0));

    public static final LEDColor OFF = new LEDColor(getAWTColor(0, 0, 0));

    public static final LEDInstruction RAINBOW = new LEDRainbow();
    // public static final LEDInstruction PULSE_RED = new LEDPulseColor(RED.getAWTColor());
    // public static final LEDInstruction PULSE_RED_BLUE = new LEDPulseColor(RED.getAWTColor(), BLUE.getAWTColor());
    // public static final LEDInstruction RICHIE = new RichieMode(RED.getAWTColor());
    // public static final LEDInstruction BANGLADESH = new LEDSection(new Color[] {RED.getAWTColor(), BLACK.getAWTColor(), DARK_GREEN.getAWTColor()});   
}

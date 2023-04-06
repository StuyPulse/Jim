/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.util;
import com.stuypulse.robot.subsystems.leds.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.LEDPulseColor;
import com.stuypulse.robot.subsystems.leds.LEDRainbow;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
  
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

    public LEDColor(SLColor color) {
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

    public SLColor getSLColor() {
        return new SLColor(red, green, blue);
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

    public static final LEDColor AQUA = new LEDColor(new SLColor(0, 255, 255));
    public static final LEDColor BLACK = new LEDColor(new SLColor(0, 0, 0));
    public static final LEDColor BLUE = new LEDColor(new SLColor(0, 128, 255));
    public static final LEDColor BLUE_GREEN = new LEDColor(new SLColor(0, 255, 128));
    public static final LEDColor BLUE_VIOLET = new LEDColor(new SLColor(51, 51, 255));
    public static final LEDColor DARK_BLUE = new LEDColor(new SLColor(0, 0, 204));
    public static final LEDColor DARK_GRAY = new LEDColor(new SLColor(64, 64, 64));
    public static final LEDColor DARK_GREEN = new LEDColor(new SLColor(0, 153, 0));
    public static final LEDColor DARK_RED = new LEDColor(new SLColor(204, 0, 0));
    public static final LEDColor GOLD = new LEDColor(new SLColor(218,165,32));
    public static final LEDColor GRAY = new LEDColor(new SLColor(128, 128, 128));
    public static final LEDColor GREEN = new LEDColor(new SLColor(0, 255, 0));
    public static final LEDColor HOT_PINK = new LEDColor(new SLColor(255, 105, 180));
    public static final LEDColor LAWN_GREEN = new LEDColor(new SLColor(102, 204, 0));
    public static final LEDColor LIME = new LEDColor(new SLColor(191, 255, 0));
    public static final LEDColor ORANGE = new LEDColor(new SLColor(255, 128, 0));
    public static final LEDColor PINK = new LEDColor(new SLColor(255, 192, 203));
    public static final LEDColor PURPLE = new LEDColor(new SLColor(160, 32, 240));
    public static final LEDColor RED = new LEDColor(new SLColor(255, 0 , 0));
    public static final LEDColor RED_ORANGE = new LEDColor(new SLColor(255, 83, 73));
    public static final LEDColor VIOLET = new LEDColor(new SLColor(127, 0, 255));
    public static final LEDColor WHITE = new LEDColor(new SLColor(255, 255, 255));
    public static final LEDColor YELLOW = new LEDColor(new SLColor(255, 255, 0));

    public static final LEDColor OFF = new LEDColor(new SLColor(0, 0, 0));

    public static final LEDInstruction RAINBOW = new LEDRainbow();
    public static final LEDInstruction PULSE_RED = new LEDPulseColor(RED.getSLColor());
    public static final LEDInstruction PULSE_RED_BLUE = new LEDPulseColor(RED.getSLColor(), BLUE.getSLColor());
    
    
}

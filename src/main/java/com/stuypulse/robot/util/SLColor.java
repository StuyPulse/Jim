/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.awt.Color;
// class is also importing edu.wpi.first.wpilibj.util.Color8 implicitly 

import com.stuypulse.stuylib.math.SLMath;
  
/**
 * StuyLib wrapper class for colors that handles various Color classes used in FRC (java.awt.Color, wpilibj.util.Color, wpilibj.util.Color8Bit)
 * @author Richie Xue
 */
public class SLColor {

    private final int red;
    private final int green;
    private final int blue;

    /**
     * Create an LEDColor object using RGB values
     *
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
    */
    public SLColor(int red, int green, int blue) {
        this.red = (int) SLMath.clamp(red, 0, 255);
        this.green = (int) SLMath.clamp(green, 0, 255);
        this.blue = (int) SLMath.clamp(blue, 0, 255);
    }

    /**
    * Create an LEDColor object using java.awt.Color
     *
     * @param color The java.awt.Color object
    */
    public SLColor(Color color) {
        this(color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
    * Create an LEDColor object using edu.wpi.first.wpilibj.util.Color
     *
     * @param color The edu.wpi.first.wpilibj.util.Color object
    */
    public SLColor(edu.wpi.first.wpilibj.util.Color color) {
        this((int) color.red * 255, (int) color.green * 255, (int) color.green * 255);
    }

    /**
    * Create an LEDColor object using edu.wpi.first.wpilibj.util.Color8Bit
     *
     * @param color The edu.wpi.first.wpilibj.util.Color8Bit object
    */
    public SLColor(Color8Bit color) {
        this(color.red, color.blue, color.green);
    }

    /**
     * Gets the r value [0-255]
     *
     * @return the LED color's red value
     */
    public int getRed() {
        return red;
    }

    /**
     * Gets the g value [0-255]
     *
     * @return the LED color's green value
     */
    public int getGreen() {
        return green;
    }

    /**
     * Gets the b value [0-255]
     *
     * @return the LED color's blue value
     */
    public int getBlue() {
        return blue;
    }

    /* Getters to convert LEDColor objects into other Color types */
 
    public Color getAWTColor() {
        return new Color(this.red, this.green, this.blue);
    }

    public edu.wpi.first.wpilibj.util.Color getWPILibColor() {
        return new edu.wpi.first.wpilibj.util.Color(this.red / 255.0, this.green / 255.0, this.blue / 255.0);
    }

    public Color8Bit getColor8Bit() {
        return new Color8Bit(red, green, blue);
    }

    /***********************/
    /*** COLOR CONSTANTS ***/
    /***********************/

    public static final SLColor AQUA = new SLColor(0, 255, 255);
    public static final SLColor BLACK = new SLColor(0, 0, 0);
    public static final SLColor BLUE = new SLColor(0, 128, 255);
    public static final SLColor BLUE_GREEN = new SLColor(0, 255, 128);
    public static final SLColor BLUE_VIOLET = new SLColor(51, 51, 255);
    public static final SLColor DARK_BLUE = new SLColor(0, 0, 204);
    public static final SLColor DARK_GRAY = new SLColor(64, 64, 64);
    public static final SLColor DARK_GREEN = new SLColor(0, 153, 0);
    public static final SLColor DARK_RED = new SLColor(204, 0, 0);
    public static final SLColor GOLD = new SLColor(218, 165, 32);
    public static final SLColor GRAY = new SLColor(128, 128, 128);
    public static final SLColor GREEN = new SLColor(0, 255, 0);
    public static final SLColor HOT_PINK = new SLColor(255, 105, 180);
    public static final SLColor LAWN_GREEN = new SLColor(102, 204, 0);
    public static final SLColor LIME = new SLColor(191, 255, 0);
    public static final SLColor ORANGE = new SLColor(255, 128, 0);
    public static final SLColor PINK = new SLColor(255, 192, 203);
    public static final SLColor PURPLE = new SLColor(160, 32, 240);
    public static final SLColor RED = new SLColor(255, 0 , 0);
    public static final SLColor RED_ORANGE = new SLColor(255, 83, 73);
    public static final SLColor VIOLET = new SLColor(127, 0, 255);
    public static final SLColor WHITE = new SLColor(255, 255, 255);
    public static final SLColor YELLOW = new SLColor(255, 255, 0);

}

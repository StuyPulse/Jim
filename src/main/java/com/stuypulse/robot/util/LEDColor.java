/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;



/**
 * Class that stores all of the different RGB values for the LED Controller.
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 * @author Reya Miller
 * @author Colyi Chen
 */
public class LEDColor {
    private final int red;
    private final int green;
    private final int blue;

    private LEDColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
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

    public LEDColor setColor(int red, int green, int blue) {
        return new LEDColor(red, green, blue);
    }

    /***********************/
    /*** COLOR CONSTANTS ***/
    /***********************/


    public static final LEDColor AQUA = new LEDColor(0, 255, 255);
    public static final LEDColor BLACK = new LEDColor(0, 0, 0);
    public static final LEDColor BLUE = new LEDColor(0, 128, 255);
    public static final LEDColor BLUE_GREEN = new LEDColor(0, 255, 128);
    public static final LEDColor BLUE_VIOLET = new LEDColor(51, 51, 255);
    public static final LEDColor DARK_BLUE = new LEDColor(0, 0, 204);
    public static final LEDColor DARK_GRAY = new LEDColor(64, 64, 64);
    public static final LEDColor DARK_GREEN = new LEDColor(0, 153, 0);
    public static final LEDColor DARK_RED = new LEDColor(204, 0, 0);
    public static final LEDColor GOLD = new LEDColor(218,165,32);
    public static final LEDColor GRAY = new LEDColor(128, 128, 128);
    public static final LEDColor GREEN = new LEDColor(0, 255, 0);
    public static final LEDColor HOT_PINK = new LEDColor(255, 105, 180);
    public static final LEDColor LAWN_GREEN = new LEDColor(102, 204, 0);
    public static final LEDColor LIME = new LEDColor(191, 255, 0);
    public static final LEDColor ORANGE = new LEDColor(255, 128, 0);
    public static final LEDColor PINK = new LEDColor(255, 192, 203);
    public static final LEDColor PURPLE = new LEDColor(160, 32, 240);
    public static final LEDColor RED = new LEDColor(255, 0 , 0);
    public static final LEDColor RED_ORANGE = new LEDColor(255, 83, 73);
    public static final LEDColor VIOLET = new LEDColor(127, 0, 255);
    public static final LEDColor WHITE = new LEDColor(255, 255, 255);
    public static final LEDColor YELLOW = new LEDColor(255, 255, 0);

    public static final LEDColor OFF = new LEDColor(0, 0, 0);
    public static final LEDColor RAINBOW = OFF;


}

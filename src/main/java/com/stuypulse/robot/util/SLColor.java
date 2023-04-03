package com.stuypulse.robot.util;

public class SLColor {
    private final int red;
    private final int green;
    private final int blue; 

    public SLColor(int red, int green, int blue) {
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

    public static final SLColor AQUA = new SLColor(0, 255, 255);
    public static final SLColor BLACK = new SLColor(0, 0, 0);
    public static final SLColor BLUE = new SLColor(0, 128, 255);
    public static final SLColor BLUE_GREEN = new SLColor(0, 255, 128);
    public static final SLColor BLUE_VIOLET = new SLColor(51, 51, 255);
    public static final SLColor DARK_BLUE = new SLColor(0, 0, 204);
    public static final SLColor DARK_GRAY = new SLColor(64, 64, 64);
    public static final SLColor DARK_GREEN = new SLColor(0, 153, 0);
    public static final SLColor DARK_RED = new SLColor(204, 0, 0);
    public static final SLColor GOLD = new SLColor(218,165,32);
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

    public static final SLColor OFF = new SLColor(0, 0, 0);
    public static final SLColor RAINBOW = new SLColor(0, 0, 0);
}

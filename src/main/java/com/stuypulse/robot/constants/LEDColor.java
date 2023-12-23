package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.leds.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.LEDPulseColor;
import com.stuypulse.robot.subsystems.leds.LEDRainbow;
import com.stuypulse.robot.subsystems.leds.LEDSection;
import com.stuypulse.robot.subsystems.leds.LEDSingleColor;
import com.stuypulse.robot.subsystems.leds.RichieMode;
import com.stuypulse.robot.util.SLColor; 

public interface LEDColor {
    /***********************/
    /*** COLOR CONSTANTS ***/
    /***********************/

    public static final LEDInstruction AQUA = new LEDSingleColor(new SLColor(0, 255, 255));
    public static final LEDInstruction BLACK = new LEDSingleColor(new SLColor(0, 0, 0));
    public static final LEDInstruction BLUE = new LEDSingleColor(new SLColor(0, 128, 255));
    public static final LEDInstruction BLUE_GREEN = new LEDSingleColor(new SLColor(0, 255, 128));
    public static final LEDInstruction BLUE_VIOLET = new LEDSingleColor(new SLColor(51, 51, 255));
    public static final LEDInstruction DARK_BLUE = new LEDSingleColor(new SLColor(0, 0, 204));
    public static final LEDInstruction DARK_GRAY = new LEDSingleColor(new SLColor(64, 64, 64));
    public static final LEDInstruction DARK_GREEN = new LEDSingleColor(new SLColor(0, 153, 0));
    public static final LEDInstruction DARK_RED = new LEDSingleColor(new SLColor(204, 0, 0));
    public static final LEDInstruction GOLD = new LEDSingleColor(new SLColor(218, 165, 32));
    public static final LEDInstruction GRAY = new LEDSingleColor(new SLColor(128, 128, 128));
    public static final LEDInstruction GREEN = new LEDSingleColor(new SLColor(0, 255, 0));
    public static final LEDInstruction HOT_PINK = new LEDSingleColor(new SLColor(255, 105, 180));
    public static final LEDInstruction LAWN_GREEN = new LEDSingleColor(new SLColor(102, 204, 0));
    public static final LEDInstruction LIME = new LEDSingleColor(new SLColor(191, 255, 0));
    public static final LEDInstruction ORANGE = new LEDSingleColor(new SLColor(255, 128, 0));
    public static final LEDInstruction PINK = new LEDSingleColor(new SLColor(255, 192, 203));
    public static final LEDInstruction PURPLE = new LEDSingleColor(new SLColor(160, 32, 240));
    public static final LEDInstruction RED = new LEDSingleColor(new SLColor(255, 0 , 0));
    public static final LEDInstruction RED_ORANGE = new LEDSingleColor(new SLColor(255, 83, 73));
    public static final LEDInstruction VIOLET = new LEDSingleColor(new SLColor(127, 0, 255));
    public static final LEDInstruction WHITE = new LEDSingleColor(new SLColor(255, 255, 255));
    public static final LEDInstruction YELLOW = new LEDSingleColor(new SLColor(255, 255, 0));

    public static final LEDInstruction OFF = new LEDSingleColor(new SLColor(0, 0, 0));

    public static final LEDInstruction RAINBOW = new LEDRainbow();
    public static final LEDInstruction PULSE_RED = new LEDPulseColor(SLColor.RED);
    public static final LEDInstruction PULSE_RED_BLUE = new LEDPulseColor(SLColor.RED, SLColor.BLUE);
    public static final LEDInstruction RICHIE = new RichieMode(SLColor.RED);
    public static final LEDInstruction BANGLADESH = new LEDSection(new SLColor[] {SLColor.RED, SLColor.BLACK, SLColor.DARK_GREEN});
    
}

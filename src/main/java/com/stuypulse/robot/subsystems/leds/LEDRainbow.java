package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
/*-
 * Contains:
 *      - setLED() : sets LEDs to rainbow colors
 * @author Richie Xue
 * @author Jo Walkup
 */
public class LEDRainbow implements LEDInstruction {
    StopWatch timer;
    int counter;
    
    public LEDRainbow() {
        timer = new StopWatch();
        counter = 0;
    }     

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        LEDColor[] rainbow = new LEDColor[]{LEDColor.RED, LEDColor.RED_ORANGE, LEDColor.ORANGE, LEDColor.YELLOW, LEDColor.LAWN_GREEN, LEDColor.GREEN, LEDColor.DARK_GREEN, LEDColor.BLUE_GREEN, LEDColor.BLUE, LEDColor.DARK_BLUE, LEDColor.BLUE_VIOLET, LEDColor.VIOLET, LEDColor.PURPLE, LEDColor.PINK};
        LEDColor color;
        // if(timer.getTime() > 3 * Math.sin(colorCounter * 2.0) + 1) {
        if(timer.getTime() > 1.0) {
            timer.reset();
            for(int i = 0; i < ledsBuffer.getLength(); i++) {
                color = rainbow[(i + counter) % rainbow.length];
                ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
                ++counter;
                if(counter == rainbow.length) counter = 0;
            }
        }
    }
}
package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;
/*-
 * Contains:
 *      - setLED() : sets LEDs to rainbow colors
 * @author Richie Xue
 * @author Jo Walkup
 * @author Naowal Rahman
 */
public class LEDRainbow implements LEDInstruction {
    StopWatch timer;
    int counter;

    private static Color[] rainbow = new Color[]{
        new Color(218,165,32), 
        new Color(255, 83, 73), 
        new Color(255, 128, 0),
        new Color(255, 255, 0), 
        new Color(102, 204, 0), 
        new Color(0, 255, 0), 
        new Color(0, 153, 0), 
        new Color(0, 255, 128), 
        new Color(0, 128, 255), 
        new Color(0, 0, 204), 
        new Color(51, 51, 255), 
        new Color(127, 0, 255), 
        new Color(160, 32, 240), 
        new Color(255, 192, 203)
    };

    public LEDRainbow() {
        timer = new StopWatch();
        counter = 0;
    }     

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {

        // if(timer.getTime() > 3 * Math.sin(colorCounter * 2.0) + 1) {
        Color color;
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
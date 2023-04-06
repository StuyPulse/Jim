package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.SLColor;
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
        SLColor[] rainbow = new SLColor[]{new SLColor (218,165,32), 
                                        new SLColor(255, 83, 73), 
                                        new SLColor(255, 128, 0),
                                        new SLColor(255, 255, 0), 
                                        new SLColor(102, 204, 0), 
                                        new SLColor(0, 255, 0), 
                                        new SLColor(0, 153, 0), 
                                        new SLColor(0, 255, 128), 
                                        new SLColor(0, 128, 255), 
                                        new SLColor(0, 0, 204), 
                                        new SLColor(51, 51, 255), 
                                        new SLColor(127, 0, 255), 
                                        new SLColor(160, 32, 240), 
                                        new SLColor(255, 192, 203)};

        // if(timer.getTime() > 3 * Math.sin(colorCounter * 2.0) + 1) {
        if(timer.getTime() > 1.0) {
            timer.reset();
            for(int i = 0; i < ledsBuffer.getLength(); i++) {
                SLColor color = rainbow[(i + counter) % rainbow.length];
                ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
                ++counter;
                if(counter == rainbow.length) counter = 0;
            }
        }
    }
}
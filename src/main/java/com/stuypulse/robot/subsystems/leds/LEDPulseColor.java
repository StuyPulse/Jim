package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Class that sets colour of pulsing LEDS on LEDController
 *
 * @author Richie Xue 
 * @author Andrew Liu
 * @author Reya Miller
 * @author Ian Shi 
 * @author Colyi Chen
 * @author Jo Walkup
 */

public class LEDPulseColor implements LEDInstruction {
    public SLColor color;
    public SLColor altcolor; 
    public StopWatch stopwatch;

    public LEDPulseColor(SLColor color) {
        this.color = color;
        stopwatch = new StopWatch();
    }

    public LEDPulseColor(SLColor color1, SLColor color2) {
        this.color = color1;
        this.altcolor = color2;
        stopwatch = new StopWatch();
        
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        if (stopwatch.getTime() < 0.5) {
            for (int i = 0; i < ledsBuffer.getLength(); i++) {
                ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
            }
        }
        else if (stopwatch.getTime() < 1){
            if (altcolor != null) {
                for (int i = 0; i < ledsBuffer.getLength(); i++) {
                    ledsBuffer.setRGB(i, altcolor.getRed(), altcolor.getGreen(), altcolor.getBlue());
                }
            }
            else {
                for (int i = 0; i < ledsBuffer.getLength(); i++) {
                    ledsBuffer.setRGB(i, 0, 0, 0);
                }
            }
        }
        else {
            stopwatch.reset();
        }
    }

}


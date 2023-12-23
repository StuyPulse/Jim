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

    public LEDPulseColor(SLColor color1, SLColor color2) {
        this.color = color1;
        this.altcolor = color2;
        stopwatch = new StopWatch();    
    }

    public LEDPulseColor(SLColor color) {
        this(color, SLColor.BLACK);
    }


    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        double time = stopwatch.getTime();

        if (time < 0.5) {
            for (int i = 0; i < ledsBuffer.getLength(); i++) {
                ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
            }
        }

        else if (time < 1){
            for (int i = 0; i < ledsBuffer.getLength(); i++) {
                ledsBuffer.setRGB(i, altcolor.getRed(), altcolor.getGreen(), altcolor.getBlue());
            }
        } 
        
        else {
            stopwatch.reset();
        }
    }

}


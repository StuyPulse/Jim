package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;

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
    public Color color;
    public Color altcolor; 
    public StopWatch stopwatch;

    public LEDPulseColor(Color color) {
        this(color, new Color(0,0,0));
    }

    public LEDPulseColor(Color color1, Color color2) {
        this.color = color1;
        this.altcolor = color2;
        stopwatch = new StopWatch();
        
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


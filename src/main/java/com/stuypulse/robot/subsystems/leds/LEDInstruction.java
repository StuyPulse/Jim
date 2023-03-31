package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// @author Richie Xue
// @author Jo Walkup

public interface LEDInstruction {
    void setLED(AddressableLEDBuffer ledsBuffer);
    
}

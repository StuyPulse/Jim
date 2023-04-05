/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.LEDColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 * @author Richie Xue
 * @author Jo Walkup
 */
public abstract class LEDController extends SubsystemBase {

// singleton
    private static LEDController instance;
    
    static {
        instance = new LEDControllerImpl();
    }

    public static LEDController getInstance() {
        return instance;
    }

    // Stopwatch to check when to start overriding manual updates
    private final StopWatch lastUpdate;
    private double manualTime;

    // The current color to set the LEDs to
    private LEDColor manualColor;

    public LEDController() {
        this.lastUpdate = new StopWatch();
    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();
    }

    public abstract void forceSetLED(LEDInstruction instruction);

    public void setLEDConditions() {
    }

    public LEDInstruction getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: return LEDColor.PURPLE;
            case CONE_TIP_IN: return LEDColor.YELLOW;
            case CONE_TIP_UP: return LEDColor.GREEN;
            case CONE_TIP_OUT: return LEDColor.ORANGE;
            default: return LEDColor.RED;
        }
    }

    @Override
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (Robot.getMatchState() == MatchState.AUTO || lastUpdate.getTime() < manualTime) {
            forceSetLED(manualColor);
        }

        // Otherwise use the default color
        else {
            forceSetLED(getDefaultColor());
        }
    }
}

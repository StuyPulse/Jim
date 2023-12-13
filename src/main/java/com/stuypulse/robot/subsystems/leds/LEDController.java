/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.leds;

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

    // LEDVisualizer
    LEDVisualizer visualizer = new LEDVisualizer();

    public abstract void forceSetLED(LEDInstruction instruction);

    public LEDInstruction getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: return LEDColor.PULSE_RED_BLUE;
            case CONE_TIP_IN: return LEDColor.YELLOW;
            case CONE_TIP_UP: return LEDColor.GREEN;
            case CONE_TIP_OUT: return LEDColor.ORANGE;
            default: return LEDColor.RED;
        }
    }

    @Override
    public void periodic() {
        if (Robot.getMatchState() == MatchState.TELEOP) {
            forceSetLED(getDefaultColor());
        }
        visualizer.setColor(LEDColor.BLUE, 10);
    }
}

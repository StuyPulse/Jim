/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;

/**
 * A feedforward term to account for gravity for motorized arms that can move continuously (if not
 * use `ArmFeedforward`)
 *
 * <p>The motor feedforward used in the context of an arm will not account for gravity that is
 * acting on the arm.
 *
 * <p>Can be paired with MotorFeedforward or other controllers with .add
 *
 * @author Benjamin Irving Goldfisher XXII (myles.pasetsky@gmail.com)
 */
public class ArmEncoderFeedforward extends Controller {

    /** voltage to hold arm horizontal */
    private final Number kG;

    /**
     * Create arm feedforward
     *
     * @param kG term to hold arm vertical against gravity (volts)
     */
    public ArmEncoderFeedforward(Number kG) {
        this.kG = kG;
    }

    /**
     * Calculates voltage to hold arm at the setpoint angle
     *
     * @param setpoint setpoint
     * @param measurement measurement
     * @return kG * cos(setpoint)
     */
    @Override
    protected double calculate(double setpoint, double measurement) {
        return kG.doubleValue() * Math.cos(measurement);
    }
}

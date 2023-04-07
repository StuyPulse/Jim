/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Pumps the robot full of air
 *
 * Contains:
 *      - network boolean for controlling state of compressor
 *      - compressor pneumatics module
 *
 * @author Myles Pasetsky
 * @author SE
 */
public class Pump extends SubsystemBase {

    // Singleton
    private static final Pump instance;

    static {
        instance = new Pump();
    }

    public static final Pump getInstance() {
        return instance;
    }

    // Pump control & hardware
    private final SmartBoolean enabled;
    private final Compressor compressor;

    protected Pump() {
        enabled = new SmartBoolean("Pump/Compressor Enabled", true);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);

        // stop();
    }

    public boolean getCompressing() {
        return compressor.isEnabled();
    }

    // Start Compressing the Robot
    public void compress() {
        this.set(true);
    }

    // Stop Compressing
    public void stop() {
        this.set(false);
    }

    // Set the compressor to on or off
    public void set(boolean compressing) {
        enabled.set(compressing);
    }

    @Override
    public void periodic() {
        if (enabled.get()) {
            compressor.enableDigital();
        } else {
            compressor.disable();
        }
    }
}

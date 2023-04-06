/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.test;

import static com.stuypulse.robot.constants.Ports.Plant.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
* @author Samuel Chen
* @author Carmin Vuong
* @author Jiayu Yan
* @author Tracey Lin
*
*/
public class TestPlant extends SubsystemBase {

    private final DoubleSolenoid solenoid;

    public TestPlant() {
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD, REVERSE);
        disengage();
    }

    public void engage() {
        solenoid.set(Value.kReverse);
    }

    public void disengage() {
        solenoid.set(Value.kForward);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Engaged", solenoid.get()==Value.kReverse);
    }
}

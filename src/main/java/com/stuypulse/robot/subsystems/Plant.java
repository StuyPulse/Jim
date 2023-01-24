package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
* @author Samuel Chen
* @author Carmin Vuong
* @author Jiayu Yan
* @author Tracey Lin
*
*/
public class Plant extends SubsystemBase{
    private final DoubleSolenoid solenoid;

    public Plant() {
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Plant.FORWARD, Ports.Plant.REVERSE);

    }

    public void engage() {
        solenoid.set(Value.kForward);
    }

    public void disengage() {
        solenoid.set(Value.kReverse);
    }
    
    public void stop() {
        solenoid.set(Value.kOff);
    }
}

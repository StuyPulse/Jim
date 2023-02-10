package com.stuypulse.robot.test.subsystems.plant;

import static com.stuypulse.robot.constants.Ports.Plant.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD, REVERSE);
        disengage();
    }

    public void engage() {
        solenoid.set(Value.kForward);
    }

    public void disengage() {
        solenoid.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Engaged", solenoid.get()==Value.kForward);
    }
}


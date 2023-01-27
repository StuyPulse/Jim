package com.stuypulse.robot.subsystems.plant;

import static com.stuypulse.robot.constants.Ports.Plant.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
* @author Samuel Chen
* @author Carmin Vuong
* @author Jiayu Yan
* @author Tracey Lin
*
*/
public class Plant extends IPlant {
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

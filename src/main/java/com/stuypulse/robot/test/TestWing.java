package com.stuypulse.robot.test;


import static com.stuypulse.robot.constants.Ports.Wings.*;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestWing extends SubsystemBase {

    // left solenoids
    private final DoubleSolenoid deploy;
    private final Solenoid latch;

    public TestWing() {
        deploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DEPLOY_FORWARD, DEPLOY_REVERSE);
        latch = new Solenoid(PneumaticsModuleType.CTREPCM, LATCH);

        latch.set(true);

        deploy.set(DoubleSolenoid.Value.kForward);
    }

    public void extendLatch() {
        latch.set(true);
    }

    public void retractLatch() {
        latch.set(false);
    }

    public void extendDeploy() {
        deploy.set(DoubleSolenoid.Value.kReverse);
    }

    public void retractDeploy() {
        deploy.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wings/Latch Engaged", latch.get());
        SmartDashboard.putBoolean("Wings/Deployed", deploy.get() == DoubleSolenoid.Value.kReverse);

    }
}
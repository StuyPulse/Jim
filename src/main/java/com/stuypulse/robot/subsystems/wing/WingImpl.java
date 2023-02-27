package com.stuypulse.robot.subsystems.wing;

import static com.stuypulse.robot.constants.Settings.Wings.*;

import com.stuypulse.robot.constants.Settings;

import static com.stuypulse.robot.constants.Ports.Wings.*;
import com.stuypulse.stuylib.util.StopWatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class WingImpl extends Wing {

    private final StopWatch timer;

    private double deployTime;
    private double retractTime;

    // solenoids
    private final DoubleSolenoid deploy;
    private final Solenoid latch;

    public WingImpl() {
        timer = new StopWatch();

        deploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DEPLOY_FORWARD, DEPLOY_REVERSE);
        latch = new Solenoid(PneumaticsModuleType.CTREPCM, LATCH);
        deployTime = -1.0;
        retractTime = -1.0;

        setLatched(true);

        deploy.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void extend() {
        if (isLatched() && !isExtended()) {
            setLatched(false);
            deployTime = timer.getTime();
        }
    }

    @Override
    public void retract() {
        if (!isLatched() && isExtended()) {
            deploy.set(DoubleSolenoid.Value.kForward);
            retractTime = timer.getTime();
        }
    }

    @Override
    public boolean isExtended(){
        return deploy.get() == DoubleSolenoid.Value.kReverse;
    }

    private boolean isLatched() {
        return !latch.get();
    }

    private void setLatched(boolean latched) {
        latch.set(!latched);
    }

    @Override
    public void periodic() {
        if (deployTime > 0 && timer.getTime() - deployTime >= LATCH_DELAY.get()){
            deploy.set(DoubleSolenoid.Value.kReverse); // dont set off
            deployTime = -1.0;
        }
        if (retractTime > 0 && timer.getTime() - retractTime >= RETRACT_DELAY.get()){
            setLatched(true);
            retractTime = -1.0;
        }

        if (Settings.isDebug()) {
            Settings.putBoolean("Wings/Latch Engaged", isLatched());
            Settings.putBoolean("Wings/Deploy Extended", isExtended());

            Settings.putNumber("Wings/Current Time", timer.getTime());
            Settings.putNumber("Wings/Deploy Time", deployTime);
            Settings.putNumber("Wings/Retract Time", retractTime);
        }
    }
}
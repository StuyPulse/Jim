package com.stuypulse.robot.subsystems.wings;

import static com.stuypulse.robot.constants.Settings.Wings.*;

import com.stuypulse.robot.constants.Settings;

import static com.stuypulse.robot.constants.Ports.Wings.*;
import com.stuypulse.stuylib.util.StopWatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WingsImpl extends Wings {

    private final StopWatch timer;

    private double redDeployTime;
    private double whiteDeployTime;
    private double redRetractTime;
    private double whiteRetractTime;

    // red solenoids
    private final DoubleSolenoid redDeploy;
    private final Solenoid redLatch;

    // white solenoids
    private final DoubleSolenoid whiteDeploy;
    private final Solenoid whiteLatch;

    public WingsImpl() {
        timer = new StopWatch();

        redDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RED_DEPLOY_FORWARD, RED_DEPLOY_REVERSE);
        redLatch = new Solenoid(PneumaticsModuleType.CTREPCM, RED_LATCH);
        redDeployTime = -1.0;
        redRetractTime = -1.0;
        
        whiteDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WHITE_DEPLOY_FORWARD, WHITE_DEPLOY_REVERSE);
        whiteLatch = new Solenoid(PneumaticsModuleType.CTREPCM, WHITE_LATCH);
        whiteDeployTime = -1.0;
        whiteRetractTime = -1.0;

        redLatch.set(true);
        whiteLatch.set(true);

        whiteDeploy.set(DoubleSolenoid.Value.kReverse);
        redDeploy.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void extendRed() {
        if (redLatch.get() && !isEngaged(redDeploy)) {
            redLatch.set(false);
            redDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractRed() {
        if (!redLatch.get() && isEngaged(redDeploy)) {
            redDeploy.set(DoubleSolenoid.Value.kReverse);
            redRetractTime = timer.getTime();
        }
    }

    @Override
    public void extendWhite() {
        if(whiteLatch.get() && !isEngaged(whiteDeploy)){
            whiteLatch.set(false);
            whiteDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractWhite() {
        if(!whiteLatch.get() && isEngaged(whiteDeploy)){
            whiteDeploy.set(DoubleSolenoid.Value.kReverse);
            whiteRetractTime = timer.getTime();
        }
    }

    public boolean isEngaged(DoubleSolenoid solenoid){
        if(solenoid.get() == DoubleSolenoid.Value.kForward){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void periodic() {
        if(redDeployTime > 0 && timer.getTime() - redDeployTime >= RED_LATCH_DELAY.get()){
            redDeploy.set(DoubleSolenoid.Value.kForward); // dont set off
            redDeployTime = -1.0;
        }
        if(whiteDeployTime > 0 && timer.getTime() - whiteDeployTime >= WHITE_LATCH_DELAY.get()){
            whiteDeploy.set(DoubleSolenoid.Value.kForward);
            whiteDeployTime = -1.0;
        }
        if(redRetractTime > 0 && timer.getTime() - redRetractTime >= RED_RETRACT_DELAY.get()){
            redLatch.set(true);
            redRetractTime = -1.0;
        }
        if(whiteRetractTime > 0 && timer.getTime() - whiteRetractTime >= WHITE_RETRACT_DELAY.get()){
            whiteLatch.set(true);
            whiteRetractTime = -1.0;
        }


        if (Settings.isDebug()) {
            Settings.putBoolean("Wings/White Latch Engaged", whiteLatch.get());
            Settings.putBoolean("Wings/White Deploy Engaged", isEngaged(whiteDeploy));
            Settings.putBoolean("Wings/Red Latch Engaged", redLatch.get());
            Settings.putBoolean("Wings/Red Deploy Engaged", isEngaged(redDeploy));

            Settings.putNumber("Wings/Current Time", timer.getTime());
            Settings.putNumber("Wings/Red Deploy Time", redDeployTime);
            Settings.putNumber("Wings/White Deploy Time", whiteDeployTime);
            Settings.putNumber("Wings/Red Retract Time", redRetractTime);
            Settings.putNumber("Wings/White Retract Time", whiteRetractTime);
        }
    }

    @Override
    public boolean isRedExtended() {
        return isEngaged(redDeploy);
    }

    @Override
    public boolean isWhiteExtended() {
        return isEngaged(whiteDeploy);
    }
}
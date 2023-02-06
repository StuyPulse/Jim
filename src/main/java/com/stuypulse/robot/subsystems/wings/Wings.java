package com.stuypulse.robot.subsystems.wings;

import static com.stuypulse.robot.constants.Settings.Wings.*;
import static com.stuypulse.robot.constants.Ports.Wings.*;
import com.stuypulse.stuylib.util.StopWatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wings extends IWings {

    private final StopWatch timer;

    private double leftDeployTime;
    private double rightDeployTime;
    private double leftRetractTime;
    private double rightRetractTime;

    // left solenoids
    private final DoubleSolenoid leftDeploy;
    private final Solenoid leftLatch;

    // right solenoids
    private final DoubleSolenoid rightDeploy;
    private final Solenoid rightLatch;

    public Wings() {
        timer = new StopWatch();

        leftDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, LEFT_DEPLOY_FORWARD, LEFT_DEPLOY_REVERSE);
        leftLatch = new Solenoid(PneumaticsModuleType.CTREPCM, LEFT_LATCH);
        leftDeployTime = -1.0;
        leftRetractTime = -1.0;
        
        rightDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RIGHT_DEPLOY_FORWARD, RIGHT_DEPLOY_REVERSE);
        rightLatch = new Solenoid(PneumaticsModuleType.CTREPCM, RIGHT_LATCH);
        rightDeployTime = -1.0;
        rightRetractTime = -1.0;

        leftLatch.set(true);
        rightLatch.set(true);

        rightDeploy.set(DoubleSolenoid.Value.kReverse);
        leftDeploy.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void extendLeft() {
        if (leftLatch.get() && !isEngaged(leftDeploy)) {
            leftLatch.set(false);
            leftDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractLeft() {
        if (!leftLatch.get() && isEngaged(leftDeploy)) {
            leftDeploy.set(DoubleSolenoid.Value.kReverse);
            leftRetractTime = timer.getTime();
        }
    }

    @Override
    public void extendRight() {
        if(rightLatch.get() && !isEngaged(rightDeploy)){
            rightLatch.set(false);
            rightDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractRight() {
        if(!rightLatch.get() && isEngaged(rightDeploy)){
            rightDeploy.set(DoubleSolenoid.Value.kReverse);
            rightRetractTime = timer.getTime();
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
        if(leftDeployTime > 0 && timer.getTime() - leftDeployTime >= LEFT_LATCH_DELAY.get()){
            leftDeploy.set(DoubleSolenoid.Value.kForward); // dont set off
            leftDeployTime = -1.0;
        }
        if(rightDeployTime > 0 && timer.getTime() - rightDeployTime >= RIGHT_LATCH_DELAY.get()){
            rightDeploy.set(DoubleSolenoid.Value.kForward);
            rightDeployTime = -1.0;
        }
        if(leftRetractTime > 0 && timer.getTime() - leftRetractTime >= LEFT_RETRACT_DELAY.get()){
            leftLatch.set(true);
            leftRetractTime = -1.0;
        }
        if(rightRetractTime > 0 && timer.getTime() - rightRetractTime >= RIGHT_RETRACT_DELAY.get()){
            rightLatch.set(true);
            rightRetractTime = -1.0;
        }

        SmartDashboard.putBoolean("Wings/Right Latch Engaged", rightLatch.get());
        SmartDashboard.putBoolean("Wings/Right Deploy Engaged", isEngaged(rightDeploy));
        SmartDashboard.putBoolean("Wings/Left Latch Engaged", leftLatch.get());
        SmartDashboard.putBoolean("Wings/Left Deploy Engaged", isEngaged(leftDeploy));

        SmartDashboard.putNumber("Wings/Current Time", timer.getTime());
        SmartDashboard.putNumber("Wings/Left Deploy Time",leftDeployTime);
        SmartDashboard.putNumber("Wings/Right Deploy Time",rightDeployTime);
        SmartDashboard.putNumber("Wings/Left Retract Time",leftRetractTime);
        SmartDashboard.putNumber("Wings/Right Retract Time",rightRetractTime);
    }
}
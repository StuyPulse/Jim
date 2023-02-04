package com.stuypulse.robot.subsystems.wings;

import static com.stuypulse.robot.constants.Settings.Wings.*;
import static com.stuypulse.robot.constants.Ports.Wings.*;
import com.stuypulse.stuylib.util.StopWatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

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
    }

    @Override
    public void extendLeft() {
        if (leftLatch.get() && leftDeploy.get() == DoubleSolenoid.Value.kOff) {
            leftLatch.set(false);
            leftDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractLeft() {
        if (!leftLatch.get() && leftDeploy.get() == DoubleSolenoid.Value.kForward) {
            leftDeploy.set(DoubleSolenoid.Value.kReverse);
            leftRetractTime = timer.getTime();
        }
    }

    @Override
    public void extendRight() {
        if(rightLatch.get() && rightDeploy.get() == DoubleSolenoid.Value.kOff){
            rightLatch.set(false);
            rightDeployTime = timer.getTime();
        }
    }

    @Override
    public void retractRight() {
        if(!rightLatch.get() && rightDeploy.get() == DoubleSolenoid.Value.kForward){
            rightDeploy.set(DoubleSolenoid.Value.kReverse);
            rightRetractTime = timer.getTime();
        }
    }

    @Override
    public void periodic() {
        if(leftDeployTime > 0 && timer.getTime() - leftDeployTime >= LEFT_LATCH_DELAY.get()){
            leftDeploy.set(DoubleSolenoid.Value.kOff);
            leftDeployTime = -1.0;
        }
        if(rightDeployTime > 0 && timer.getTime() - rightDeployTime >= RIGHT_LATCH_DELAY.get()){
            rightDeploy.set(DoubleSolenoid.Value.kOff);
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
    }
}
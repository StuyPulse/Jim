package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wings extends IWings {

    private final StopWatch timer;

    private double leftDeployTime;
    private double rightDeployTime;

    // left solenoids
    private final DoubleSolenoid leftDeploy;
    private final Solenoid leftPancake;

    // right solenoids
    private final DoubleSolenoid rightDeploy;
    private final Solenoid rightPancake;

    public Wings() {
        timer = new StopWatch();

        leftDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.LEFT_DEPLOY_FORWARD, Ports.Wings.LEFT_DEPLOY_REVERSE);
        leftPancake = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.LEFT_PANCAKE);
        leftDeployTime = -1.0;

        rightDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.RIGHT_DEPLOY_FORWARD, Ports.Wings.RIGHT_DEPLOY_REVERSE);
        rightPancake = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.LEFT_PANCAKE);
        rightDeployTime = -1.0;
    }

    public void extendLeft() {
        if (leftPancake.get() && !leftDeploy.get()) {
            leftPancake.set(false);
            leftDeployTime = timer.getTime();
        }
    }

    public void retractLeft() {
        if (!leftPancake.get() && leftDeploy.get()) {
            leftDeploy.set(false);
        }
    }

    public void extendRight() {
        if(rightPancake.get() && !rightDeploy.get()){
            rightPancake.set(false);
            rightDeployTime = timer.getTime();
        }
    }

    public void retractRight() {
        if(!rightPancake.get() && rightDeploy.get()){
            rightDeploy.set(false);
        }
    }

    @Override
    public void periodic() {
        if(leftDeployTime > 0 && timer.getTime() - leftDeployTime >= Settings.Wings.PANCAKE_DELAY){
            leftDeploy.set(true);
            leftDeployTime = -1.0;
        }
        if(leftDeployTime > 0 && timer.getTime() - rightDeployTime >= Settings.Wings.PANCAKE_DELAY){
            rightDeploy.set(true);
            rightDeployTime = -1.0;
        }
    }
}
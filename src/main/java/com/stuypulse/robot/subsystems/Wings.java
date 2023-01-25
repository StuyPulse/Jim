package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wings extends SubsystemBase {

    private final StopWatch timer;

    private double currentTime1;
    private double currentTime2;

    // left solenoids
    private final Solenoid deploy1;
    private final Solenoid pancake1;

    // right solenoids
    private final Solenoid deploy2;
    private final Solenoid pancake2;

    public Wings() {
        deploy1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.DEPLOY_1);
        deploy2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.DEPLOY_2);
        pancake1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.PANCAKE_1);
        pancake2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Wings.PANCAKE_2);
        timer = new StopWatch();
        currentTime1 = -1.0;
        currentTime2 = -1.0;
    }

    public void extendWing1() {
        if (pancake1.get() && !deploy1.get()) {
            pancake1.set(false);
            currentTime1 = timer.getTime();
        }
    }

    public void retractWing1() {
        if (!pancake1.get() && deploy1.get()) {
            deploy1.set(false);
        }
    }

    public void extendWing2() {
        if(pancake2.get() && !deploy2.get()){
            pancake2.set(false);
            currentTime2 = timer.getTime();
        }
    }

    public void retractWing2() {
        if(!pancake2.get() && deploy2.get()){
            deploy2.set(false);
        }
    }

    @Override
    public void periodic() {
        if(currentTime1 > 0 && timer.getTime() - currentTime1 >= Settings.Wings.PANCAKE_DELAY){
            deploy1.set(true);
            currentTime1 = -1.0;
        }
        if(currentTime1 > 0 && timer.getTime() - currentTime2 >= Settings.Wings.PANCAKE_DELAY){
            deploy2.set(true);
            currentTime2 = -1.0;
        }
    }
}
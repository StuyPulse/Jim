package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Wings extends SubsystemBase {

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
    }

    public void extendWing1() {
        pancake1.set(false);
        deploy1.set(true);
    }

    public void retractWing1() {
        pancake1.set(true);
        Timer.delay(Settings.Wings.RETRACT_DELAY);
        deploy1.set(false);
    }

    public void extendWing2() {
        pancake2.set(false);
        deploy2.set(true);
    }

    public void retractWing2() {
        pancake2.set(true);
        Timer.delay(Settings.Wings.RETRACT_DELAY);
        deploy2.set(false);
    }

    @Override
    public void periodic() {

    }
}
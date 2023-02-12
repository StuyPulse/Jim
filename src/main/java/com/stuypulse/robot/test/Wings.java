package com.stuypulse.robot.test;


import static com.stuypulse.robot.constants.Ports.Wings.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wings extends SubsystemBase {

    // left solenoids
    private final DoubleSolenoid leftDeploy;
    private final Solenoid leftLatch;

    // right solenoids
    private final DoubleSolenoid rightDeploy;
    private final Solenoid rightLatch;

    public Wings() {
        leftDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, LEFT_DEPLOY_FORWARD, LEFT_DEPLOY_REVERSE);
        leftLatch = new Solenoid(PneumaticsModuleType.CTREPCM, LEFT_LATCH);

        
        rightDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RIGHT_DEPLOY_FORWARD, RIGHT_DEPLOY_REVERSE);
        rightLatch = new Solenoid(PneumaticsModuleType.CTREPCM, RIGHT_LATCH);


        leftLatch.set(true);
        rightLatch.set(true);

        rightDeploy.set(DoubleSolenoid.Value.kReverse);
        leftDeploy.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendLeftLatch() {
        leftLatch.set(true);
    }

    public void retractLeftLatch() {
        leftLatch.set(false);
    }

    public void extendLeftDeploy() {
        leftDeploy.set(DoubleSolenoid.Value.kForward);
    }

    public void retractLeftDeploy() {
        leftDeploy.set(DoubleSolenoid.Value.kReverse);
    }

    
    public void extendRightLatch() {
        rightLatch.set(true);
    }

    public void retractRightLatch() {
        rightLatch.set(false);
    }

    public void extendRightDeploy() {
        rightDeploy.set(DoubleSolenoid.Value.kForward);
    }

    public void retractRightDeploy() {
        rightDeploy.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wings/Right Latch Engaged", rightLatch.get());
        SmartDashboard.putBoolean("Wings/Left Latch Engaged", leftLatch.get());

        SmartDashboard.putBoolean("Wings/Right Deployed", rightDeploy.get() == DoubleSolenoid.Value.kForward);
        SmartDashboard.putBoolean("Wings/Left Deployed", leftDeploy.get() == DoubleSolenoid.Value.kForward);

    }
}
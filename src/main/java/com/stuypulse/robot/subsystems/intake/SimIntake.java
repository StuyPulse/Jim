package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Settings.Intake.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartNumber;

public class SimIntake extends Intake {

    SmartNumber frontMotor = new SmartNumber("Intake/Front Motor", 0);
    SmartNumber backMotor = new SmartNumber("Intake/Back Motor", 0);

    public SimIntake() {}

    // INTAKING MODES

    @Override
    public void acquireCube() {
        frontMotor.set(INTAKE_CUBE_ROLLER_FRONT.doubleValue());
        backMotor.set(INTAKE_CUBE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void acquireCone() {
        frontMotor.set(-INTAKE_CONE_ROLLER_FRONT.doubleValue());
        backMotor.set(INTAKE_CONE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void deacquireCube() {
        frontMotor.set(-INTAKE_CUBE_ROLLER_FRONT.doubleValue());
        backMotor.set(-INTAKE_CUBE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void deacquireCone() {
        frontMotor.set(INTAKE_CONE_ROLLER_FRONT.doubleValue());
        backMotor.set(-INTAKE_CONE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    @Override
    public void periodic() {
        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
        }
    }
    
}

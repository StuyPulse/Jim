package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Settings.Intake.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartNumber;

public class SimIntake extends Intake {

    SmartNumber frontMotor = new SmartNumber("Intake/Front Motor", 0);
    SmartNumber backMotor = new SmartNumber("Intake/Back Motor", 0);

    public SimIntake() {}

    // INTAKING MODES

    @Override
    public void acquire() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE:
                frontMotor.set(Acquire.CUBE_FRONT.doubleValue());
                backMotor.set(Acquire.CUBE_BACK.doubleValue());
                break;
            case CONE_TIP_UP: // not really necessary, we can't pick up cones
            case CONE_TIP_IN:
                frontMotor.set(Acquire.CONE_FRONT.doubleValue());
                backMotor.set(-Acquire.CONE_BACK.doubleValue());    
                break;
            default:
                break;
        }
    }

    @Override
    public void deacquire() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE:
                frontMotor.set(-Deacquire.CUBE_FRONT.doubleValue());
                backMotor.set(-Deacquire.CUBE_BACK.doubleValue());
                break;
            case CONE_TIP_UP:
                // maybe check if in autonomous 
                frontMotor.set(Deacquire.CONE_UP_FRONT.doubleValue());
                backMotor.set(-Deacquire.CONE_UP_BACK.doubleValue()); 
            case CONE_TIP_IN:
                frontMotor.set(-Deacquire.CONE_FRONT.doubleValue());
                backMotor.set(Deacquire.CONE_BACK.doubleValue());    
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    @Override
    public void periodic() {
        Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
    }   
}

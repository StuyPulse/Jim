package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

public class IntakeImpl extends Intake {

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private BStream stalling;

    public IntakeImpl() {
        frontMotor = new CANSparkMax(FRONT_MOTOR_PORT, MotorType.kBrushless);
        backMotor = new CANSparkMax(BACK_MOTOR_PORT, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME));
    }

    // CONE DETECTION (stall detection)

    private double getMaxCurrent(){
        return Math.max(frontMotor.getOutputCurrent(), backMotor.getOutputCurrent());
    }

    private boolean isMomentarilyStalling() {
        return getMaxCurrent() > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    private boolean hasCone() {
        return isStalling();
    }

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
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // forward and stalling
        if (frontMotor.get() > 0 && hasCone()) {
            stop();
        }

        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
       
            Settings.putNumber("Intake/Front Roller Speed", frontMotor.get());
            Settings.putNumber("Intake/Back Roller Speed", backMotor.get());
            Settings.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
            Settings.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());
            Settings.putBoolean("Intake/Is Stalling", isStalling());
    
            Settings.putNumber("Intake/Front Motor", frontMotor.get());
            Settings.putNumber("Intake/Back Motor", backMotor.get());
        }
    }

}
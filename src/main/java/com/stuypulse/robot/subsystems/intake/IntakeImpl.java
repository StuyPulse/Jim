package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartBoolean;
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
    
    // GAMEPIECE DETECTION

    private boolean hasCone() {
        return isStalling();
    }

    @Override
    public void acquireCube() {
        frontMotor.set(INTAKE_CUBE_ROLLER_FRONT.doubleValue());
        backMotor.set(INTAKE_CUBE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void acquireCone() {
        frontMotor.set(INTAKE_CONE_ROLLER_FRONT.doubleValue());
        backMotor.set(-INTAKE_CONE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void deacquireCube() {
    frontMotor.set(-OUTTAKE_CUBE_ROLLER_FRONT.doubleValue());
        backMotor.set(-OUTTAKE_CUBE_ROLLER_BACK.doubleValue());
    }

    @Override
    public void deacquireCone() {
        if (Manager.getInstance().getGamePiece() == GamePiece.CONE_TIP_UP) {
            frontMotor.set(OUTTAKE_CONE_ROLLER_FRONT.doubleValue());
            backMotor.set(-OUTTAKE_CONE_ROLLER_BACK.doubleValue());
        } else {
            frontMotor.set(-OUTTAKE_CONE_ROLLER_FRONT.doubleValue());
            backMotor.set(OUTTAKE_CONE_ROLLER_BACK.doubleValue());
        }
    }

    @Override
    public void stop() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (hasCone()) {
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
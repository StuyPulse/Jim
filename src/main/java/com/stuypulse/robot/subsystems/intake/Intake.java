package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Motors.Intake.*;

public class Intake extends IIntake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private DigitalInput cubeSensor;
    private BStream stalling;

    public Intake(){
       
        frontMotor = new CANSparkMax(Ports.Intake.FRONTMOTOR, MotorType.kBrushless);
        backMotor = new CANSparkMax(Ports.Intake.BACKMOTOR, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME))
            .polling(Settings.DT);

        cubeSensor = new DigitalInput(Ports.Intake.IRSENSOR);
    }

    // CONE DETECTION (stall detection)

    private boolean isMomentarilyStalling() {
        return Math.abs(frontMotor.getOutputCurrent()) > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    // CUBE DETECTION (ir sensor)

    private boolean hasCube() {
        return !cubeSensor.get();
    }

    // WRIST ORIENTATION

    private boolean isFlipped() {
        IArm arm = IArm.getInstance();
        return arm.getWristAngle().toRadians() > Math.PI /2 || arm.getWristAngle().toRadians() < 3 * Math.PI / 2;
    }

    // INTAKING MODES

    public void cubeIntake(){
        if (isFlipped()) {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        }
    }

    public void coneIntake() {
        if (isFlipped()) {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        }
    }

    public void cubeOuttake(){
        if (isFlipped()) {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        }
    }

    public void coneOuttake(){
        if (isFlipped()) {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void periodic(){
        if (isStalling() || hasCube()) {
            frontMotor.stopMotor();
            backMotor.stopMotor();
        }
        SmartDashboard.putBoolean("Intake/Is Flipped", isFlipped());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Has Cube", hasCube());
    }

}
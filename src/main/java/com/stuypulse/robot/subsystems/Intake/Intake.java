package com.stuypulse.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.arm.Arm;

import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Motors.Intake.*;

public class Intake extends IIntake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;
    private DigitalInput IRSensor; 
    private Debouncer isTripped;
    private boolean isReversed;

    private final Arm arm;

    public Intake(Arm arm){
       
        frontMotor = new CANSparkMax(Ports.Intake.FRONTMOTOR, MotorType.kBrushless);
        backMotor = new CANSparkMax(Ports.Intake.BACKMOTOR, MotorType.kBrushless);
        IRSensor = new DigitalInput(Ports.Intake.IRSENSOR);

        isTripped = new Debouncer(debounceTime, DebounceType.kRising);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);
        this.arm = arm;
    }
    public double isReversed(){
        if(isReversed){
            return -1;
        } else{
            return 1;
        }
    }
    public void cubeIntake(){
        frontMotor.set(frontMotorSpeed.get() * isReversed());
        backMotor.set(cubeBackMotorSpeed.get() * isReversed());
    }

    public void coneIntake() {
        frontMotor.set(frontMotorSpeed.get() * isReversed());
        backMotor.set(-1 * cubeBackMotorSpeed.get() * isReversed());
    }

    public void cubeOuttake(){
        frontMotor.set(-1 * frontMotorSpeed.get()* isReversed());
        backMotor.set(-1 * cubeBackMotorSpeed.get() * isReversed());
    }

    public void coneOuttake(){
        frontMotor.set(-1 * frontMotorSpeed.get());
        backMotor.set(coneBackMotorSpeed.get());
    }


    public boolean isStalling() {
        return frontMotor.getOutputCurrent() > STALLING_THRESHOLD;
    }

    public void shouldStop(){
        if(isStalling() || isTripped.calculate(IRSensor.get())){ 
            frontMotor.set(0);
            backMotor.set(0);
        }
    }

    public double getAngle(){
        return arm.getWristAngle().toRadians();
    }

    // shut up amber
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake/is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/is Tripped", isTripped.calculate(IRSensor.get()));

        
        if (getAngle() > Math.PI) {
            isReversed = true;
        }
        
        shouldStop();
    }

}
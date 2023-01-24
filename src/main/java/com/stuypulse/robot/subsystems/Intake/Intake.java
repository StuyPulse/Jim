package com.stuypulse.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Ports;
import static com.stuypulse.robot.constants.Settings.Intake.*;

public class Intake extends IIntake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;
    private DigitalInput IRSensor; 
    private Debouncer isTripped;
    private boolean isReversed;

    public Intake(){
       
        frontMotor = new CANSparkMax(Ports.Intake.FRONTMOTOR, MotorType.kBrushless);
        backMotor = new CANSparkMax(Ports.Intake.BACKMOTOR, MotorType.kBrushless);
        IRSensor = new DigitalInput(Ports.Intake.IRSENSOR);

        backMotor.setInverted(true);

        frontMotor.setIdleMode(IdleMode.kBrake);
        backMotor.setIdleMode(IdleMode.kBrake);

        isTripped = new Debouncer(debounceTime, DebounceType.kRising);

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
        return 0;
    }

    // shut up amber
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake/is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/is Tripped", isTripped.calculate(IRSensor.get()));

        shouldStop();
        if (getAngle() > Math.PI) {
            isReversed = true;
        }
    }

}
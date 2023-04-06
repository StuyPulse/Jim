/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.test;
import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TestIntake extends SubsystemBase{

    private SmartNumber frontDutyCycle;
    private SmartNumber backDutyCycle;

    private CANSparkMax frontMotor;
    private CANSparkMax backMotor;

    private DigitalInput frontSensor;
    private DigitalInput backSensor;

    public TestIntake() {
        frontDutyCycle = new SmartNumber("Intake/Front Duty Cycle", 0.0);
        backDutyCycle = new SmartNumber("Intake/Back Duty Cycle", 0.0);

        frontMotor = new CANSparkMax(FRONT_MOTOR_PORT, MotorType.kBrushless);
        backMotor = new CANSparkMax(BACK_MOTOR_PORT, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        frontSensor = new DigitalInput(FRONT_SENSOR);
        backSensor = new DigitalInput(BACK_SENSOR);
    }

    public void runFront() {
        frontMotor.set(frontDutyCycle.get());
    }

    public void runBack() {
        backMotor.set(backDutyCycle.get());
    }

    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());

        SmartDashboard.putNumber("Intake/Front Motor", frontMotor.get());
        SmartDashboard.putNumber("Intake/Back Motor", backMotor.get());

        SmartDashboard.putBoolean("Intake/Front Sensor", frontSensor.get());
        SmartDashboard.putBoolean("Intake/Back Sensor", backSensor.get());
    }

}

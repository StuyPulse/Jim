/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeImpl extends Intake {

    private CANSparkMax frontMotor;
    private CANSparkMax backMotor;

    private BStream stalling;

    private boolean acquiring;

    protected IntakeImpl() {
        frontMotor = new CANSparkMax(FRONT_MOTOR_PORT, MotorType.kBrushless);
        backMotor = new CANSparkMax(BACK_MOTOR_PORT, MotorType.kBrushless);

        Motors.disableStatusFrames(frontMotor, 3, 4, 5, 6);
        Motors.disableStatusFrames(backMotor, 3, 4, 5, 6);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME));
    }

    public void enableCoast() {
        frontMotor.setIdleMode(IdleMode.kCoast);
        backMotor.setIdleMode(IdleMode.kCoast);
    }

    public void enableBreak() {
        frontMotor.setIdleMode(IdleMode.kBrake);
        backMotor.setIdleMode(IdleMode.kBrake);
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

    @Override
    public boolean hasGamePiece() {
        return isStalling();
    }

    @Override
    public void acquire() {
        acquiring = true;
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE:
                frontMotor.set(Acquire.CUBE_FRONT.doubleValue());
                backMotor.set(Acquire.CUBE_BACK.doubleValue());
                break;
            case CONE_TIP_UP:
                break;
            case CONE_TIP_OUT:
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
        acquiring = false;
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE:
                frontMotor.set(-Deacquire.CUBE_FRONT.doubleValue());
                backMotor.set(-Deacquire.CUBE_BACK.doubleValue());
                break;
            case CONE_TIP_UP:
                frontMotor.set(+Deacquire.CONE_UP_FRONT.doubleValue());
                backMotor.set(-Deacquire.CONE_UP_BACK.doubleValue());
                break;
            case CONE_TIP_OUT:
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
        acquiring = false;
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // acquiring
        if (Robot.getMatchState() == MatchState.TELEOP && acquiring) {
            acquire();
        }

        // forward and stalling
        if (frontMotor.get() > 0 && hasGamePiece()) {
            stop();
        }

        Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());

        SmartDashboard.putNumber("Intake/Front Roller Speed", frontMotor.get());
        SmartDashboard.putNumber("Intake/Back Roller Speed", backMotor.get());
        SmartDashboard.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());

        SmartDashboard.putNumber("Intake/Front Motor", frontMotor.get());
        SmartDashboard.putNumber("Intake/Back Motor", backMotor.get());
    }

}

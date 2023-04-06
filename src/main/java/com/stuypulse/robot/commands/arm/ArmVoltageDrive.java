/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm;

import static com.stuypulse.robot.constants.Settings.Operator.*;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmVoltageDrive extends CommandBase {

    private final Arm arm;

    private final IStream shoulderVoltage;
    private final IStream wristVoltage;

    public ArmVoltageDrive(Gamepad gamepad) {
        arm = Arm.getInstance();

        shoulderVoltage = IStream.create(gamepad::getLeftY)
            .filtered(
                x -> MathUtil.applyDeadband(x, 0.08),
                x -> Math.pow(x, 3),
                x -> SHOULDER_DRIVE_VOLTAGE.get()*x);

        wristVoltage = IStream.create(gamepad::getRightY)
            .filtered(
                x -> MathUtil.applyDeadband(x, 0.08),
                x -> Math.pow(x, 3),
                x -> WRIST_DRIVE_VOLTAGE.get()*x);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setCoast(false, false);
    }

    @Override
    public void execute() {
        arm.setShoulderVoltage(shoulderVoltage.get());
        arm.setWristVoltage(wristVoltage.get());
    }

}

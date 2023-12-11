/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm;

import static com.stuypulse.robot.constants.Settings.Operator.*;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.util.StopWatch;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmDrive extends CommandBase {
    private final Arm arm;

    private final IStream shoulder;
    private final IStream wrist;

    private final StopWatch timer;

    private final Gamepad gamepad;

    public ArmDrive(Gamepad gamepad) {
        arm = Arm.getInstance();

        this.gamepad = gamepad;

        // these give values in deg / s
        this.shoulder = IStream.create(gamepad::getLeftY).filtered(
            x -> SLMath.deadband(x, DEADBAND.get()),
            x -> SLMath.spow(x, 2));

        this.wrist = IStream.create(gamepad::getRightY).filtered(
            x -> SLMath.deadband(x, DEADBAND.get()),
            x -> SLMath.spow(x, 2));

        // timer is used to get deg from deg / s (by multiplying by time)
        timer = new StopWatch();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute(){
        final double dt = timer.reset();

        double sign = AbstractOdometry.getInstance().getRotation().getCos() > 0 ? 1 : -1;

        if (gamepad.getRawRightBumper()) {
            arm.setShoulderVoltage(shoulder.get() * SHOULDER_DRIVE_VOLTAGE.get());
            arm.setWristVoltage(wrist.get() * WRIST_DRIVE_VOLTAGE.get());
        } else {
            arm.moveShoulderTargetAngle(sign * shoulder.get() * dt * SHOULDER_TELEOP_SPEED.get());
            arm.moveWristTargetAngle(sign * wrist.get() * dt * WRIST_TELEOP_SPEED.get());
        }

    }
}

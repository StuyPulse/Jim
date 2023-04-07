/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

public class ArmStow extends ArmRoutine {

    public ArmStow() {
        super(Manager.getInstance()::getStowTrajectory);
    }

	@Override
	protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        double wristSafeAngle = Settings.Arm.Wrist.WRIST_SAFE_ANGLE.get();

        return new ArmTrajectory()
            .addState(new ArmState(src.getShoulderDegrees(), wristSafeAngle).setWristTolerance(12))
            .addState(new ArmState(dest.getShoulderDegrees(), wristSafeAngle).setWristTolerance(45))
            .addState(dest.setShoulderTolerance(4).setWristTolerance(5));
	}

}

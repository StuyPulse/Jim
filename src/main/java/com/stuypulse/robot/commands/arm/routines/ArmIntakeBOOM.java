/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.constants.ArmTrajectories.*;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;


public class ArmIntakeBOOM extends ArmRoutine {

    private final Arm arm;
    private final Manager manager;

    public ArmIntakeBOOM() {
        super(() -> ArmTrajectories.Acquire.kBOOMCubeAuton);

        arm = Arm.getInstance();
        manager = Manager.getInstance();
    }

	@Override
	protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        if (Robot.getMatchState() == MatchState.AUTO) {
            dest = Acquire.kBOOMCubeAuton;
            double intermediateShoulderDegrees = manager.getIntakeIntermediateTrajectory().getShoulderDegrees();
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

            return new ArmTrajectory()
                .addState(src.getShoulderDegrees(), wristSafeAngle)

                .addState(
                    new ArmState(intermediateShoulderDegrees, wristSafeAngle)
                        .setShoulderTolerance(15)
                        .setWristTolerance(360))

                .addState(
                    new ArmState(intermediateShoulderDegrees, dest.getWristDegrees())
                        .setWristTolerance(360))

                .addState(
                    new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                        .setShoulderTolerance(3)
                        .setWristTolerance(4));
        }

        double intermediateShoulderDegrees = manager.getIntakeIntermediateTrajectory().getShoulderDegrees();
        double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)

            .addState(
                new ArmState(intermediateShoulderDegrees, wristSafeAngle)
                    .setShoulderTolerance(15)
                    .setWristTolerance(360))

            .addState(
                new ArmState(intermediateShoulderDegrees, dest.getWristDegrees())
                    .setWristTolerance(360))

            .addState(
                new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                    .setShoulderTolerance(3)
                    .setWristTolerance(4));
	}

    @Override
    public void end(boolean interrupted) {
    }
}

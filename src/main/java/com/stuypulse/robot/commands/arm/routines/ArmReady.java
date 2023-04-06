/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;


public class ArmReady extends ArmRoutine {

    public ArmReady() {
        super(Manager.getInstance()::getReadyTrajectory);
    }

    @Override
    public void initialize() {
        super.initialize();


        arm.enableGamePieceGravityCompensation();
    }

    @Override
    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        if (Manager.getInstance().getGamePiece() == GamePiece.CONE_TIP_UP) {
            return new ArmTrajectory()

                .addState(
                    new ArmState(dest.getShoulderDegrees(), src.getWristDegrees())
                        .setWristLimp(true))

                .addState(
                    dest.getShoulderDegrees(),
                    (Manager.getInstance().getNodeLevel() == NodeLevel.MID) ? dest.getWristDegrees() : (src.getWristDegrees() + dest.getWristDegrees()) / 2.0)

                .addState(dest);
        }

        if (Robot.getMatchState() == MatchState.AUTO) {
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

            return new ArmTrajectory()
                .addState(new ArmState(src.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(45))
                .addState(
                    new ArmState(dest.getShoulderDegrees(), wristSafeAngle).setWristLimp(true).setWristTolerance(360))
                .addState(dest);
        }

        if (src.isOnSameSide(dest) && !src.isOverBumper()) {
            return new ArmTrajectory()
            .addState(
                new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                    .setShoulderTolerance(3)
                    .setWristTolerance(3));
        }

        // if (Manager.getInstance().getNodeLevel() == NodeLevel.LOW &&
        //     Manager.getInstance().getScoreSide() == ScoreSide.BACK) {
        // }

        double wristSafeAngle = Settings.Arm.Wrist.WRIST_SAFE_ANGLE.get();

        return new ArmTrajectory()
            .addState(
                new ArmState(src.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(12 /* TODO CHANGE ME */))

            .addState(
                new ArmState(dest.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(45))

            .addState(new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                .setShoulderTolerance(3)
                .setWristTolerance(3));
    }


}

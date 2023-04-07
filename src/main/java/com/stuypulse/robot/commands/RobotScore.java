/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Settings.Score.*;

import com.stuypulse.robot.constants.ArmTrajectories.Score;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotScore extends CommandBase {

    private final SwerveDrive swerve;
    private final Arm arm;
    private final Intake intake;
    private final Manager manager;

    public RobotScore() {
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        manager = Manager.getInstance();
        
        addRequirements(swerve, arm, intake);
    }


    @Override
    public void initialize() {
        switch (manager.getGamePiece()) {
            case CUBE:
                intake.deacquire();
                break;
            case CONE_TIP_IN:
                arm.setShoulderTargetAngle(Manager.getInstance().getReadyTrajectory().getShoulderState());
                arm.setWristVoltage(kWristVoltage.get());
                break;
            case CONE_TIP_OUT:
                if (manager.getNodeLevel() == NodeLevel.HIGH)
                    arm.setTargetState(Score.High.kConeTipOutFront);
                else if (manager.getNodeLevel() == NodeLevel.MID)
                    arm.setTargetState(Score.Mid.kConeTipOutFront);
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        if (manager.getGamePiece() == GamePiece.CONE_TIP_IN && manager.getNodeLevel() != NodeLevel.LOW) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kForwardSpeed.get()), 0, 0);

            // This assumes the cone tip in always does opposite side
            slowSpeeds.vxMetersPerSecond *= -1;

            swerve.setChassisSpeeds(slowSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        if (Manager.getInstance().getGamePiece() == GamePiece.CONE_TIP_IN) {
            arm.setWristVoltage(0);
        }

        swerve.stop();
    }

}

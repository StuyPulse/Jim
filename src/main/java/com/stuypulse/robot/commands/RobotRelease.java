/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Settings.Score.*;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotRelease extends CommandBase {

    private final SwerveDrive swerve;
    private final Arm arm;
    private final Intake intake;
    private final Manager manager;

    public RobotRelease() {
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        manager = Manager.getInstance();

        addRequirements(swerve, arm, intake);
    }


    @Override
    public void initialize() {
        if (!(manager.getGamePiece() == GamePiece.CONE_TIP_OUT && manager.getNodeLevel() != NodeLevel.LOW)) {
            intake.deacquire();
        }

        if ( manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {
            intake.enableCoast();
        }
    }

    @Override
    public void execute() {
        if (manager.getGamePiece() == GamePiece.CONE_TIP_IN && manager.getNodeLevel() == NodeLevel.HIGH) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kBackwardsTipInSpeed.get()), 0, 0);

            swerve.setChassisSpeeds(slowSpeeds);
        } else if (manager.getGamePiece() == GamePiece.CONE_TIP_OUT && manager.getNodeLevel() != NodeLevel.LOW) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(-Units.inchesToMeters(kBackwardsTipOutSpeed.get()), 0, 0);

            swerve.setChassisSpeeds(slowSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        intake.stop();
        intake.enableBreak();
        swerve.stop();
    }

}

/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireColyi extends InstantCommand {
    private Intake intake;
    private Manager manager;

    public IntakeAcquireColyi(){
        intake = Intake.getInstance();
        manager = Manager.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        boolean isCubeMidOppositeBackConditionBooleanExpressionStoredInAVariable = manager.getScoreSide() == ScoreSide.BACK && manager.getGamePiece().isCube() && manager.getNodeLevel() == NodeLevel.MID;

        if (manager.getGamePiece().isCone() || isCubeMidOppositeBackConditionBooleanExpressionStoredInAVariable) {
            intake.acquire();
        }
    }
}

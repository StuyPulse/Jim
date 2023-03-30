package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.swerve.SwerveDriveToScorePose;
import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotAlignThenScore extends SwerveDriveToScorePose {    
    private static double getTimeout() {
        switch (Manager.getInstance().getGamePiece()) {
            case CONE_TIP_IN:
                return 1;
            case CONE_TIP_OUT:
                return 0.5;
            default:
                return 0;
        }
    }

    public RobotAlignThenScore() {
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        // if (isReadyToScore()) {
        //     // do stuff >w<
        // } else {
        //     super.execute();
        // }
    }

}

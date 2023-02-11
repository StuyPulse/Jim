package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetScoreSide extends InstantCommand {

    public ManagerSetScoreSide(ScoreSide scoreSide) {
        super(() -> Manager.getInstance().setScoreSide(scoreSide));
    }

}
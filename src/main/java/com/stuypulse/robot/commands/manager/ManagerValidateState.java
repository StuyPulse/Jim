package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerValidateState extends InstantCommand {

    public ManagerValidateState() {
        super(() -> {
            var manager = Manager.getInstance();

            // if trying to score tip out and high, set node level to mid
            if (manager.getNodeLevel() == NodeLevel.HIGH && 
                manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {

                manager.setNodeLevel(NodeLevel.MID);
            }

            // allow any scoring side for low
            if (manager.getNodeLevel() != NodeLevel.LOW) {
                
                // if trying to score a tip out cone opposite, set same side
                if (manager.getScoreSide() == ScoreSide.OPPOSITE && manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {
                    manager.setScoreSide(ScoreSide.SAME);
                }

                // if trying to score tip in on thes same side, set opposite side
                else if (manager.getScoreSide() == ScoreSide.SAME && manager.getGamePiece() == GamePiece.CONE_TIP_IN) {
                    manager.setScoreSide(ScoreSide.OPPOSITE);
                }
            }


        });
    }
}

package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerValidateState extends InstantCommand {
    public ManagerValidateState() {
        super(() -> {
            var manager = Manager.getInstance();

            if (manager.getNodeLevel() == NodeLevel.HIGH && 
                manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {

                manager.setNodeLevel(NodeLevel.MID);
            }
        });
    }
}

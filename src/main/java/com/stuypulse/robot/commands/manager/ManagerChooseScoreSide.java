package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerChooseScoreSide extends InstantCommand {
    
    private static boolean facingAway(Rotation2d angle) {
        return Math.abs(MathUtil.inputModulus(angle.getDegrees(), -180, 180)) < 90;
    }

    public static boolean possibleScoringMotion(NodeLevel level, GamePiece piece, ScoreSide side) {
        if (piece == GamePiece.CONE_TIP_OUT) {
            if (level == NodeLevel.HIGH)
                return false;
            
            else if (level == NodeLevel.MID && side == ScoreSide.OPPOSITE)
                return false;
        }
        
        return true;
    }

    private static ScoreSide currentScoringSide() {
        var manager = Manager.getInstance();
        if (facingAway(Odometry.getInstance().getRotation())) {
            if (manager.getIntakeSide() == IntakeSide.FRONT) {
                return ScoreSide.OPPOSITE;
            } else {
                return ScoreSide.SAME;
            }
        } else {
            if (manager.getIntakeSide() == IntakeSide.FRONT) {
                return ScoreSide.SAME;
            } else {
                return ScoreSide.OPPOSITE;
            }
        }
    }
    
    public ManagerChooseScoreSide() {
        super(() -> {
            var manager = Manager.getInstance();
            if (possibleScoringMotion(manager.getNodeLevel(), manager.getGamePiece(), currentScoringSide()))
                manager.setScoreSide(currentScoringSide());
            else
                manager.setScoreSide(currentScoringSide().getOpposite());
        });
    }

}

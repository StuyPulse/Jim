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

    public static boolean isScoreMotionPossible(NodeLevel level, GamePiece piece, ScoreSide side) {
        
        if (piece == GamePiece.CONE_TIP_OUT) {
            // cannot score high tip in
            if (level == NodeLevel.HIGH)
                return false;
            
            // cannot score mid, opposite side tip in
            else if (level == NodeLevel.MID && side == ScoreSide.OPPOSITE)
                return false;
        }

        else if (piece == GamePiece.CONE_TIP_IN) {
            // cannot score mid or high, same side
            if (level != NodeLevel.LOW && side == ScoreSide.SAME) 
                return false;
        }
        
        return true;
    }

    private static ScoreSide getCurrentScoringSide(IntakeSide intakeSide, Rotation2d heading) {
        if (facingAway(heading)) {
            if (intakeSide == IntakeSide.FRONT) {
                return ScoreSide.OPPOSITE;
            } else {
                return ScoreSide.SAME;
            }
        } else {
            if (intakeSide == IntakeSide.FRONT) {
                return ScoreSide.SAME;
            } else {
                return ScoreSide.OPPOSITE;
            }
        }
    }
    
    public ManagerChooseScoreSide() {
        super(() -> {
            var manager = Manager.getInstance();
            var currentScoringSide = getCurrentScoringSide(manager.getIntakeSide(), Odometry.getInstance().getRotation());
            if (isScoreMotionPossible(manager.getNodeLevel(), manager.getGamePiece(), currentScoringSide))
                manager.setScoreSide(currentScoringSide);
            else
                manager.setScoreSide(currentScoringSide.getOpposite());
        });
    }

}

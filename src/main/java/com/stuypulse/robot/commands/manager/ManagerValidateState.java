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

public class ManagerValidateState extends InstantCommand {

    /** determines if the robot is facing towards or away from the grid */
    private static boolean facingAway(Rotation2d angle) {
        return Math.abs(MathUtil.inputModulus(angle.getDegrees(), -180, 180)) < 90;
    }

    /** determines if a scoring motion is possible */
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

    /** gets the IDEAL scoring side based on intake side and the robot facing. This does not 
     * take into account game pieces
     */
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

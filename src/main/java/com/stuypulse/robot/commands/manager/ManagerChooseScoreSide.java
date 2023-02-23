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

            // if game piece is cube, automatically choose scoring side
            if (manager.getGamePiece() == GamePiece.CUBE) {
                var currentScoringSide = getCurrentScoringSide(manager.getIntakeSide(), Odometry.getInstance().getRotation());
                manager.setScoreSide(currentScoringSide);
            }
        });
    }

}

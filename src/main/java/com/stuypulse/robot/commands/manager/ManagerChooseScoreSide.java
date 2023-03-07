package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
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

    private static ScoreSide getCurrentScoringSide(Rotation2d heading) {
        return facingAway(heading) ? ScoreSide.BACK : ScoreSide.FRONT;
    }
    
    public ManagerChooseScoreSide() {
        super(() -> {
            var manager = Manager.getInstance();

            // if game piece is cube, automatically choose scoring side
            if (manager.getGamePiece().isCube() || manager.getNodeLevel() == NodeLevel.LOW) {
                manager.setScoreSide(getCurrentScoringSide(Odometry.getInstance().getRotation()));
            }
        });
    }

}

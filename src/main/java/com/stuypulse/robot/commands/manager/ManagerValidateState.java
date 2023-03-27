package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerValidateState extends InstantCommand {
    
    private static boolean facingAway(Rotation2d angle) {
        return Math.abs(MathUtil.inputModulus(angle.getDegrees(), -180, 180)) < 90;
    }

    private static ScoreSide getCurrentScoringSide(Rotation2d heading) {
        return facingAway(heading) ? ScoreSide.BACK : ScoreSide.FRONT;
    }
    
    public ManagerValidateState() {
        super(() -> {
            var manager = Manager.getInstance();
            
            ScoreSide facingSide = getCurrentScoringSide(Odometry.getInstance().getRotation());
            manager.setScoreSide(facingSide);

            // if game piece is cone, choose cone type based on side
            if (manager.getGamePiece().isCone()) {
                if (facingSide == ScoreSide.FRONT) {
                    manager.setGamePiece(GamePiece.CONE_TIP_OUT);
                } else {
                    manager.setGamePiece(GamePiece.CONE_TIP_IN);
                }
            }
        });
    }

}

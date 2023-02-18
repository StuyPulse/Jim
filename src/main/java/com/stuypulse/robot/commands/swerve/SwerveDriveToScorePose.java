package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

public class SwerveDriveToScorePose extends SwerveDriveToPose {
    
    public SwerveDriveToScorePose() {
        super(() -> Manager.getInstance().getScorePose());
    }

    @Override
    public void initialize() {
        super.initialize();

        Manager manager = Manager.getInstance();
        ScoreSide scoreSide = manager.currentScoringSide();

        manager.setScoreSide(
            manager.possibleScoringMotion(manager.getNodeLevel(), manager.getGamePiece(), scoreSide)
                ? scoreSide
                : scoreSide.getOpposite()
        );
    }

}

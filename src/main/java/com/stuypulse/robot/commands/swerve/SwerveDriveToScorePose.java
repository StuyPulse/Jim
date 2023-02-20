package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.Manager;

public class SwerveDriveToScorePose extends SwerveDriveToPose {
    
    public SwerveDriveToScorePose() {
        super(() -> Manager.getInstance().getScorePose());
    }

}

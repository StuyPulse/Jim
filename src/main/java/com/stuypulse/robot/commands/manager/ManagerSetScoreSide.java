package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetScoreSide extends InstantCommand {
    private final Manager manager;
    private final ScoreSide scoreSide; 

    public ManagerSetScoreSide(ScoreSide scoreSide) {
        manager = Manager.getInstance();        
        this.scoreSide = scoreSide;
        // addRequirements(manager); Myles think about this later
    }

    @Override
    public void initialize() {
        manager.setScoreSide(scoreSide);
    }
}
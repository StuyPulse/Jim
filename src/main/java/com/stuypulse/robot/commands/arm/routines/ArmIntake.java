package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.ArmTrajectories.Neutral;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

public class ArmIntake extends ArmRoutine {
    
    public ArmIntake() {
        super(Manager.getInstance()::getIntakeTrajectory);
    }

	@Override
	protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        double intermediateShoulderDegrees = Neutral.kTrajectory.getShoulderDegrees();

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), 90)
            .addState(intermediateShoulderDegrees, 90)
            .addState(intermediateShoulderDegrees, dest.getWristDegrees())
            .addState(dest);
	}
}
package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

public class ArmStow extends ArmRoutine {
    
    public ArmStow() {
        super(Manager.getInstance()::getStowTrajectory);
    }

	@Override
	protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        double wristSafeAngle = 90; // src.getShoulderState().getCos() > 0 ? 120 : 60;
        if (Math.abs(src.getShoulderDegrees() + 90) < 30)
            wristSafeAngle = 90;

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)
            .addState(dest.getShoulderDegrees(), wristSafeAngle)
            .addState(dest.setShoulderTolerance(4).setWristTolerance(5));
	}

}

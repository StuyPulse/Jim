package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.Settings;
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
        double wristSafeAngle = Settings.Arm.Wrist.WRIST_SAFE_ANGLE.get();

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)
            .addState(dest.getShoulderDegrees(), wristSafeAngle)
            .addState(dest.setShoulderTolerance(4).setWristTolerance(5));
	}

}

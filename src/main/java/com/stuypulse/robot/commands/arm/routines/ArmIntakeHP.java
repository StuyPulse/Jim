package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

public class ArmIntakeHP extends ArmRoutine {

    private final Arm arm;
    private final Manager manager;

    public ArmIntakeHP() {
        super(() -> ArmTrajectories.Acquire.kHPCone);

        arm = Arm.getInstance();
        manager = Manager.getInstance();
    }

    @Override
    public ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        if (src.getShoulderDegrees() > 70 && dest.getShoulderDegrees() > 70) {
            return new ArmTrajectory().addState(dest);
        }
        return super.getTrajectory(src, dest);
    }
}

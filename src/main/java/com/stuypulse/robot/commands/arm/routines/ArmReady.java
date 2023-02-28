package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

public class ArmReady extends ArmRoutine {
    
    public ArmReady() {
        super(Manager.getInstance()::getReadyTrajectory);
    }

    @Override
    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        if (!DriverStation.isAutonomous()) {
            return super.getTrajectory(src, dest);
        }
        // TODO: check if src and dest are on the same side
        double wristSafeAngle = 90; // src.getShoulderState().getCos() > 0 ? 120 : 60;

        return new ArmTrajectory()
            .addState(dest.getShoulderDegrees(), src.getWristDegrees())
            .addState(dest);
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return super.isFinished(); // Math.abs(Arm.getInstance().getWristVelocityRadiansPerSecond()) < Math.toRadians(5);
        } else {
            return super.isFinished();
        }
    }
}

package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.ArmTrajectories.*;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

public class ArmIntake extends ArmRoutine {

    private final Arm arm;
    private final Manager manager;
    
    public ArmIntake() {
        super(Manager.getInstance()::getIntakeTrajectory);

        arm = Arm.getInstance();
        manager = Manager.getInstance();
    }

	@Override
	protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        // if (DriverStation.isAutonomous()) {
        //     double wristSafeAngle = src.getShoulderState().getCos() > 0 ? 90 : 60;
        //     return new ArmTrajectory()
        //         .addState(src.getShoulderDegrees(), wristSafeAngle)
        //         .addState(dest.getShoulderDegrees(), wristSafeAngle)
        //         .addState(dest);
        // }

        double intermediateShoulderDegrees = Acquire.kIntermediate.getShoulderDegrees();
        double wristSafeAngle = src.getShoulderState().getCos() > 0 ? 90 : 60;

        // TODO: make tolerance on second to last lower
        // TODO: make second only check shoulder tolerance
        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)

            .addState(
                new ArmState(intermediateShoulderDegrees, wristSafeAngle)
                    .setShoulderTolerance(15)
                    .setWristTolerance(360))

            .addState(
                new ArmState(intermediateShoulderDegrees, dest.getWristDegrees())
                    .setWristTolerance(360))

            .addState(dest);
	}

    @Override
    public void end(boolean interrupted) {
        // super.end(interrupted);

        if (!interrupted) {
            arm.setShoulderVoltage(Shoulder.INTAKE_VOLTAGE.get());

            // if (manager.getGamePiece().isCube()) {
            //     arm.setWristVoltage(Wrist.INTAKE_VOLTAGE.get());
            // }
        }
    }
}
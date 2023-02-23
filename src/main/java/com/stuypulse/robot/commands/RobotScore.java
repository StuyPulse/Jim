package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.Routine;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotScore extends CommandBase {
    
    private final static SmartNumber kForwardSpeed = new SmartNumber("Robot Score/Forward Speed (in per s)", 16);
    private final static SmartNumber kBackupSpeed = new SmartNumber("Robot Score/Backup Speed (in per s)", 24);

    private final SwerveDrive swerve;
    private final Arm arm;
    private final Intake intake;
    private final Manager manager;

    public RobotScore() {
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        manager = Manager.getInstance();

        addRequirements(swerve, arm, intake);
    }


    @Override
    public void initialize() {
        manager.setRoutine(Routine.READY);
        arm.setTrajectory(manager.getScoreTrajectory());


        boolean shouldOuttake = !(manager.getNodeLevel() != NodeLevel.LOW && manager.getGamePiece() == GamePiece.CONE_TIP_OUT);
        if (shouldOuttake) {
            if (manager.getGamePiece().isCone()) {
                intake.deacquireCone();
            } else {
                intake.deacquireCube();
            }
        }
    }

    @Override
    public void execute() {
        if (manager.getGamePiece() == GamePiece.CONE_TIP_IN && manager.getNodeLevel() == NodeLevel.HIGH) {
            
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kForwardSpeed.get()), 0, 0);

            // THis assumes the cone tip in always does opposite side (which is true for now)
            if (Intake.getInstance().getIntookSide() == IntakeSide.FRONT) {
                slowSpeeds.vxMetersPerSecond *= -1;
            }
            
            swerve.setChassisSpeeds(slowSpeeds);
        }


        else if (manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kBackupSpeed.get()), 0, 0);

            // THis assumes the cone tip in always does same side (which is true for now)
            if (Intake.getInstance().getIntookSide() == IntakeSide.FRONT) {
                slowSpeeds.vxMetersPerSecond *= -1;
            }
            
            swerve.setChassisSpeeds(slowSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        intake.stop();

        // holds arm in place
        arm.setTargetState(arm.getState());
    }

}

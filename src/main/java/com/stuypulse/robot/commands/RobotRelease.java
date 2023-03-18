package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotRelease extends CommandBase {
    
    private final static SmartNumber kBackwardsTipInSpeed = new SmartNumber("Robot Score/Tip In Backwards Speed (in per s)", 16);
    private final static SmartNumber kBackwardsTipOutSpeed = new SmartNumber("Robot Score/Tip Out Backwards Speed (in per s)", 16);

    private final SwerveDrive swerve;
    private final Arm arm;
    private final Intake intake;
    private final Manager manager;

    public RobotRelease() {
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        manager = Manager.getInstance();

        addRequirements(swerve, arm, intake);
    }


    @Override
    public void initialize() {
        intake.deacquire();
    }

    @Override
    public void execute() {
        if (manager.getGamePiece() == GamePiece.CONE_TIP_IN && manager.getNodeLevel() == NodeLevel.HIGH) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kBackwardsTipInSpeed.get()), 0, 0);
            
            swerve.setChassisSpeeds(slowSpeeds);
        } else if (manager.getGamePiece() == GamePiece.CONE_TIP_OUT && manager.getNodeLevel() != NodeLevel.LOW) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kBackwardsTipOutSpeed.get()), 0, 0);
            
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
        swerve.stop();
    }

}

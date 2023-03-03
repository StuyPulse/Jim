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

public class RobotScore extends CommandBase {
    
    private final static SmartNumber kForwardSpeed = new SmartNumber("Robot Score/Forward Speed (in per s)", 4);
    private final static SmartNumber kWristVoltage = new SmartNumber("Robot Score/Wrist Voltage", 2);
    private final static SmartNumber kShoulderDownVolts = new SmartNumber("Robot Score/Shoulder Down Speed (V)", 0.5);

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

        arm.setWristVoltage(kWristVoltage.get());

        if (manager.getGamePiece().isCube()) {
            intake.deacquire();
        }
        
    }

    @Override
    public void execute() {
        if (manager.getGamePiece() == GamePiece.CONE_TIP_IN && manager.getNodeLevel() != NodeLevel.LOW) {
            ChassisSpeeds slowSpeeds = new ChassisSpeeds(Units.inchesToMeters(kForwardSpeed.get()), 0, 0);

            // This assumes the cone tip in always does opposite side
            slowSpeeds.vxMetersPerSecond *= -1;
            
            swerve.setChassisSpeeds(slowSpeeds);
        } else if (manager.getGamePiece() == GamePiece.CONE_TIP_OUT && manager.getNodeLevel() != NodeLevel.LOW) {
            arm.setShoulderVoltage(kShoulderDownVolts.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        // holds arm in place
        arm.setTargetState(arm.getState());

        swerve.stop();
    }

}

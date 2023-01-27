package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.DoubleJointedArmSim;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends IArm {

    private final DoubleJointedArmSim armSim;
    private final ArmVisualizer visualizer;

    // TODO: Make AngleController
    private final Controller shoulderController; 
    private final Controller wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new DoubleJointedArmSim(new SingleJointedArmSim(DCMotor.getNEO(1), Shoulder.GEARING, Shoulder.JKG+Wrist.JKG, Units.inchesToMeters(Shoulder.LENGTH), Shoulder.MIN_ANGLE, Shoulder.MAX_ANGLE, Shoulder.MASS, true), 
            new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Units.inchesToMeters(Wrist.LENGTH), Wrist.MIN_ANGLE, Wrist.MAX_ANGLE, Wrist.MASS, true));

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).position()
                                    .add(new ArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).position()
                                    .add(new ArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));

        shoulderTargetAngle = new SmartNumber("Arm/Target Arm Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle", 0);

        visualizer = new ArmVisualizer();
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(armSim.getShoulderAngleDegrees());
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(armSim.getWristAngleDegrees());
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetAngle.get());
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Shoulder.MIN_ANGLE, Shoulder.MAX_ANGLE));
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        wristTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Wrist.MIN_ANGLE, Wrist.MAX_ANGLE));
    }
    
    // don't need methods below
    
    @Override
    public boolean isShoulderAtAngle(Rotation2d maxError) {
        return true;    
    }

    @Override
    public boolean isWristAtAngle(Rotation2d maxError) {
        return true;
    }

    @Override
    public void periodic() {    
        armSim.setInput(shoulderController.update(shoulderTargetAngle.get(), getShoulderAngle().getDegrees()), wristController.update(wristTargetAngle.get(), getWristAngle().getDegrees()));

        armSim.update(Settings.DT);

        visualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        visualizer.setTargetAngles(shoulderTargetAngle.get(), wristTargetAngle.get());

        SmartDashboard.putNumber("Arm/Arm Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Arm Voltage", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}
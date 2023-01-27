package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.DoubleJointedArmSim;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.interpolation.CubicInterpolator;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends IArm {

    private final DoubleJointedArmSim armSim;

    private final Controller shoulderController; 
    private final Controller wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new DoubleJointedArmSim(new SingleJointedArmSim(DCMotor.getNEO(1), Shoulder.GEARING, Shoulder.JKG+Wrist.JKG, Units.inchesToMeters(Shoulder.LENGTH), Shoulder.MINANGLE, Shoulder.MAXANGLE, Shoulder.MASS, true), 
            new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Units.inchesToMeters(Wrist.LENGTH), Wrist.MINANGLE, Wrist.MAXANGLE, Wrist.MASS, true));

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kA, Shoulder.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kA, Wrist.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));

        shoulderTargetAngle = new SmartNumber("Arm/Target Arm Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle", 0);
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
    public double getShoulderTargetAngle() {
        return shoulderTargetAngle.get();
    }

    @Override
    public double getWristTargetAngle() {
        return wristTargetAngle.get();
    }

    @Override
    public void setTargetShoulderAngle(double angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Shoulder.MINANGLE), Math.toDegrees(Shoulder.MAXANGLE)));
    }

    @Override
    public void setTargetWristAngle(double angle, boolean longPath) {
        double error = MathUtil.inputModulus(angle - getWristAngle().getDegrees(), -180, 180);
        angle = getWristAngle().getDegrees() + error;

        if (longPath) {
            if (error < 0) angle += 360;
            else           angle -= 360;
        }

        wristTargetAngle.set(angle);
    }

    // don't need methods below
    
    @Override
    public boolean isShoulderAtAngle(double maxError) {
        return true;    
    }

    @Override
    public boolean isWristAtAngle(double maxError) {
        return true;
    }

    @Override
    public void periodic() {        
        armSim.setInput(shoulderController.update(shoulderTargetAngle.get(), getShoulderAngle().getDegrees()), wristController.update(wristTargetAngle.get(), getWristAngle().getDegrees()));

        armSim.update(Settings.DT);

        SmartDashboard.putNumber("Arm/Arm Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Arm Voltage", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}
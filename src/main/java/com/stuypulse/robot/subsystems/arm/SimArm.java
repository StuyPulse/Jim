package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import static com.stuypulse.robot.constants.Settings.Arm.*;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.DoubleJointedArmSim;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends Arm {

    private final DoubleJointedArmSim armSim;
    private final ArmVisualizer visualizer;

    private final AngleController shoulderController; 
    private final AngleController wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    private final FieldObject2d fieldObject;

    public SimArm() { 
        setSubsystem("SimArm");
        
        armSim = new DoubleJointedArmSim(
            // shoulder
            DCMotor.getNEO(2), Shoulder.GEARING, Shoulder.JKG, Units.inchesToMeters(Shoulder.LENGTH), Shoulder.MIN_ANGLE, Shoulder.MAX_ANGLE, 
            // wrist
            DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Units.inchesToMeters(Wrist.LENGTH), Wrist.MIN_ANGLE, Wrist.MAX_ANGLE
        );

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));

        shoulderTargetAngle = new SmartNumber("Arm/Target Arm Angle (deg)", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle (deg)", 0);
        visualizer = new ArmVisualizer();

        fieldObject = Odometry.getInstance().getField().getObject("Field Arm");
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
    public boolean isShoulderAtAngle(Rotation2d maxError) {
        return Math.abs(shoulderTargetAngle.get() - getShoulderAngle().getDegrees()) < maxError.getDegrees();    
    }

    @Override
    public boolean isWristAtAngle(Rotation2d maxError) {
        return Math.abs(wristTargetAngle.get() - getWristAngle().getDegrees()) < maxError.getDegrees();
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Shoulder.MIN_ANGLE, Shoulder.MAX_ANGLE));
    }
    
    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        wristTargetAngle.set(MathUtil.clamp(angle.getDegrees(), Wrist.MIN_ANGLE, Wrist.MAX_ANGLE));
    }

    private void updateFieldObject() {
        double distanceFromSwerveCenter = getShoulderAngle().getCos() * Shoulder.LENGTH + getWristAngle().getCos() * Wrist.LENGTH;

        Pose2d swervePose = Odometry.getInstance().getPose();
        Translation2d topDownTranslation = new Translation2d(distanceFromSwerveCenter, swervePose.getRotation());
        
        fieldObject.setPose(new Pose2d(
            topDownTranslation.plus(swervePose.getTranslation()),
            swervePose.getRotation()
        ));
    }

    @Override
    public void periodic() {

        double shoulderOutput = shoulderController.update(Angle.fromDegrees(shoulderTargetAngle.get()), Angle.fromRotation2d(getShoulderAngle()));
        double wristOutput = wristController.update(Angle.fromDegrees(wristTargetAngle.get()), Angle.fromRotation2d(getWristAngle()));;

        // if (Shoulder.DEADZONE_ENABLED.get() & Math.abs(shoulderTargetAngle.get()) < Shoulder.ANGLE_DEADZONE_HIGH & Math.abs(shoulderTargetAngle.get()) > Shoulder.ANGLE_DEADZONE_LOW) {
        //     wristOutput = wristController.update(Angle.k90deg, Angle.fromRotation2d(getWristAngle()));
        // } else {
        //     wristOutput = wristController.update(Angle.fromDegrees(wristTargetAngle.get()), Angle.fromRotation2d(getWristAngle()));
        // }
    
        armSim.setInput(shoulderOutput, wristOutput);

        armSim.update(Settings.DT);

        visualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        visualizer.setTargetAngles(shoulderTargetAngle.get(), wristTargetAngle.get());

        updateFieldObject();

        SmartDashboard.putNumber("Arm/Arm Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle (deg)", getWristAngle().getDegrees());

        SmartDashboard.putNumber("Arm/Arm Voltage", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}
package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmJoint;
import com.stuypulse.robot.util.ArmVisualizer;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpinnyTestSimArm extends Arm {

    private ArmDynamics dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
    private ArmVisualizer visualizer = new ArmVisualizer();

    private Vector<N4> state = VecBuilder.fill(-Math.PI/6, Math.PI/2, 0, 0);

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRadians(state.get(0, 0));
    }

    @Override
    public Rotation2d getWristAngle() {
        // TODO Auto-generated method stub
        return Rotation2d.fromRadians(state.get(1, 0));
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        // TODO Auto-generated method stub
        return new Rotation2d();
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        // TODO Auto-generated method stub
        return new Rotation2d();
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setFeedbackEnabled(boolean enabled) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public ArmVisualizer getVisualizer() {
        // TODO Auto-generated method stub
        return visualizer;
    }


    SmartNumber wristVoltage = new SmartNumber("Arm/Wrist Input Voltage", 0);
    PIDController pdController = new PIDController(
        new SmartNumber("Arm/Shoulder P", 5.0),
        0, 
        new SmartNumber("Arm/Shoulder D", 1.0));

    @Override
    public void periodic() {
        var u_ff = dynamics.feedforward(
                VecBuilder.fill(-Math.PI/6, 0),
                VecBuilder.fill(0, 0),
                VecBuilder.fill(0, 0));

        var u = VecBuilder.fill(u_ff.get(0, 0) + pdController.update(-Math.PI/6, state.get(0, 0)), wristVoltage.get());
        u = VecBuilder.fill(
            MathUtil.clamp(u.get(0, 0), -12, 12),
            MathUtil.clamp(u.get(1, 0), -12, 12));

        SmartDashboard.putNumber("Arm/Shoulder Voltage", u.get(0, 0));
        SmartDashboard.putNumber("Arm/Wrist Voltage", u.get(1, 0));
        state = dynamics.simulate(state, u, 0.02);

        visualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getShoulderAngle().plus(getWristAngle()).getDegrees());
    }

}
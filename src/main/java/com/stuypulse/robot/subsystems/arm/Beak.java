package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Beak extends SubsystemBase {
    private static Beak instance;
    static {
        instance = new BeakImpl();
    }

    public static Beak getInstance() {
        return instance;
    }

    // controllers for each joint
    private final Controller shoulderController;
    private final AngleController wristController;

    protected Beak() {
        
        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, 
        Shoulder.Feedforward.kA)
                .position()
                .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD));

        // commented out cuz errors
        // wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, 
        // Wrist.feedforward.kA).position()
        //         .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD));

    }

}
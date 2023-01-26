    package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm;
import com.stuypulse.robot.constants.Settings.Arm.Simulation.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Simulation.Wrist;
import com.stuypulse.robot.subsystems.IArm;
import com.stuypulse.robot.util.DoubleJointedArmSim;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SimArm extends IArm {

    private final DoubleJointedArmSim armSim;
    
    private MechanismLigament2d baseLigament;
    private MechanismLigament2d shoulderLigament;
    private MechanismLigament2d wristLigament;
    private MechanismLigament2d swerveLigamentRight;
    private MechanismLigament2d pegMid;
    private MechanismLigament2d pegTop;


    private MechanismRoot2d shoulderRoot;
    private MechanismRoot2d wristRoot;
    private MechanismRoot2d swerveRoot;
    private MechanismRoot2d pegRootMid;
    private MechanismRoot2d pegRootTop;

    private final Mechanism2d arm;

    private final Controller shoulderController; 
    private final Controller wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new DoubleJointedArmSim(new SingleJointedArmSim(DCMotor.getNEO(1), Shoulder.GEARING, Shoulder.JKG+Wrist.JKG, Units.inchesToMeters(Shoulder.LENGTH), Shoulder.MINANGLE, Shoulder.MAXANGLE, Shoulder.MASS, true), 
            new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Units.inchesToMeters(Wrist.LENGTH), Wrist.MINANGLE, Wrist.MAXANGLE, Wrist.MASS, true));
    
        
        //controller initialization

        shoulderController = new Feedforward.Motor(Shoulder.Feedforward.kS, Shoulder.Feedforward.kA, Shoulder.Feedforward.kV).position()
            .add(new FeedforwardArm(Shoulder.Feedforward.kG))
            .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
            .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));
        
        wristController = new Feedforward.Motor(Wrist.Feedforward.kS, Wrist.Feedforward.kA, Wrist.Feedforward.kV).position()
            .add(new FeedforwardArm(Wrist.Feedforward.kG))
            .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
            // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
            .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));


        // ligament initialization
        arm = new Mechanism2d(16, 8);
        shoulderRoot = arm.getRoot("Arm Root", 8, 5.2);
        wristRoot = arm.getRoot("Wrist Root", 8, 4);
        swerveRoot = arm.getRoot("Swerve Root", 8-((12+3.5)/10), 0);
        pegRootMid = arm.getRoot("Peg Root Mid", (80+22.75+(12+3.5)) / 10, 0);
        pegRootTop = arm.getRoot("Peg Root Top", (80+36.75+(12+3.5)) / 10, 0);
        // Low Peg: 2 ft 10 inch
        // High Peg: 3 ft 10 inch
        // Height of base: 48.245 inches done
        // Height of robot: 51 inches
        // Length of arm: 42 inches
        // Length of wrist: 17 inches
        // Width of robot: 24 inches
        
        // 1 : 10 inches 

        pegMid = new MechanismLigament2d("Peg Mid", 3.4, 90);
        pegTop = new MechanismLigament2d("Peg Top", 4.4, 90);
        baseLigament = new MechanismLigament2d("Base", 5.2, 0);
        swerveLigamentRight = new MechanismLigament2d("SwerveRight", (31)/10, 0);

        wristLigament = new MechanismLigament2d("Wrist", Wrist.LENGTH / 10, getWristDegrees());
        shoulderLigament = new MechanismLigament2d("Arm", Shoulder.LENGTH / 10, getShoulderDegrees());
        baseLigament.setColor(new Color8Bit(0, 255, 0));
        shoulderLigament.setColor(new Color8Bit(255, 0, 255));
        wristLigament.setColor(new Color8Bit(0, 0, 255));

        pegRootMid.append(pegMid);
        pegRootTop.append(pegTop);
        swerveRoot.append(swerveLigamentRight);
        shoulderRoot.append(baseLigament);
        shoulderRoot.append(shoulderLigament);
        wristRoot.append(wristLigament);

        shoulderTargetAngle = new SmartNumber("Arm/Target Arm Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle", 0);

        SmartDashboard.putData("Arm Mech2d", arm);

    }

    @Override
    public double getShoulderDegrees() {
        return armSim.getShoulderAngleDegrees();
    }

    @Override
    public double getWristDegrees() {
        return armSim.getWristAngleDegrees();
    }

    @Override
    public void setTargetShoulderAngle(double angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Shoulder.MINANGLE), Math.toDegrees(Shoulder.MAXANGLE)));
    }

    @Override
    public void setTargetWristAngle(double angle) {
        wristTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Wrist.MINANGLE), Math.toDegrees(Wrist.MAXANGLE)));
    }
    public void setTargetWristAngle(double angle, boolean clockwise) {
        double clamped = MathUtil.clamp(angle, Math.toDegrees(Wrist.MINANGLE), Math.toDegrees(Wrist.MAXANGLE));

        if (clockwise) {
            
        }

        // wristTargetAngle.set();
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
        armSim.setInput(shoulderController.update(shoulderTargetAngle.get(), getShoulderDegrees()), wristController.update(wristTargetAngle.get(), getWristDegrees()));

        armSim.update(0.02);

        wristRoot.setPosition(8 + (Shoulder.LENGTH/10)*Math.cos(Units.degreesToRadians(getShoulderDegrees())),  
        5.2 + (Shoulder.LENGTH/10)*Math.sin(Units.degreesToRadians(getShoulderDegrees())));

        baseLigament.setAngle(-90);
        shoulderLigament.setAngle(getShoulderDegrees());
        wristLigament.setAngle(getWristDegrees());

//sus
        SmartDashboard.putNumber("Arm/Arm Angle", getShoulderDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristDegrees());
        SmartDashboard.putNumber("Arm/Arm Voltage", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}
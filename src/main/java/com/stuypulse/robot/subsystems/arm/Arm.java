// package com.stuypulse.robot.subsystems.arm;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import static com.stuypulse.robot.constants.Motors.Arm.*;
// import static com.stuypulse.robot.constants.Ports.Arm.*;
// import static com.stuypulse.robot.constants.Settings.Arm.*;
// import com.stuypulse.robot.subsystems.IArm;
// import com.stuypulse.stuylib.control.Controller;
// import com.stuypulse.stuylib.control.feedback.PIDController;
// import com.stuypulse.stuylib.control.feedforward.Feedforward;
// import com.stuypulse.stuylib.math.Angle;
// import com.stuypulse.stuylib.network.SmartAngle;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Arm extends IArm {
    
//     private final CANSparkMax shoulderLeft;
//     private final CANSparkMax shoulderRight;
//     private final CANSparkMax wrist;

//     // todo: ask verit
//     private final RelativeEncoder shoulderEncoder;
//     private final RelativeEncoder wristEncoder;

//     private final Controller shoulderController;
//     private final Controller wristController;

//     // degrees
//     private final SmartAngle shoulderTargetAngle;
//     private final SmartAngle wristTargetAngle; 

//     public Arm() {
//         shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
//         shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
//         wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

//         shoulderEncoder = shoulderLeft.getEncoder();
//         shoulderEncoder.setPositionConversionFactor(SHOULDER_CONVERSION);
//         wristEncoder = wrist.getEncoder();
//         wristEncoder.setPositionConversionFactor(WRIST_CONVERSION);

//         shoulderController = new Feedforward.Motor(ShoulderFeedForward.kS, ShoulderFeedForward.kA, ShoulderFeedForward.kV).position()
//             .add(new FeedforwardArm(ShoulderFeedForward.kG))
//             .add(new PIDController(ShoulderFeedback.P, ShoulderFeedback.I, ShoulderFeedback.D))
//             // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
//             .setOutputFilter(x -> MathUtil.clamp(x, -SHOULDER_MAX_VOLTAGE, +SHOULDER_MAX_VOLTAGE));

//         wristController = new Feedforward.Motor(WristFeedForward.kS, WristFeedForward.kA, WristFeedForward.kV).position()
//             .add(new FeedforwardArm(WristFeedForward.kG))
//             .add(new PIDController(WristFeedback.P, WristFeedback.I, WristFeedback.D))
//             // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
//             .setOutputFilter(x -> MathUtil.clamp(x, -WRIST_MAX_VOLTAGE, +WRIST_MAX_VOLTAGE));

//         shoulderTargetAngle = new SmartAngle("Arm/Shoulder Target Angle", Angle.kZero);
//         wristTargetAngle = new SmartAngle("Arm/Wrist Target Angle", Angle.kZero);

//         SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
//         SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
//         WRIST_CONFIG.configure(wrist);
//     }

//     // rotations * 2pi gets us radians
//     @Override
//     public double getShoulderDegrees() {
//         return shoulderEncoder.getPosition();
//     }

//     @Override
//     public double getWristDegrees() {
//         return wristEncoder.getPosition();
//     }

//     @Override
//     public void setTargetShoulderAngle(double degrees) {
//         shoulderTargetAngle.set(degrees);
//     }

//     @Override
//     public void setTargetWristAngle(double degrees) {
//         wristTargetAngle.set(degrees);
//     }

//     @Override
//     public boolean isShoulderAtAngle(double maxError) {
//         return Math.abs(getShoulderAngle().add(shoulderTargetAngle.get().negative()).toDegrees()) < maxError;
//     }

//     @Override
//     public boolean isWristAtAngle(double maxError) {
//         return Math.abs(getWristAngle().add(wristTargetAngle.get().negative()).toDegrees()) < maxError;
//     }

//     public void moveShoulder(Angle angle) {
//         shoulderTargetAngle.set(shoulderTargetAngle.get().add(angle));
//     }

//     public void moveWrist(Angle angle) {
//         wristTargetAngle.set(wristTargetAngle.get().add(angle));
//     }

//     public Angle getAbsoluteWristAngle() {
//         return Angle.k180deg.add(getShoulderAngle()).add(getWristAngle().negative());
//     }

//     private void runShoulder(double voltage) {
//         shoulderLeft.setVoltage(voltage);
//         shoulderRight.setVoltage(voltage);
//     }

//     private void runWrist(double voltage) {
//         wrist.setVoltage(voltage);
//     }

//     public void execute() {
//         double shoulderOutput = shoulderController.update(shoulderTargetAngle.get().toDegrees(), getShoulderAngle().toDegrees());
//         double wristOutput = wristController.update(wristTargetAngle.get().toDegrees(), getWristAngle().toDegrees());

//         runShoulder(shoulderOutput);
//         runWrist(wristOutput);

//         SmartDashboard.putNumber("Arm/Shoulder/Angle", getShoulderAngle().toDegrees());
//         SmartDashboard.putNumber("Arm/Wrist/Angle", getWristAngle().toDegrees());
//         SmartDashboard.putNumber("Arm/Wrist/Abs Angle", getAbsoluteWristAngle().toDegrees());
        
//         SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
//         SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
//     }
// }

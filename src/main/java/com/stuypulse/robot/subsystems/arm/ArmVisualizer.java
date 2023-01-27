package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {

    private final Mechanism2d arm;

    private MechanismLigament2d baseLigament;
    private MechanismLigament2d shoulderLigament;
    private MechanismLigament2d wristLigament;
    private MechanismLigament2d targetShoulderLigament;
    private MechanismLigament2d targetWristLigament;
    private MechanismLigament2d swerveLigamentRight;
    private MechanismLigament2d pegMid;
    private MechanismLigament2d pegTop;

    private MechanismRoot2d shoulderRoot;
    private MechanismRoot2d wristRoot;
    private MechanismRoot2d targetWristRoot;
    private MechanismRoot2d swerveRoot;
    private MechanismRoot2d pegRootMid;
    private MechanismRoot2d pegRootTop;

    public ArmVisualizer() {

        // ligament initialization
        arm = new Mechanism2d(16, 8);
        shoulderRoot = arm.getRoot("Arm Root", 8, 5.2);
        wristRoot = arm.getRoot("Wrist Root", 8, 4);
        targetWristRoot = arm.getRoot("Target Wrist Root", 8, 4);
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

        wristLigament = new MechanismLigament2d("Wrist", Units.metersToInches(Wrist.LENGTH) / 10, 0);
        shoulderLigament = new MechanismLigament2d("Arm", Units.metersToInches(Shoulder.LENGTH) / 10, 0);
        targetWristLigament = new MechanismLigament2d("Target Wrist", Units.metersToInches(Wrist.LENGTH) / 10, 0);
        targetShoulderLigament = new MechanismLigament2d("Target Arm", Units.metersToInches(Shoulder.LENGTH) / 10, 0);
        baseLigament.setColor(new Color8Bit(0, 255, 0));
        shoulderLigament.setColor(new Color8Bit(255, 0, 255));
        wristLigament.setColor(new Color8Bit(0, 0, 255));

        targetShoulderLigament.setColor(new Color8Bit(255, 100, 255));
        targetWristLigament.setColor(new Color8Bit(0, 100, 255));

        pegRootMid.append(pegMid);
        pegRootTop.append(pegTop);
        swerveRoot.append(swerveLigamentRight);
        shoulderRoot.append(baseLigament);
        shoulderRoot.append(shoulderLigament);
        wristRoot.append(wristLigament);
        shoulderRoot.append(targetShoulderLigament);
        targetWristRoot.append(targetWristLigament);

        baseLigament.setAngle(-90);
        
        SmartDashboard.putData("Arm Mech2d", arm);
    }

    public void setTargetAngles(double shoulderAngle, double wristAngle) {
        targetWristRoot.setPosition(8 + (Units.metersToInches(Shoulder.LENGTH)/10)*Math.cos(Units.degreesToRadians(shoulderAngle)),  
                                        5.2 + (Units.metersToInches(Shoulder.LENGTH)/10)*Math.sin(Units.degreesToRadians(shoulderAngle)));

        targetShoulderLigament.setAngle(shoulderAngle);
        targetWristLigament.setAngle(wristAngle);
    }

    public void setMeasuredAngles(double shoulderAngle, double wristAngle) {
        wristRoot.setPosition(8 + (Units.metersToInches(Shoulder.LENGTH)/10)*Math.cos(Units.degreesToRadians(shoulderAngle)),  
                                5.2 + (Units.metersToInches(Shoulder.LENGTH)/10)*Math.sin(Units.degreesToRadians(shoulderAngle)));

        shoulderLigament.setAngle(shoulderAngle);
        wristLigament.setAngle(wristAngle);
    }
}

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class FieldArm2d {
    private final FieldObject2d object;

    public FieldArm2d(FieldObject2d object) {
        this.object = object;
    }

    public void update(Pose2d robot, ArmState state) {
        // Visualize the arm on the field
        double distanceFromSwerveCenter = state.getShoulderAngle().getCos() * Shoulder.LENGTH + state.getWristAngle().getCos() * Wrist.LENGTH;

        Translation2d topDownTranslation = new Translation2d(distanceFromSwerveCenter, robot.getRotation());
        
        object.setPose(new Pose2d(
            topDownTranslation.plus(robot.getTranslation()),
            robot.getRotation()
        ));
    }
}

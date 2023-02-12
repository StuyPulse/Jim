package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;

public class AstarImp {

    public AstarImp(int resolutionx, int resolutiony, double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
    }




    public void addConstraints() {
        // todo finish
    }

    public static ArmTrajectory generateTrajectory(ArmState start, ArmState end) {
        return generateTrajectory(
            start.getShoulderState().getDegrees(),
            start.getWristState().getDegrees(),
            end.getShoulderState().getDegrees(),
            end.getWristState().getDegrees()
        );
    }

    // TODO: Take in resolution multiplier
    public static ArmTrajectory generateTrajectory(double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
        // convert a negative angle into a usable angle
        
        Astar astar = new Astar(
                new Node(initialShoulderAngle, initialWristAngle), 
                new Node(finalShoulderAngle, finalWristAngle)
            );
        


        astar.addConstraint((s, w) -> 0 <= s && s <= 180);
        astar.addConstraint((s, w) -> Math.abs(s - (-90)) < 30 && ((-180 <= w && w <= 10)));

        List<Node> path = astar.findPath();
        for (Node node : path) {
            System.out.println(node);
        }
        
        
        ArmTrajectory trajectory = new ArmTrajectory();

        for (Node node : path) {
            trajectory.addState(node.toArmState()); // CHANGE IMPORTANT
        }
        return trajectory;
    }

    public static Constraint generateHorizontalLineConstraint(int startX, int startY, int length) {
        return (s, w) -> {
            return (startX <= s && s <= startX + length) && w == startY;
        };
    }

    public static Constraint generateVerticalLineConstraint(int startX, int startY, int length) {
        return (s, w) -> {
            return s == startX && (startY <= w && w <= startY + length);
        };
    }

    
    



}

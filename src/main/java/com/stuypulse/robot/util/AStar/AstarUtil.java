package com.stuypulse.robot.util.AStar;

import java.util.List;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;


public class AstarUtil {
 
    public static ArmTrajectory generateTrajectory(Constraint constraint, ArmState... states) {
        ArmTrajectory trajectory = new ArmTrajectory();

        for (int i = 0; i < states.length - 1; i += 2) {
            trajectory.append(generateTrajectory(constraint, states[i], states[i+1]));
        }

        return trajectory;
    }

    public static ArmTrajectory generateTrajectory(Constraint constraint, ArmState start, ArmState end) {
        return generateTrajectory(
            constraint,
            start.getShoulderState().getDegrees(),
            start.getWristState().getDegrees(),
            end.getShoulderState().getDegrees(),
            end.getWristState().getDegrees()
        );
    }

    public static ArmTrajectory generateTrajectory(
        Constraint constraint, 
        double initialShoulderAngle, 
        double initialWristAngle, 
        double finalShoulderAngle, 
        double finalWristAngle) 
    {
        
        Astar astar = new Astar(
                new Node(initialShoulderAngle, initialWristAngle), 
                new Node(finalShoulderAngle, finalWristAngle)
            );
        

        astar.addConstraint(constraint);

        // add constraints to astar path (example constraints)
        // astar.addConstraint((s, w) -> 0 <= s && s <= 180);
        // astar.addConstraint((s, w) -> Math.abs(s - (-90)) < 30 && ((-180 <= w && w <= 10)));

        List<Node> path = astar.findPath();
        // for (Node node : path) {
            // System.out.println(node);
        // }
        
        ArmTrajectory trajectory = new ArmTrajectory();

        for (Node node : path) {
            trajectory.addState(node.toArmState());
        }
        return trajectory;
    }

    public static Constraint getHorizontalConstraint(int startX, int startY, int length) {
        return (s, w) -> {
            return (startX <= s && s <= startX + length) && w == startY;
        };
    }

    public static Constraint getVerticalConstraint(int startX, int startY, int length) {
        return (s, w) -> {
            return s == startX && (startY <= w && w <= startY + length);
        };
    }

    
 
    
    



}

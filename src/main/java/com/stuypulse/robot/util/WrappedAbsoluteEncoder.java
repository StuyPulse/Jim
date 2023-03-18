package com.stuypulse.robot.util;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

// TODO: implement AbsolulteEncoder??
public class WrappedAbsoluteEncoder {
    
    private final AbsoluteEncoder encoder;
    private double previousRotation;
    private double totalRotations;

    public WrappedAbsoluteEncoder(AbsoluteEncoder encoder) {
        this.encoder = encoder;

        previousRotation = getUnwrappedRotations();
    }

    private double getUnwrappedRotations() {
        return encoder.getPosition();
    }

    public double getRotations() {
        return totalRotations;
    }

    public AbsoluteEncoder getEncoder() {
        return encoder;
    }

    public void update() {
        double currentRotations = getUnwrappedRotations();
        
        Rotation2d delta = Rotation2d.fromRotations(currentRotations)
            .minus(Rotation2d.fromRotations(previousRotation));
        totalRotations += delta.getRotations();

        previousRotation = currentRotations;
    }

}

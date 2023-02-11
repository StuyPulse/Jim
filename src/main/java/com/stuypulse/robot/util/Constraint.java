package com.stuypulse.robot.util;

public interface Constraint {

    public boolean isInvalid(int shoulderDeg, int wristDeg);
    
    public default Constraint add(Constraint other) {
        return (s, w) -> this.isInvalid(s, w) || other.isInvalid(s, w);
    }


}
package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.IStream;

public class ArmDriveFeedforward extends AngleController {

    private final Number kG;
    private final IStream forwardAccelerationInGs;

    public ArmDriveFeedforward(Number kG, IStream forwardAccelerationInGs) {
        this.kG = kG;
        this.forwardAccelerationInGs = forwardAccelerationInGs;
    }

    @Override
    protected double calculate(Angle setpoint, Angle measurement) {
        return kG.doubleValue() * measurement.sin() * forwardAccelerationInGs.get();
    }
    
}

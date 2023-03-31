package com.stuypulse.robot.subsystems.LED;

import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// idk?
public abstract class LED extends SubsystemBase {

    //public static final LEDController instance;

    static {
        if(Robot.isSimulation()){
            final SimLED instance = new SimLED();
        }
        else{
             final LEDImpl instance = new LEDImpl();
        }
    }

    private void LEDController(){};

    public void setColor(LEDColor color, double time){};

    private void forceSetLEDs(LEDColor color){};

    private void setLEDConditions(){};

    public LEDColor getDefaultColor(){
        return null;};

    public void periodic(){};

}

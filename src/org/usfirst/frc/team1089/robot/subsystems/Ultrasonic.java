package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.DriveToWall;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Ultrasonic extends Subsystem {
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private AnalogInput ultrasonic;
	
	/** Scaling factor - in / volts */
	private final double SCALING_FACTOR = 1/9.8; 
	
    public Ultrasonic() {
    	ultrasonic = new AnalogInput(RobotMap.Analog.ULTRASONIC);
    }
    
    public AnalogInput getUltrasonic() {
    	return ultrasonic;
    }
    
    /**
     * Gets the range between the rangefinder board and the object across from it
     * as perceived by the rangefinder by getting the voltage and multiplying it by the scaling factor
     * 
     * @return range between the board and the object across from it in inches
     */
    public double getRange() {
    	return ultrasonic.getVoltage() * SCALING_FACTOR;
    }
    
    public void initDefaultCommand() {
        //setDefaultCommand(new DriveToWall());
    }
    
}


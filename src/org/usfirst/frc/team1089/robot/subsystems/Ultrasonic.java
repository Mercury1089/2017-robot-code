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
	private double min_voltage;	//Minimum voltage the ultrasonic sensor can return
	private double voltage_range; //The range of the voltages returned by the sensor
	private double min_distance; //Minimum distance the ultrasonic sensor can return in ft
	private double distance_range;//The range of the distances returned by this class in ft
	
	/** Scaling factor - inches / volts */
	private final double SCALING_FACTOR = 1/9.8; 
	
    public Ultrasonic() {
    	ultrasonic = new AnalogInput(RobotMap.Analog.ULTRASONIC);
    	min_voltage = .5;
    	voltage_range = 5.0 - min_voltage;
    	min_distance = 0.25;
    	distance_range = 5 - min_distance;
    }
    
    public AnalogInput getUltrasonic() {
    	return ultrasonic;
    }
    
    /**
     * Gets the range between the rangefinder board and the object across from it
     * as perceived by the rangefinder by getting the voltage and multiplying it by the scaling factor
     * 
     * 
     * @return range between the board and the object across from it in inches
     */
    public double getRange() {
//    	return ultrasonic.getVoltage() * SCALING_FACTOR;
    	double range = ultrasonic.getVoltage();
    	//first, normalize the voltage
    	range = (range - min_voltage) / voltage_range;
    	//next, denormalize to the unit range
    	range = (range * distance_range) + min_distance;
    	return range;
    }
    
    public void initDefaultCommand() {
        //setDefaultCommand(new DriveToWall());
    }
    
}


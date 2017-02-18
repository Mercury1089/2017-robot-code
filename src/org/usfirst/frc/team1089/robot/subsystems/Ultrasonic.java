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
	private final double SCALING_FACTOR = 1/9.8;
	
    public Ultrasonic() {
    	ultrasonic = new AnalogInput(RobotMap.Analog.ULTRASONIC);
    }
    
    public AnalogInput getUltrasonic() {
    	return ultrasonic;
    }
    
    /**
     * Multiplies Analog Voltage on the sensor by the scaling factor in inches/V
     * Scaling Factor = 9.8 V/inches
     * @return Distance
     */
    public double getRange() {
    	return ultrasonic.getVoltage() * SCALING_FACTOR;
    }
    
    public void initDefaultCommand() {
        //setDefaultCommand(new DriveToWall());
    }
    
}


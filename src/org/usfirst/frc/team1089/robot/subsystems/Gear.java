package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.ToggleGearDelivery;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Gear Delivery Mechanism
 * 
 * @author Luke Letourneau
 * @version 1
 * 
 * {@link}
 */
public class Gear extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private Servo servo;
	
	private final double CLOSED_POSITION, OPEN_POSITION;
	
	public Gear() {
		servo = new Servo(RobotMap.PWM.SERVO_ID);
		CLOSED_POSITION = 0.75; //TODO Edit these values
		OPEN_POSITION = 0.4;  //TODO Edit these values
	}

    public void initDefaultCommand() {
    }
    
    public Servo getServo() {
    	return servo;
    }
    
    public double getServoPosition(){
    	return servo.get();
    }
    
    public void setServoPosition(double position) {
    	servo.set(position);
    }
    
    public double getClosedPosition() {
    	return CLOSED_POSITION;
    }
    
    public double getOpenPosition() {
    	return OPEN_POSITION;
    }
}



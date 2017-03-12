package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This {@link Subsystem} encapsulates everything we need to drive:
 * motor controllers, gyros, and the {@link RobotDrive} class we need to interface
 * with the system.
 * 
 * @see PIDOutput
 * @see AnalogGyro
 * @see CANTalon
 */
public class Climber extends Subsystem {
	
	private CANTalon motor;
	
	public Climber(int ID) {
		motor = new CANTalon(ID);
		//motor.enableBrakeMode(false);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    }
    
    public void runClimber(boolean run) {
    	motor.set(run ? -1 : 0);
    }
	
}
package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.RunAgitator;
import org.usfirst.frc.team1089.robot.commands.RunFeeder;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Agitator extends Subsystem {

	public CANTalon motor;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Agitator(int ID) {
		motor = new CANTalon(ID);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new RunAgitator(Robot.leftShooter, Robot.rightShooter));
    }
}
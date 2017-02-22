package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.commands.SetRoller;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

	public CANTalon motor;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Intake(int ID) {
		motor = new CANTalon(ID);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new SetRoller(this,0)); 	
    }
}


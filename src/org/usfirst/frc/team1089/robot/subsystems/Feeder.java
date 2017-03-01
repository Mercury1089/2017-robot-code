package org.usfirst.frc.team1089.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Feeder extends Subsystem {

	public CANTalon motor;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Feeder(int ID) {
		motor = new CANTalon(ID);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    }
}


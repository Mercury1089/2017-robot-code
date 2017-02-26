package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.subsystems.Intake;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetRoller extends Command {

	private Intake roller;
	private int speed;
	
    public SetRoller(Intake i, int s) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(i);
    	roller = i;
    	speed = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	roller.motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	roller.motor.enableBrakeMode(true);
    	roller.motor.enableControl();
    	MercLogger.logMessage(Level.INFO, "SetRoller: Initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	roller.motor.set(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "SetRoller: Completed");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "SetRoller: Interrupted");
    }
}

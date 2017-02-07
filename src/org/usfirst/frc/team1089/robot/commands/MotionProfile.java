package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MotionProfileExample;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionProfile extends Command {

	public static final int kNumPoints = 185;
	CANTalon l, r;
	MotionProfileExample example;
	
    public MotionProfile(CANTalon left) {

    	l = left;
    	//r = right;
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	l.changeControlMode(TalonControlMode.MotionProfile);
    	//r.changeControlMode(TalonControlMode.MotionProfile);
    	//requires(Robot.driveTrain);
    	example = new MotionProfileExample(l);
    	example.reset();
    	example.startMotionProfile();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	example.control();
    	CANTalon.SetValueMotionProfile setOutput = example.getSetValue();
    	l.set(setOutput.value);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

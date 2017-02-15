package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercPathPlanner;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestCommand extends Command {

	CANTalon leftTalon, rightTalon;
	
	MercPathPlanner path;
	
	double totalTime = 8; //seconds
	double timeStep = 0.1; //period of control loop on Rio, seconds
	double robotTrackWidth = 2; //distance between left and right wheels, feet
	
	//create waypoint path
	double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
	};
	
    public TestCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.driveTrain);
    	
    	leftTalon = Robot.driveTrain.getLeft();
    	rightTalon = Robot.driveTrain.getRight();
    	path = new MercPathPlanner(waypoints);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	path.calculate(totalTime, timeStep, robotTrackWidth);
    	
    	leftTalon.set(path.smoothLeftVelocity[0][0]);
    	rightTalon.set(path.smoothRightVelocity[0][1]);
    	
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

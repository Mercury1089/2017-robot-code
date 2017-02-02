package org.usfirst.frc.team1089.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.logging.Level;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.VisionSystem.GOAL_TYPE;
import org.usfirst.frc.team1089.robot.util.Debug;
import org.usfirst.frc.team1089.robot.util.MercPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;

public class GetDistanceFromTarget extends Command {	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		//requires(Robot.visionSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//Debug.logMessage(Level.INFO, "Distance (ft): " + Robot.visionSystem.getDistance(GOAL_TYPE.GEAR));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
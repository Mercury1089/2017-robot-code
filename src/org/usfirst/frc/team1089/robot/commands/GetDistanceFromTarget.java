package org.usfirst.frc.team1089.robot.commands;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;


public class GetDistanceFromTarget extends Command {
	private Thread vThread;
	private Object imgLock;
	private GRIPPipeline pipeline;
	
	private final int NUM_TARGETS = 2;
	private final double 
		TARGET_WIDTH_INCHES = 10.75,
		TARGET_HEIGHT_INCHES = 5,
		TARGET_ELEVATION_FEET = 10.75,
		IN_TO_FT = 12.0; // Study your freedom units guys
	private final double[] DEF_VAL = {-1};
	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.visionSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Get the screen res
		int
			IMG_WIDTH = (int)NetworkTable.getTable("GRIP/screenSize").getNumberArray("width", DEF_VAL)[0] + 2,
			IMG_HEIGHT = (int)NetworkTable.getTable("GRIP/screenSize").getNumberArray("height", DEF_VAL)[0] + 2;
		
		// Get all target widths and heights
		double[] 
			ntTargetWidth = NetworkTable.getTable("GRIP/target").getNumberArray("width", DEF_VAL),
			ntTargetHeight = NetworkTable.getTable("GRIP/target").getNumberArray("height", DEF_VAL),
			ntTargetCenterX = NetworkTable.getTable("GRIP/target").getNumberArray("centerX", DEF_VAL),
			ntTargetCenterY = NetworkTable.getTable("GRIP/target").getNumberArray("centerY", DEF_VAL);
		
		if (ntTargetWidth.length == NUM_TARGETS && ntTargetHeight.length == NUM_TARGETS
			&& ntTargetCenterX.length == NUM_TARGETS && ntTargetCenterY.length == NUM_TARGETS) {
			double
				targetWidth = Math.abs(ntTargetCenterX[1] + ntTargetWidth[1] - ntTargetCenterX[0] - ntTargetWidth[0]),
				targetHeight = ntTargetWidth[0] / 2 + ntTargetWidth[1] / 2;
			
			// Using proportions, we can convert the target width in pixels 
			// to the target width in feet.
			// TODO: Fix this; it does not work.
			double dist = (TARGET_WIDTH_INCHES / IN_TO_FT) * (IMG_WIDTH / targetWidth);
			dist /= 2.0;
			dist /= Math.tan(Math.toRadians(Robot.visionSystem.HFOV / 2));
			
			System.out.println("Distance (u.) = " + dist);
			System.out.println("Resolution: " + IMG_WIDTH + " x " + IMG_HEIGHT);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		vThread.interrupt();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// excuse me princess
	}
}
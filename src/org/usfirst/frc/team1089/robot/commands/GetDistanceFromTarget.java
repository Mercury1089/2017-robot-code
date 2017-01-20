package org.usfirst.frc.team1089.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

/**
 *
 */
public class GetDistanceFromTarget extends Command {
	private VisionThread vThread;
	private Object imgLock;
	private double targetWidth, targetHeight;
	
	private final int NUM_TARGETS = 2;
	private final double 
		TARGET_WIDTH_INCHES = 10.75,
		TARGET_HEIGHT_INCHES = 5,
		TARGET_ELEVATION_FEET = 10.75,
		IN_TO_FT = 12.0; // Study your freedom units guys
		
	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.targetingSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Create synchronization object
		imgLock = new Object();

		// Create VisionThread that allows
		// image processing off the main robot thread
		vThread = new VisionThread(Robot.targetingSystem.AXIS_CAMERA, new GRIPPipeline(), pipeline -> {
			if (pipeline.filterContoursOutput().size() == NUM_TARGETS) {
				MatOfPoint[] contours = pipeline.filterContoursOutput().toArray(new MatOfPoint[1]);
				Rect 
					target1 = Imgproc.boundingRect(contours[0]), 
					target2 = Imgproc.boundingRect(contours[1]);
				
				synchronized(imgLock) {
					targetWidth = target1.x + target2.x + target2.width;
					targetHeight = (target1.height + target2.height) / 2;
				}
			}
		});
		
		// Begin the thread
		vThread.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Using proportions, we can convert the target width in pixels 
		// to the target width in feet.
		// TODO: Fix this; it does not work.
		double dist = (TARGET_WIDTH_INCHES / IN_TO_FT) * (Robot.targetingSystem.IMG_WIDTH / targetWidth);
		dist /= 2.0;
		dist /= Math.tan(Math.toRadians(Robot.targetingSystem.HFOV / 2));
		
		System.out.println("Distance (u.) = " + dist);
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
		// excuse me princess
	}
}

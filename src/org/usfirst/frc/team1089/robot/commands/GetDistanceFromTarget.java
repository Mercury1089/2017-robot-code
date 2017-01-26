package org.usfirst.frc.team1089.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;

public class GetDistanceFromTarget extends Command {
	private Thread vThread;
	private MercPipeline pipeline;
	private double targetWidth, targetHeight;
	private Point center;
	
	private final int NUM_TARGETS = 2;
	private final double 
		TARGET_WIDTH_INCHES = 10.75,
		TARGET_HEIGHT_INCHES = 5,
		TARGET_ELEVATION_FEET = 10.75,
		IN_TO_FT = 12.0; // Study your freedom units guys
	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.visionSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		pipeline = new MercPipeline();
		
		
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		// Calculate target distance.
		// This is calculated with knowledge of the target width in feet, 
		// the HFOV in pixels (the width of the camera's viewport in pixels),
		// the target width as it appears in the camera feed,
		// and the physical FOV of the camera.
		double diagTargetDistance = (TARGET_WIDTH_INCHES / IN_TO_FT) * Robot.visionSystem.IMG_WIDTH 
				/ ( 2 * targetWidth * Math.tan( Math.toRadians( Robot.visionSystem.HFOV / 2 ) ) );
		
		System.out.println("Distance (ft): " + diagTargetDistance);
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
		end();
	}
}
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
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;

public class GetDistanceFromTarget extends Command {
	private Thread vThread;
	private GRIPPipeline pipeline;
	private double targetWidth, targetHeight;
	
	private final int NUM_TARGETS = 2;
	private final double 
		TARGET_WIDTH_INCHES = 10.75,
		TARGET_HEIGHT_INCHES = 5,
		TARGET_ELEVATION_FEET = 10.75,
		IN_TO_FT = 12.0; // Study your freedom units guys
	private final double[] DEF_VAL = {-1};
	private final int IMG_WIDTH = 800, IMG_HEIGHT = 600;
	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.visionSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		pipeline = new GRIPPipeline();
		
		// Set up our vision thread
		vThread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-1089.local");
			// Set the resolution
			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Vision", IMG_WIDTH, IMG_HEIGHT);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					System.out.println(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}		
						
				pipeline.process(mat);
				ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();
				if (contours.size() == NUM_TARGETS) {					
					Rect target1, target2;
					
					if (Imgproc.boundingRect(contours.get(1)).x < Imgproc.boundingRect(contours.get(0)).x) {
						target1 = Imgproc.boundingRect(contours.get(1));
						target2 = Imgproc.boundingRect(contours.get(0));
					} else {
						target1 = Imgproc.boundingRect(contours.get(0));
						target2 = Imgproc.boundingRect(contours.get(1));
					}
				
					targetWidth = target2.x + target2.width - target1.x;
					targetHeight = target2.y / 2 + target1.y / 2;	
					
					Imgproc.rectangle(
							mat, 
							new Point(target1.x, target1.y), 
							new Point(target2.x + target2.width, target2.y + target2.height), 
							new Scalar(0, 0, 255), 
							10
					);
				}
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		
		vThread.setDaemon(true);
		vThread.start();
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
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
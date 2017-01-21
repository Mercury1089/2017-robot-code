package org.usfirst.frc.team1089.robot.commands;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;


public class GetDistanceFromTarget extends Command {
	private Thread vThread;
	private Object imgLock;
	private GRIPPipeline pipeline;
	private double targetWidth, targetHeight;
	
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
		// Create synchronization object
		imgLock = new Object();
		
		// Create pipeline
		pipeline = new GRIPPipeline();
		
		vThread = new Thread(() -> {
			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

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
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Let's process the image for the target
				pipeline.process(mat);
				
				// Only if we find the correct number of targets
				// will we even bother doing math.
				MatOfPoint[] contours = pipeline.filterContoursOutput().toArray(new MatOfPoint[1]);
				if (contours.length == NUM_TARGETS) {
					Rect 
						target1 = Imgproc.boundingRect(contours[0]), 
						target2 = Imgproc.boundingRect(contours[1]);
					
					targetWidth = target2.x + target2.width - target1.x; // Full width between targets
					targetHeight = target1.x / 2 + target2.x / 2; // Get the average height
					
					// Draw a rectangle around what we found so we can see what's been found.
					Imgproc.rectangle(mat, 
							new Point(target1.x, target1.y), 
							new Point(target2.x, target2.y + target2.height), 
							new Scalar(255, 0, 0), 2);
				} else {
					System.out.println("Targets not found! Number of targets found: " + contours.length);
				}
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		
		// Create VisionThread that allows
		// image processing off the main robot thread
		/*vThread = new VisionThread(Robot.targetingSystem.AXIS_CAMERA, new GRIPPipeline(), pipeline -> {
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
		});*/
		
		
		
		// Make the thread a daemon and 
		// begin the thread
		vThread.setDaemon(true);
		vThread.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Using proportions, we can convert the target width in pixels 
		// to the target width in feet.
		// TODO: Fix this; it does not work.
		double dist = (TARGET_WIDTH_INCHES / IN_TO_FT) * (Robot.visionSystem.IMG_WIDTH / targetWidth);
		dist /= 2.0;
		dist /= Math.tan(Math.toRadians(Robot.visionSystem.HFOV / 2));
		
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
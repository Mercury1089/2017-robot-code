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
	private final double[] DEF_VAL = {-1};
	private final int IMG_WIDTH = 800, IMG_HEIGHT = 600;
	
	public GetDistanceFromTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.visionSystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		pipeline = new MercPipeline();
		
		// Set up our vision thread
		vThread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-1089.local");
			// Set the resolution
			camera.setResolution(Robot.visionSystem.IMG_WIDTH, Robot.visionSystem.IMG_HEIGHT);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Vision", Robot.visionSystem.IMG_WIDTH, Robot.visionSystem.IMG_HEIGHT);

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
					
					// Our targeting rect needs to encapsulate both vision targets
					Point 
						topLeft = new Point(target1.x, target1.y < target2.y ? target1.y : target2.y),
						bottomRight = new Point(target2.x + target2.width, target1.y < target2.y ? target2.y + target2.height : target1.y + target1.height);
					
					Scalar red = new Scalar(0, 0, 255);
				
					targetWidth = bottomRight.x - topLeft.x;
					targetHeight = bottomRight.y - topLeft.y;
					
					// Get the center of the target
					// and check if we are centered
					center = new Point(topLeft.x + targetWidth / 2, topLeft.y + targetHeight / 2);
					boolean isCentered = Math.abs(center.x - Robot.visionSystem.IMG_WIDTH / 2) <= 5;
					System.out.println("Centered: " + isCentered);
					
					// Draw target
					Imgproc.rectangle(
							mat, 
							topLeft, 
							bottomRight, 
							red, 
							3
					);
					
					Imgproc.line(mat, new Point(center.x, center.y - 5), new Point(center.x, center.y + 5), red, 3);
					Imgproc.line(mat, new Point(center.x - 5, center.y), new Point(center.x + 5, center.y), red, 3);
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

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// excuse me princess
	}
}
package org.usfirst.frc.team1089.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;
import org.usfirst.frc.team1089.robot.util.MercPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionSystem extends Subsystem {
	private final MercPipeline PIPELINE;
	
	public final double HFOV = 58.0; // The horizontal FOV of the camera. TODO: Calibrate values
	public final int IMG_WIDTH = 640, IMG_HEIGHT = 480;
	private final int NUM_TARGETS = 2;
	
	// Targeting member
	private double targetWidth, targetHeight;
	private final Point TARGET_TOP_LEFT, TARGET_BOT_RIGHT, TARGET_CENTER;
	
	private final Thread V_THREAD;
	public final CvSource OUTPUT_STREAM;
	public final CvSink CV_SINK;
	
	public VisionSystem() {
		// Initialize vision pipeline
		PIPELINE = new MercPipeline();
		
		// Get camera feed and output stream
		CameraServer.getInstance().addAxisCamera("axis-1089.local").setResolution(IMG_WIDTH, IMG_HEIGHT);
		OUTPUT_STREAM = CameraServer.getInstance().putVideo("Vision", IMG_WIDTH, IMG_HEIGHT);
		CV_SINK = CameraServer.getInstance().getVideo();
		
		// Initialize target points
		TARGET_CENTER = new Point();
		TARGET_TOP_LEFT = new Point();
		TARGET_BOT_RIGHT = new Point();
		
		// Process everything
		V_THREAD = new Thread(() -> {
			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();
	
			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (Robot.visionSystem.CV_SINK.grabFrame(mat) == 0) {
					// Send the output the error.
					System.out.println(Robot.visionSystem.CV_SINK.getError());
					// skip the rest of the current iteration
					continue;
				}		
						
				PIPELINE.process(mat);
				ArrayList<MatOfPoint> contours = PIPELINE.filterContoursOutput();
				
				if (contours.size() == NUM_TARGETS) {					
					Rect target1, target2;
					
					if (Imgproc.boundingRect(contours.get(1)).x < Imgproc.boundingRect(contours.get(0)).x) {
						target1 = Imgproc.boundingRect(contours.get(1));
						target2 = Imgproc.boundingRect(contours.get(0));
					} else {
						target1 = Imgproc.boundingRect(contours.get(0));
						target2 = Imgproc.boundingRect(contours.get(1));
					}
					
					Scalar red = new Scalar(0, 0, 255);
					
					// Our targeting rect needs to encapsulate both vision targets
					TARGET_TOP_LEFT.set(new double[] {
							target1.x, 
							target1.y < target2.y ? target1.y : target2.y
					});
					
					TARGET_BOT_RIGHT.set(new double[] {
							target2.x + target2.width, 
							target1.y < target2.y ? target2.y + target2.height : target1.y + target1.height
					});
				
					targetWidth = TARGET_BOT_RIGHT.x - TARGET_TOP_LEFT.x;
					targetHeight = TARGET_BOT_RIGHT.y - TARGET_TOP_LEFT.y;
					
					// Get the center of the target
					// and check if we are centered
					TARGET_CENTER.set(new double[]{TARGET_TOP_LEFT.x + targetWidth / 2, TARGET_TOP_LEFT.y + targetHeight / 2});
					boolean isCentered = Math.abs(TARGET_CENTER.x - IMG_WIDTH / 2) <= 5;
					System.out.println("Centered: " + isCentered);
					
					// Draw target
					Imgproc.rectangle(
							mat, 
							TARGET_TOP_LEFT, 
							TARGET_BOT_RIGHT, 
							red, 
							3
					);
					
					Imgproc.line(
							mat, 
							new Point(TARGET_CENTER.x, TARGET_CENTER.y - 5), 
							new Point(TARGET_CENTER.x, TARGET_CENTER.y + 5), 
							red, 
							3
					);
					
					Imgproc.line(
							mat, 
							new Point(TARGET_CENTER.x - 5, TARGET_CENTER.y), 
							new Point(TARGET_CENTER.x + 5, TARGET_CENTER.y), 
							red, 
							3
					);
				}
				
				// Give the output stream a new image to display
				OUTPUT_STREAM.putFrame(mat);
			}
		});
		
		V_THREAD.setDaemon(true);
		V_THREAD.start();
	}
	
	public double getDistance() {
		return 0;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

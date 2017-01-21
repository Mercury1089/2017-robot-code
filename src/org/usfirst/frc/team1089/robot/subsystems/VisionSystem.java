package org.usfirst.frc.team1089.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionSystem extends Subsystem {
	public final double HFOV = 67.0; // The horizontal FOV of the camera. TODO: Calibrate values
	public final double[] DEF = {-1};
	
	// Initializes the AXIS camera and 
	// sets the proper resolution
	public VisionSystem() {

	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

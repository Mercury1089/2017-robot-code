package org.usfirst.frc.team1089.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionSystem extends Subsystem {
	public final double HFOV = 67.0; // The horizontal FOV of the camera. TODO: Calibrate values
	private final int NUM_TARGETS = 2;
	public final GRIPPipeline PIPELINE;
	private double targetWidth, targetHeight;
	
	public VisionSystem() {
		PIPELINE = new GRIPPipeline();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

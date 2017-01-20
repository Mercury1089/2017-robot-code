package org.usfirst.frc.team1089.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;
import org.usfirst.frc.team1089.robot.util.GRIPPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.wpilibj.vision.VisionThread;


public class VisionSystem extends Subsystem {
	public final AxisCamera AXIS_CAMERA;
	public final int
		IMG_WIDTH = 320,
		IMG_HEIGHT = 240;
	public final double HFOV = 58.0; // The horizontal FOV of the camera. TODO: Calibrate values
	
	// Initializes the AXIS camera and 
	// sets the proper resolution
	public VisionSystem() {
		// Set up axis camera
		AXIS_CAMERA = CameraServer.getInstance().addAxisCamera("axis-1089.local");
		AXIS_CAMERA.setResolution(IMG_WIDTH, IMG_HEIGHT);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

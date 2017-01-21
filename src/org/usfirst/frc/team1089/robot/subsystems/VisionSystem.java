package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionSystem extends Subsystem {
	public final CameraServer CAM_SERVER;
	public final AxisCamera AXIS_CAMERA;
	public final int
		IMG_WIDTH = 320,
		IMG_HEIGHT = 240;
	public final double HFOV = 58.0; // The horizontal FOV of the camera. TODO: Calibrate values
	
	// Initializes the AXIS camera and 
	// sets the proper resolution
	public VisionSystem() {
		// Set up camera server
		CAM_SERVER = CameraServer.getInstance();
		// Set up axis camera
		AXIS_CAMERA = CAM_SERVER.addAxisCamera("axis-1089.local");
		AXIS_CAMERA.setResolution(IMG_WIDTH, IMG_HEIGHT);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

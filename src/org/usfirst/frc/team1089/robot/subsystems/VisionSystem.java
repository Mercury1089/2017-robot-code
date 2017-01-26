package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.commands.GetDistanceFromTarget;

import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionSystem extends Subsystem {
	public final double HFOV = 67.0; // The horizontal FOV of the camera. TODO: Calibrate values
	public final int IMG_WIDTH = 640, IMG_HEIGHT = 480;
	private final int NUM_TARGETS = 2;
	private double targetWidth, targetHeight;
	
	public VisionSystem() {
		
	}

	public void initDefaultCommand() {
		setDefaultCommand(new GetDistanceFromTarget());
	}
}

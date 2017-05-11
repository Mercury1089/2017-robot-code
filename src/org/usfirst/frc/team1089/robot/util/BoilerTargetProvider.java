package org.usfirst.frc.team1089.robot.util;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

public class BoilerTargetProvider implements ITargetProvider{
	private VisionProcessor visionProcessor;
	
	public BoilerTargetProvider(VisionProcessor visionProcessor) {
		this.visionProcessor = visionProcessor;
	}
	
	@Override
	public double getDistance() {
		return visionProcessor.getDistanceUsingVerticalInformation(TargetType.HIGH_GOAL);
	}

	@Override
	public boolean isOnTarget() {
		return visionProcessor.isOnTarget(TargetType.HIGH_GOAL);
	}
	
}

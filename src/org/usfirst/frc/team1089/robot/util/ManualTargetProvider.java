package org.usfirst.frc.team1089.robot.util;

import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class ManualTargetProvider implements ITargetProvider {
	
	@Override
	public double getDistance() {
		return 10;
	}

	@Override
	public boolean isOnTarget() {
		return true;
	}
}

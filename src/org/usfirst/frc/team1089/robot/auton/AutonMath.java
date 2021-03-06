package org.usfirst.frc.team1089.robot.auton;

import org.usfirst.frc.team1089.robot.util.Preferences;

public class AutonMath {
	private static final double AIRSHIP_DISTANCE = 150.05 / 12, AIRSHIP_RADIUS = 35.75 / 12;		//feet
	
	public static double[] autonDistances(int position) {
		double[] d = new double[2];
		double midlineDist;
		
		switch(position) {
			case 1:
				midlineDist = 10.2338 - Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			case 2:
				midlineDist = 6.6629 + Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			case 3:
				midlineDist = 6.6629 - Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			case 4:
			case 5:
			case 6:
				midlineDist = 0;
				break;
			case 7:
				midlineDist = 4.4855 - Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			case 8:
				midlineDist = 4.4855 + Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			case 9:
				midlineDist = 10.3645 - Preferences.ROBOT_LENGTH_COMPETITION / 2;
				break;
			default:
				midlineDist = 0;
				break;
		}
		
		double degTheta = 30;
		double radTheta = Math.toRadians(degTheta);
		double tanTheta = Math.tan(radTheta);
		double distanceFromTheta = AIRSHIP_DISTANCE / tanTheta - midlineDist; //Distance from the corner of the 30 degree angle
		
		d[0] = distanceFromTheta * tanTheta - (1 / 2 * Preferences.ROBOT_LENGTH_PROTO); //Distance to drive forward off wall
		d[1] = (AIRSHIP_DISTANCE / Math.sin(radTheta)) - (d[0] / Math.sin(radTheta)) - AIRSHIP_RADIUS; //Distance to drive to gear; Will use auto align so may become deprecated 
		return d;
	}
	
	public static double alignToGear(double angleOff)
	{
		return -angleOff;
	}
}

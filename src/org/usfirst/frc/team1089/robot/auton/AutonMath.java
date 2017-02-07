package org.usfirst.frc.team1089.robot.auton;

import org.usfirst.frc.team1089.robot.util.Config;

public class AutonMath {
	private static final double AIRSHIP_DISTANCE = 150.05, AIRSHIP_RADIUS = 35.75;		//inches
	
	public static double[] autonDistances(int position) {
		double[] d = new double[2];
		double midlineDist;
		
		switch(position) {
		case 1:
			midlineDist = 144;
			break;
		case 2:
			midlineDist = 108;
			break;
		case 3:
			midlineDist = 84;
			break;
		case 4:
		case 5:
		case 6:
			midlineDist = 0;
			break;
		case 7:
			midlineDist = 36;
			break;
		case 8:
			midlineDist = 60;
			break;
		case 9:
			midlineDist = 132;
			break;
		default:
			midlineDist = 0;
			break;
		}
		
		double distFromMid = AIRSHIP_DISTANCE / Math.tan(30) - midlineDist;
		d[0] = -(distFromMid / 1.173)/* - (1 / 2 * Config.ROBOT_LENGTH_PROTO))*/;
		d[1] = (150.05 / Math.sin(30)) - (distFromMid / Math.cos(30)) - AIRSHIP_RADIUS;
		return d;
	}
}

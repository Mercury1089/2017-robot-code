package org.usfirst.frc.team1089.robot.util;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * This class encapsulates the the Preferences {@link NetworkTable} to get
 * specific properties that would normally be handled by a config file.
 *
 */
public class Preferences {
	public static final int ROBOT_LENGTH_COMPETITION = 40, ROBOT_LENGTH_PROTO = 33;
	public static final int ROBOT_WIDTH_COMPETITION = 36, ROBOT_WIDTH_PROTO = 32;
	private final NetworkTable NT;
	
	public Preferences() {
		NT = NetworkTable.getTable("Preferences");
	}
	
	/**
	 * <pre>
	 * public double[] getNumber(String key,
	 *                           double def)
	 * </pre>
	 * Gets the specified double in the Prefrences network table,
	 * or returns a default value if the key cannot be found.
	 * 
	 * @param key the key in the network table to get a double from
	 * @param def the default value in case the double cannot be retrieved
	 *            from the specified key
	 */
	public double getNumber(String key, double def) {
		return NT.getNumber(key, def);
	}
	
	/**
	 * <pre>
	 * public double[] getNumber(String key,
	 *                           double... def)
	 * </pre>
	 * Gets the specified double array in the Prefrences network table,
	 * or returns a default value if the key cannot be found.
	 * 
	 * @param key the key in the network table to get a double array from
	 * @param def the values to return in an array in case
	 *            the double array cannot be retrieved from the specified key
	 */
	public double[] getNumberArray(String key, double... def) {
		return NT.getNumberArray(key, def);
	}
	
	/**
	 * <pre>
	 * public String getString(String key,
	 *                         String def)
	 * </pre>
	 * Gets the specified string in the Prefrences network table,
	 * or returns a default value if the key cannot be found.
	 * 
	 * @param key the key in the network table to get a string from
	 * @param def the values to return in case 
	 *            the string cannot be retrieved from the specified key
	 */
	public String getString(String key, String def) {
		return NT.getString(key, def);
	}
}
     
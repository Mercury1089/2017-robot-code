package org.usfirst.frc.team1089.robot.util;

import java.awt.Dimension;
import java.awt.Point;
import java.util.Calendar;

import org.usfirst.frc.team1089.robot.util.VisionProcessor.TargetType;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

/**
 * The {@code VisionListener} class is an {@link ITableListener} that handles getting updates for contour values
 * for use with targeting
 * 
 * @deprecated The variables have been integrated with the {@link VisionProcessor} class and the listeners are already created
 *             in that class as well.
 */
public class VisionListener implements ITableListener {

	private final NetworkTable GEAR_TABLE, HIGH_TABLE;
	
	private final Dimension DIM_GEAR, DIM_HIGH;
	private final Point CENTER_GEAR, CENTER_HIGH;
	private double timeGear, timeHigh;

	public VisionListener(NetworkTable gear, NetworkTable high){
		GEAR_TABLE = gear;
		HIGH_TABLE = high;
		
		DIM_GEAR = new Dimension(-1, -1);
		DIM_HIGH = new Dimension(-1, -1);
		
		CENTER_GEAR = new Point(-1, -1);
		CENTER_HIGH = new Point(-1, -1);
		
		timeGear = timeHigh = System.currentTimeMillis();
		
		gear.addTableListener(this);
		high.addTableListener(this);
	}
	
	/**
	 * <pre>
	 * public void valueChanged(ITable source, String string , Object o, boolean bln)
	 * </pre>
	 * Runs every time a value changes in the network table and logs the change
	 * @param source 
	 * 		  The table from which to get the data and the table to check for changes
	 * @param key
	 * 		  The key associated with the value that changed
	 * @param value
	 * 		  The new value from the table
	 * @param isNew
	 * 		  true if the key did not previously exist in the table, otherwise it is false
	 */
	@Override
	public void valueChanged(ITable source, String key , Object value, boolean isNew){
		long sysTS = System.currentTimeMillis(); // Get the time stamp to check with the time stamp of the vision feed
		synchronized(this) {
			if (source.equals(GEAR_TABLE)) {
				switch (key) {
					case "targetWidth":
						DIM_GEAR.width = (Integer)value;
						break;
					case "targetHeight":
						DIM_GEAR.height = (Integer)value;
						break;
					case "center":
						double[] center = (double[]) value;
						CENTER_GEAR.setLocation(center[0], center[1]);
						break;
					case "deltaTime":
						timeGear = System.currentTimeMillis() - (Double)value;
						break;
				}
			} else if (source.equals(HIGH_TABLE)) {
				switch (key) {
					case "targetWidth":
						DIM_HIGH.width = (Integer)value;
						break;
					case "targetHeight":
						DIM_HIGH.height = (Integer)value;
						break;
					case "center":
						double[] center = (double[]) value;
						CENTER_HIGH.setLocation(center[0], center[1]);
						break;
					case "deltaTime":
						timeHigh = System.currentTimeMillis() - (Double)value;
						break;
				}
			}
		}
	}

	/**
	 * <pre>
	 * public synchronized double getModificationTime()
	 * </pre>
	 * 
	 * Get the last modification time of the vision data.
	 * 
	 * @return The modification time of the specified NetworkTable's data.
	 */
	public synchronized double getModificationTime(TargetType type) {
		if (type.equals(TargetType.GEAR_VISION))
			return timeGear;
		else if (type.equals(TargetType.HIGH_GOAL))
			return timeHigh;
		else
			return 0;
	}	
}

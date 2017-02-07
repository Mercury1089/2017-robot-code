package org.usfirst.frc.team1089.robot.util;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class utilizes Java's built-in {@link Logger} to output debug text to a logfile
 * at {@code "home/lvuser/logs/"} on the roboRIO.
 * 
 */
public class MercLogger {
	private static final DateFormat 
		DATE = new SimpleDateFormat("yyyy-dd-MM"),
		TIME = new SimpleDateFormat("hhmmss.SS"),
		TIME_EXTENDED = new SimpleDateFormat("hh:mm:ss.SS");
	
    private static final Logger LOGGER = Logger.getLogger("");
    private static final DriverStation DRIVER_STATION = DriverStation.getInstance();
    
    private static boolean exists = false;
    
    private static final Formatter FORMATTER = new Formatter() {
        @Override
        public String format(LogRecord record) {
        	
            String output = "";
            
            // Format: [<real_time> / <match_time>] <log_level>: <message>
            output += "[" + TIME_EXTENDED.format(record.getMillis()) + " / " + TIME_EXTENDED.format(DRIVER_STATION.getMatchTime()) + "] ";
            output += record.getLevel() + ": " + record.getMessage();
            output += "\n";

            return output;
        }
    };

    /**
     * <pre>
     * public void init(String path)
     * </pre>
     * Initializes the logger and its {@link FileHandler}.
     * 
     * @param path the directory that you want the log to be stored in
     */
    public static synchronized void init(String path) {
    	if (!exists) { // If we have never initialized the Logger
    		Date d = Calendar.getInstance().getTime();
    		String date = DATE.format(d), time = TIME.format(d);
    		try {
	    		if (!path.endsWith("/"))
	    			path += "/";
	    		
	    		FileHandler fh = new FileHandler(path + "log_" + date + "T" + time + "Z" + ".txt");
				LOGGER.setUseParentHandlers(false);
		    	fh.setFormatter(FORMATTER);
		    	LOGGER.addHandler(fh);
		    	exists = true;
			} catch (Exception e) {
				// He's dead, Jim!
			}
    	}
    }
    
    /**
     * <pre>
     * public static void logMessage(Level lvl, String msg)
     * </pre>
     * Logs a message.
     * 
     * @param lvl the level of the log message
     * @param msg the message to write to the logfile
     */
    public static synchronized void logMessage(Level lvl, String msg) {
    	LOGGER.log(lvl, msg);
    }
    
    /**
     * <pre>
     * public static void logException(Exception e)
     * </pre>
     * Logs an exception to the logfile.
     * This should only be used when an exception is thrown.
     * 
     * @param e the {@Link Exception} that was thrown
     */
    public static synchronized void logException(Exception e) {
    	LOGGER.log(Level.SEVERE, e.getMessage(), e);
    }

}

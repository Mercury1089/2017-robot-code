package org.usfirst.frc.team1089.robot.util;

import java.io.File;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
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
public class Debug {
	private static final DateFormat ISO8601 = new SimpleDateFormat("yyyy-dd-MM_hh-mm-ss.SS"); // Because of Windows, time has to be stored like this.
    private static final Logger LOGGER = Logger.getLogger("");
    private static final DriverStation DRIVER_STATION = DriverStation.getInstance();
    
    private static final Formatter FORMATTER = new Formatter() {
    	private DateFormat 
    		realTime = new SimpleDateFormat("hh:mm:ss.SS"),
    		matchTime = new SimpleDateFormat("mm:ss.SS");

        @Override
        public String format(LogRecord record) {
        	
            String output = "";
            
            // Format: [<real_time> / <match_time>] <log_level>: <message>
            output += "[" + realTime.format(record.getMillis()) + " / " + matchTime.format(DRIVER_STATION.getMatchTime()) + "] ";
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
    	try {
    		if (!path.endsWith("/"))
    			path += "/";
    		
    		FileHandler fh = new FileHandler(path + "log_" + ISO8601.format(Calendar.getInstance().getTime()) + ".txt");
			LOGGER.setUseParentHandlers(false);
	    	fh.setFormatter(FORMATTER);
	    	LOGGER.addHandler(fh);
		} catch (Exception e) {
			// He's dead, Jim!
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

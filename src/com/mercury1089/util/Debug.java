package com.mercury1089.util;

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
    private static FileHandler fh;
    private static final Formatter FORMATTER = new Formatter() {
    	private DateFormat 
    		realTime = new SimpleDateFormat("hh:mm:ss.SS"),
    		matchTime = new SimpleDateFormat("mm:ss.SS");

        @Override
        public String format(LogRecord record) {
        	
            String output = "";
            
            // Format: [<real_time> / <match_time>] <log_level>: <message>
            output += "[" + realTime.format(Calendar.getInstance().getTime()) + " / " + matchTime.format(DRIVER_STATION.getMatchTime()) + "] ";
            output += record.getLevel() + ": " + record.getMessage();

            return output;
        }
    };

    /**
     * <pre>
     * public void init()
     * </pre>
     * Initializes the logger and its {@link FileHandler}.
     */
    public void init() {
    	try {
			fh = new FileHandler("home/lvuser/log/log_" + ISO8601.format(Calendar.getInstance().getTime()) + ".txt");
		} catch (Exception e) {
			// He's dead, Jim!
		} 
    	
    	LOGGER.setUseParentHandlers(false);
    	LOGGER.addHandler(fh);
    	fh.setFormatter(FORMATTER);
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
    public static void logMessage(Level lvl, String msg) {
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
    public static void logException(Exception e) {
    	LOGGER.log(Level.SEVERE, e.getMessage(), e);
    }

}

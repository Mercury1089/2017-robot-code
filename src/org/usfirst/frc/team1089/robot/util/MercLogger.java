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
 * This class utilizes Java's built-in {@link Logger} to output debug text to a logfile.
 * 
 */
public class MercLogger {
	private static final DateFormat 
		DATE = new SimpleDateFormat("yyyy-dd-MM"),
		TIME = new SimpleDateFormat("hhmmss.SSS"),
		TIME_EXTENDED = new SimpleDateFormat("hh:mm:ss.SSS");
	
    private static final Logger LOGGER;
    private static FileHandler handler;
    private static final DriverStation DRIVER_STATION = DriverStation.getInstance();

    static {
    	LOGGER = Logger.getLogger("");
    	LOGGER.setUseParentHandlers(false);
    }
    
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
     * Initializes the logger's {@link FileHandler}.
     * 
     * @param path the directory that you want the log to be stored in
     */
    public static synchronized void init(String path) {
		Date d = Calendar.getInstance().getTime();
		String date = DATE.format(d), time = TIME.format(d);
		try {
    		if (!path.startsWith("/"))
    			path = "/" + path;
    		
    		handler = new FileHandler(path + "log_" + date + "T" + time + "Z" + ".txt");
	    	handler.setFormatter(FORMATTER);
	    	LOGGER.addHandler(handler);
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
    
    /**
     * <pre>
     * public static synchronized void close()
     * </pre>
     * 
     * Closes the Logger's current {@link FileHandler}
     */
    public static synchronized void close() {
    	if (handler != null)
    		handler.close();
    }
}

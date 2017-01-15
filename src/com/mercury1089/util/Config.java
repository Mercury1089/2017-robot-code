package com.mercury1089.util;

import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;

/**
 * This class pulls a {@code ./config.properties} file for deployment configuration,
 * such as what type of environment (testing, competition) we are using the robot in.
 *
 */
public class Config {
	private static final Properties PROP = new Properties();
	private static final String FILE = "config.properties";
	
	/**
	 * <pre>
	 * public synchronized double getNumber(String key)
	 * </pre>
	 * Gets a string based on the specified property.
	 * It is up to the user to parse the string to the value they need it in.
	 * 
	 * @param property the property name to get a value from in the config.properties file
	 * 
	 * @return the value of the property if it exists, {@code null} otherwise.
	 */
	public synchronized String getValue(String property) {
		String val = null;
		
		try {
            InputStream input = new FileInputStream(FILE);
            PROP.load(input);
            
            val = PROP.getProperty(property);
            input.close();
        } catch (Exception e) {
            Debug.logException(e);
        }
    	
		return val;
	}
}

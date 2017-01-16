package com.mercury1089.util;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * This class pulls a {@code config.properties} file for deployment configuration,
 * such as what type of environment (testing, competition) we are using the robot in,
 * how much fuel we have ejected over the season, and other things.
 *
 */
public class Config {
	private static final Properties PROP = new Properties();
	private static final String FILE = "config.properties";
	
	/*
	 * This is a static initialization block that will just initialize this class at runtime
	 * by loading the config.properties file if it exits, or create a new one with default settings.
	 */
	static {
		File f = new File(FILE);
		
		try {
			if (!f.exists()) {
				PROP.setProperty("fuel_ejected", "0");
				
				save();
			} else {
				FileReader reader = new FileReader(f);
				
				PROP.load(reader);
				reader.close();
			}
		} catch (Exception e) {
			Debug.logException(e);
		}
		
	}
	
	/**
	 * <pre>
	 * public synchronized String getValue(String property,
	 *                                     String def)
	 * </pre>
	 * Gets a string based on the specified property.
	 * It is up to the user to parse the string to the value they need it in.
	 * 
	 * @param property the property name to get a value from in the config.properties file
	 * @param def      the default value in case the property does not exist in the config.
	 * 
	 * @return the value of the property if it exists, {@code def} otherwise.
	 */
	public synchronized String getValue(String property, String def) {
		return PROP.getProperty(property, def);
	}
	
	/**
	 * <pre>
	 * public synchronized void setProperty(String key)
	 * </pre>
	 * Sets a specified property's value based on the given string.
	 * It is up to the user to encode the value into a string.
	 * 
	 * @param property the property name to set a value of in the config.properties file
	 */
	public synchronized void setValue(String property, String value) {
        PROP.setProperty(property, value);
	}
	
	/**
	 * <pre>
	 * public void save()
	 * </pre>
	 * Saves the config to the {@code config.properties} file.
	 * 
	 * @throws IOException 
	 */
	public static void save() throws IOException {
		FileWriter writer = new FileWriter(FILE);
		PROP.store(writer, "Robot Configuration");
		writer.close();
	}
}

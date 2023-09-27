// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structure.operator;

import java.util.HashMap;

/** Class to store the preferences of the operators */
public class OperatorProfiles {

    /* Stores preferences */
    private static HashMap<String, Double> m_deadzones = new HashMap<String, Double>();
    private static HashMap<String, Boolean> m_squareInputs = new HashMap<String, Boolean>();

    /* List of currently stored drivers */
    private static final String[] m_knownDrivers = { "Default", "Aaron" };

    /* Selected driver */
    private static String m_driver;

    /**
     * Loads default operator profile settings
     */
    public OperatorProfiles() {
        m_driver = "Default";
        configureProfiles();
    }

    /**
     * Loads operator settings based on the selected driver, sets to system default
     * if driver does not have prefernces
     * 
     * @param driver The selected driver to load settings from
     */
    public OperatorProfiles(String driver) {
        /* Checks if selected driver has logged preferences */
        for (String s : m_knownDrivers) {
            if (s.equals(driver)) {
                m_driver = driver;
                break;
            } else {
                m_driver = "Default";
            }
        }

        configureProfiles();
    }

    /**
     * Gets the driver's preferred joystick deadzones
     * 
     * @return Preffered deadzones
     */
    public static double getDeadzones() {
        return m_deadzones.get(m_driver);
    }

    /**
     * Gets if the driver wants the ArcadeDrive() inputs to be squared or not
     * 
     * @return Should square inputs
     */
    public static boolean getIfSquaredInputs() {
        return m_squareInputs.get(m_driver);
    }

    /**
     * Adds preferred settings to their respective hashmap for each driver
     */
    private static void configureProfiles() {
        /* System defaults */
        m_deadzones.put("Default", 0.0);
        m_squareInputs.put("Default", false);

        /* Aaron */
        m_deadzones.put("Aaron", 0.1);
        m_squareInputs.put("Aaron", true);
    }
}

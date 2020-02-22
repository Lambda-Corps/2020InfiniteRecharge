/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This is a singleton class that will hold all the Shuffleboard entries. This class
 * is where the Shuffleboard layout will be taken care of.  A singleton class is 
 * a class instance where there is only one of them instantiated.  It allows a nice
 * communication method for each of the Subsystems to be able to talk to each other
 * through well-known Network Table entries.
 * 
 * This should enable things like the Drivetrain directing the LED subsystem  
 * bits of information that the LEDs  can communicate to the drive team.
 * 
 * It will take care of giving access to all the various pieces of information
 */
public class ShuffleboardInfo {
    // One tab for each subsystem (or informational piece)
    private final ShuffleboardTab m_led_tab, m_driver_tab, m_auto_tab, m_climber_tab, m_intake_tab, m_shooter_tab, m_vision_tab;

    // Driver Tab Entries
    private final NetworkTableEntry m_LowGearEntry;
    // private constructors is how you can create a singleton, then provide some 
    // sort of accessor method like getInstance(). Then the getinstance checks
    // whether or not we have an instantiated instance, and if not, then creates 
    // it.
    private static ShuffleboardInfo instance = null;

    private ShuffleboardInfo(){
        // Create the various tabs
        m_auto_tab = Shuffleboard.getTab("Autonomous");
        m_led_tab = Shuffleboard.getTab("LEDs");
        m_driver_tab = Shuffleboard.getTab("Driver");
        m_climber_tab = Shuffleboard.getTab("Climber");
        m_intake_tab = Shuffleboard.getTab("Intake");
        m_shooter_tab = Shuffleboard.getTab("Shooter");
        m_vision_tab = Shuffleboard.getTab("Vision");

        // Setup the Driver tab
        m_LowGearEntry = m_driver_tab.add("Low Gear", false).getEntry();

        // Setup the Vision
    

        // Setup the LED tab information


    }

    public static ShuffleboardInfo getInstance(){
        if( instance == null ){
            instance = new ShuffleboardInfo();
        }

        return instance;
    }

    public NetworkTableEntry getDriverLowGearEntry(){
        return m_LowGearEntry;
    }
}

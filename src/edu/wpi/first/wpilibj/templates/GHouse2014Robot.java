/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class GHouse2014Robot extends SimpleRobot {
    /*** Constants ***/
    //==== General Constants ====
    private final int PRESSURE_SWITCH_CH = 1; //Digital IO
    private final int COMPRESSOR_RELAY_CH = 2; //Digital IO
    
    //==== Drivetrain constants ====
    //Speed change solenoid channels
    private final int LEFT_SPEED_CHANGE_CH = 1; //Pneumatics Slot
    private final int RIGHT_SPEED_CHANGE_CH = 2; //Pneumatics Slot
    
    /*** General Components ***/
    private Compressor compressor = new Compressor(PRESSURE_SWITCH_CH, COMPRESSOR_RELAY_CH);
    
    /*** Drivetrain Components ***/
    private int leftControllerChannels[] = {11, 12, 13};
    private int rightControllerChannels[] = {14, 15, 16};
    private MultiCANJaguar leftController, rightController;
    private RobotDrive chassis;
    private Solenoid leftSpeedChangeSolenoid = new Solenoid(LEFT_SPEED_CHANGE_CH);
    private Solenoid rightSpeedChangeSolenoid = new Solenoid(RIGHT_SPEED_CHANGE_CH);

    public GHouse2014Robot() {
        try {
            leftController = new MultiCANJaguar(leftControllerChannels);
            rightController = new MultiCANJaguar(rightControllerChannels);
            
            chassis = new RobotDrive(leftController, rightController);
            
            //Setup routines
            //Start the compressor
            compressor.start();
            
        } catch (CANTimeoutException e) {
            System.out.println("WARNING: CANTimeoutException: " + e.getMessage());
        }
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        while (isEnabled() && isAutonomous()) {
            
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        //TODO Do any initial resetting here
        
        //Default loop
        while (isEnabled() && isOperatorControl()) {
            //1) Sense
            //TODO: Take in any sensor data that we need
            
            //2) Act
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}

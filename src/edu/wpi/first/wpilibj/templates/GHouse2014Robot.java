/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
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
    private final int COMPRESSOR_RELAY_CH = 1; //Relay Slot
    
    //==== Drivetrain Constants ====
    //Speed change solenoid channels
    private final int SPEED_CHANGE_CH = 1; //Pneumatics Slot
    
    //==== Feed Mechanism Constants ====
    //Feed arm solenoid channels
    private final int FEED_LEFT_ARM_CH = 3; //Pneumatics Slot
    private final int FEED_RIGHT_ARM_CH = 4; //Pneumatics Slot
    private final int FEED_MOTOR_CH = 1; //PWM Port
    
    //==== Scissor Lift Mechanism Constants ====
    //Scissor Lift
    private final int SCISSOR_LEFT_PISTON_CH = 5; //Pneumatics Slot
    private final int SCISSOR_RIGHT_PISTON_CH = 6; //Pneumatics Slot
    
    //==== Shooter Mechanism Constants ====
    //Shooter Motor
    private final int SHOOTER_MOTOR_CH = 2; //PWM Port
    
    
    
    /*** General Components ***/
    private Compressor compressor = new Compressor(PRESSURE_SWITCH_CH, COMPRESSOR_RELAY_CH);
    
    /*** Drivetrain Components ***/
    private final int leftControllerChannels[] = {11, 19, 15};
    private final int rightControllerChannels[] = {16, 13, 12};
    private MultiCANJaguar leftController, rightController;
    private RobotDrive chassis;
    private Solenoid speedChangeSolenoid = new Solenoid(SPEED_CHANGE_CH);
    
    /*** Feed Mechanism ***/
    private Solenoid leftFeedArmSolenoid = new Solenoid(FEED_LEFT_ARM_CH);
    private Solenoid rightFeedArmSolenoid = new Solenoid(FEED_RIGHT_ARM_CH);
    private Victor feedMotor = new Victor(FEED_MOTOR_CH);
    
    /*** Scissor Lift Mechanism ***/
    private Solenoid leftScissorPiston = new Solenoid(SCISSOR_LEFT_PISTON_CH);
    private Solenoid rightScissorPiston = new Solenoid(SCISSOR_RIGHT_PISTON_CH);
    
    /*** Shooter Mechanism ***/
    private Victor shooterMotor = new Victor(SHOOTER_MOTOR_CH);

    /*** Human Interface Components ***/
    Joystick driveStick = new Joystick(1);
    
    
    public GHouse2014Robot() {
        try {
            leftController = new MultiCANJaguar(leftControllerChannels);
            rightController = new MultiCANJaguar(rightControllerChannels);
            
            chassis = new RobotDrive(leftController, rightController);
            
            //We need to invert the motors
            chassis.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
            chassis.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
            
            //Setup routines
            //Start the compressor
            compressor.start();
            
        } catch (CANTimeoutException e) {
            System.out.println("WARNING: CANTimeoutException: " + e.getMessage());
            e.printStackTrace();
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
            //Always drive!
            chassis.arcadeDrive(driveStick);
            
            //===== SPEED CHANGE DECISIONS =====
            //Take note of whether or not the trigger is pressed
            //If so, activate the speed change. The trigger is button 1
            if (driveStick.getButton(Joystick.ButtonType.kTrigger) == true) { //pressed
                speedChangeSolenoid.set(true);
            }
            else {
                speedChangeSolenoid.set(false);
            }
            
            //===== 
            
            //2) Act
            
            
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}

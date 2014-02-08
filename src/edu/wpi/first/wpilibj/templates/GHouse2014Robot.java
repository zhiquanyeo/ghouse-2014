/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Date;

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
    
    private final boolean IS_USING_GAMEPAD = true;
    
    //==== Drivetrain Constants ====
    //Speed change solenoid channels
    private final int SPEED_CHANGE_CH = 1; //Pneumatics Slot
    
    //==== Feed Mechanism Constants ====
    //Feed arm solenoid channels
    private final int FEED_DOWN_CH = 2; //Pneumatics Slot
    private final int FEED_UP_CH = 3; //Pneumatics Slot
    private final int FEED_MOTOR_CH = 1; //PWM Port
    private final int FEED_ARM_INSIDE_SENSOR_CH = 3; //Digital IO
    private final int FEED_ARM_OUTSIDE_SENSOR_CH = 2; //Digital IO
    
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
    private DoubleSolenoid feedSolenoid = new DoubleSolenoid(FEED_DOWN_CH, FEED_UP_CH);
    private Victor feedMotor = new Victor(FEED_MOTOR_CH);
    private DigitalInput feedInsideSensor = new DigitalInput(FEED_ARM_INSIDE_SENSOR_CH);
    private DigitalInput feedOutsideSensor = new DigitalInput(FEED_ARM_OUTSIDE_SENSOR_CH);
    
    /*** Scissor Lift Mechanism ***/
    private Solenoid leftScissorPiston = new Solenoid(SCISSOR_LEFT_PISTON_CH);
    private Solenoid rightScissorPiston = new Solenoid(SCISSOR_RIGHT_PISTON_CH);
    
    /*** Shooter Mechanism ***/
    private Victor shooterMotor = new Victor(SHOOTER_MOTOR_CH);

    /*** Human Interface Components ***/
    Joystick driveStick = new Joystick(1);
    
    /*** Robot State ***/
    private boolean feedArmUp = true;
    private boolean feedArmInTransit = false;
    
    private final int SHUTOFF_TIME = 2000; //ms
    private long stopRequestedTime;
    private boolean shouldShutOffFeedSolenoid = false;
    
    public GHouse2014Robot() {
        SmartDashboard.putBoolean("Using Gamepad", IS_USING_GAMEPAD);
        
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
        chassis.setSafetyEnabled(false);
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
            
            //Figure out if we should shut off the feed solenoid
            if (shouldShutOffFeedSolenoid) {
                if (System.currentTimeMillis() - stopRequestedTime > SHUTOFF_TIME) {
                    shouldShutOffFeedSolenoid = false;
                    feedSolenoid.set(DoubleSolenoid.Value.kOff);
                    System.out.println("Feed solenoid set to OFF");
                }
            }
            
            //===== DRIVING =====
            if (IS_USING_GAMEPAD) {
                chassis.arcadeDrive(driveStick.getY(), -driveStick.getZ(), true);
            }
            else {
                chassis.arcadeDrive(driveStick.getY(), -driveStick.getX(), true);
            }
            
            //===== SPEED CHANGE DECISIONS =====
            //Take note of whether or not the trigger is pressed
            //If so, activate the speed change. The trigger is button 1
            if (IS_USING_GAMEPAD) {
                if (driveStick.getRawButton(8) == true) { //pressed
                    speedChangeSolenoid.set(true);
                }
                else {
                    speedChangeSolenoid.set(false);
                }
            }
            else {
                if (driveStick.getButton(Joystick.ButtonType.kTrigger) == true) { //pressed
                    speedChangeSolenoid.set(true);
                }
                else {
                    speedChangeSolenoid.set(false);
                }
            }
            
            //==== Feed Mechanism ====
            //TODO This should go into update state
            if (feedInsideSensor.get() == true && feedOutsideSensor.get() == false) {
                //Inside sensor on, outside sensor off => feed arm UP
                feedArmUp = false;
                feedArmInTransit = false;
                feedMotor.set(0.5);
                SmartDashboard.putBoolean("Feed Arm Up", feedArmUp);
                SmartDashboard.putBoolean("Feed Arm In Transit", feedArmInTransit);
            } 
            else if (feedInsideSensor.get() == false && feedOutsideSensor.get() == true) {
                //Inside sensor off, outside sensor on => feed arm DOWN
                feedArmUp = true;
                feedArmInTransit = false;
                feedMotor.stopMotor();
                SmartDashboard.putBoolean("Feed Arm Up", feedArmUp);
                SmartDashboard.putBoolean("Feed Arm In Transit", feedArmInTransit);
            }
            else {
                feedArmInTransit = true;
                feedMotor.stopMotor();
                SmartDashboard.putBoolean("Feed Arm In Transit", feedArmInTransit);
            }
            feedMotor.Feed();
            
            if (IS_USING_GAMEPAD) {
                if (driveStick.getRawButton(2)) {
                    //button was pressed. are we up or down?
                    //don't do anything while we are in transit
                    if (!feedArmInTransit) {
                        if (feedArmUp == false) {
                            feedSolenoid.set(DoubleSolenoid.Value.kForward);
                            shouldShutOffFeedSolenoid = true;
                            stopRequestedTime = System.currentTimeMillis();
                        }
                        else {
                            feedSolenoid.set(DoubleSolenoid.Value.kReverse);
                            shouldShutOffFeedSolenoid = true;
                            stopRequestedTime = System.currentTimeMillis();
                        }
                    }
                }
            }
            else {
                if (driveStick.getButton(Joystick.ButtonType.kTop)) {
                    //button was pressed. are we up or down?
                    //don't do anything while we are in transit
                    if (!feedArmInTransit) {
                        if (feedArmUp == false) {
                            feedSolenoid.set(DoubleSolenoid.Value.kForward);
                        }
                        else {
                            feedSolenoid.set(DoubleSolenoid.Value.kReverse);
                        }
                    }
                }
            }
            
            //===== SHOOTER =====
            //TODO Test only. need to integrate with choo-choo limit switch
            if (driveStick.getRawButton(7)) {
                shooterMotor.set(0.5);
            }
            else {
                shooterMotor.stopMotor();
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
    
    //Support Functions
    private void updateRobotState() {
        
    }
}

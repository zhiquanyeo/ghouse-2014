/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.ghouse.positional.DeadReckoningEngine;
import edu.ghouse.robot2014.FeedMechanism;
import edu.ghouse.robot2014.ScissorMechanism;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private final int ENCODER_RIGHT_A_CH = 4;
    private final int ENCODER_RIGHT_B_CH = 5;
    private final int ENCODER_LEFT_A_CH = 6;
    private final int ENCODER_LEFT_B_CH = 7;
    private final int ENCODER_PULSES_PER_ROTATION = 128;
    //3 revolutions of encoder per 1 rotation of transmission
    //TODO calculate distance per pulse
    //Total pulse = 3x ENCODER_PULSE_PER_ROTATION
    private final double WHEEL_DIAMETER = 4.0 / 12; //4 inches, converted to feet
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final int ENCODER_TO_OUTPUT_RATIO = 3; 
    private final int ENCODER_TOTAL_PULSES_PER_ROTATION = ENCODER_PULSES_PER_ROTATION * ENCODER_TO_OUTPUT_RATIO;
    
    
    //==== Feed Mechanism Constants ====
    //Feed arm solenoid channels
    private final int FEED_DOWN_CH = 2; //Pneumatics Slot
    private final int FEED_UP_CH = 3; //Pneumatics Slot
    private final int FEED_MOTOR_CH = 1; //PWM Port
    private final int FEED_ARM_INSIDE_SENSOR_CH = 3; //Digital IO
    private final int FEED_ARM_OUTSIDE_SENSOR_CH = 2; //Digital IO
    
    //==== Scissor Lift Mechanism Constants ====
    //Scissor Lift
    private final int SCISSOR_UP_CH = 5; //Pneumatics Slot
    private final int SCISSOR_DOWN_CH = 6; //Pneumatics Slot
    private final int SCISSOR_INSIDE_SENSOR_CH = 8; //Digital IO
    private final int SCISSOR_OUTSIDE_SENSOR_CH = 9; //Digital IO
    
    //==== Shooter Mechanism Constants ====
    //Shooter Motor
    private final int SHOOTER_MOTOR_CH = 2; //PWM Port
    
    
    //==== Human Interface Button Constants ====
    private final int FEED_ARM_TOGGLE_BUTTON = 2;
    private final int SPEED_CHANGE_BUTTON = 8;
    
    /*** General Components ***/
    private Compressor compressor = new Compressor(PRESSURE_SWITCH_CH, COMPRESSOR_RELAY_CH);
    
    /*** Drivetrain Components ***/
    private final int leftControllerChannels[] = {11, 19, 15};
    private final int rightControllerChannels[] = {16, 13, 12};
    private MultiCANJaguar leftController, rightController;
    private RobotDrive chassis;
    private Solenoid speedChangeSolenoid = new Solenoid(SPEED_CHANGE_CH);
    private Encoder rightEncoder = new Encoder(ENCODER_RIGHT_A_CH, ENCODER_RIGHT_B_CH, true);
    private Encoder leftEncoder = new Encoder(ENCODER_LEFT_A_CH, ENCODER_LEFT_B_CH);
    
    /*** Feed Mechanism ***/
    private DoubleSolenoid feedSolenoid = new DoubleSolenoid(FEED_DOWN_CH, FEED_UP_CH);
    private Victor feedMotor = new Victor(FEED_MOTOR_CH);
    private DigitalInput feedInsideSensor = new DigitalInput(FEED_ARM_INSIDE_SENSOR_CH);
    private DigitalInput feedOutsideSensor = new DigitalInput(FEED_ARM_OUTSIDE_SENSOR_CH);
    //object to manage the feed system
    private FeedMechanism feedMechanism = new FeedMechanism(feedSolenoid, feedMotor, feedInsideSensor, feedOutsideSensor);
    
    /*** Scissor Lift Mechanism ***/
    private DoubleSolenoid scissorSolenoid = new DoubleSolenoid(SCISSOR_UP_CH, SCISSOR_DOWN_CH);
    private DigitalInput scissorInsideSensor = new DigitalInput(SCISSOR_INSIDE_SENSOR_CH);
    private DigitalInput scissorOutsideSensor = new DigitalInput(SCISSOR_OUTSIDE_SENSOR_CH);
    //object to manage the scissor system
    private ScissorMechanism scissorMechanism = new ScissorMechanism(scissorSolenoid, scissorInsideSensor, scissorOutsideSensor);
    
    /*** Shooter Mechanism ***/
    private Victor shooterMotor = new Victor(SHOOTER_MOTOR_CH);
    
    //Support objects
    private final double AXLE_LENGTH = 2.359375; //feet
    private DeadReckoningEngine deadReckoningEngine = new DeadReckoningEngine(AXLE_LENGTH, leftEncoder, rightEncoder);

    /*** Human Interface Components ***/
    Joystick driveStick = new Joystick(1);
    
    
    public void robotInit() {
        System.out.println("Booting up");
        SmartDashboard.putBoolean("Using Gamepad", IS_USING_GAMEPAD);
        
        //Set up the distance per pulse for the encoder
        leftEncoder.setDistancePerPulse(WHEEL_DIAMETER / ENCODER_TOTAL_PULSES_PER_ROTATION);
        rightEncoder.setDistancePerPulse(WHEEL_DIAMETER / ENCODER_TOTAL_PULSES_PER_ROTATION);
        
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
        leftEncoder.reset();
        rightEncoder.reset();
        leftEncoder.start();
        rightEncoder.start();
        deadReckoningEngine.reset();
        deadReckoningEngine.start();
        while (isEnabled() && isAutonomous()) {
            //Figure out where we are
            deadReckoningEngine.updateState();
        }
        deadReckoningEngine.stop();
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        //TODO Do any initial resetting here
        leftEncoder.reset();
        rightEncoder.reset();
        leftEncoder.start();
        rightEncoder.start();
        
        //Default loop
        while (isEnabled() && isOperatorControl()) {
            //1) Sense
            
            //update the feedMechanism
            feedMechanism.updateState();
            
            
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
                if (driveStick.getRawButton(SPEED_CHANGE_BUTTON) == true) { //pressed
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
            
            if (driveStick.getRawButton(FEED_ARM_TOGGLE_BUTTON)) {
                //button was pressed. are we up or down?
                //don't do anything while we are in transit
                if (!feedMechanism.isArmInTransit()) {
                    //If the arm is up, we lower it
                    if (feedMechanism.isArmUp()) {
                        feedMechanism.lowerArm();
                    }
                    else {
                        feedMechanism.raiseArm();
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
            
            //=====  Scissor =====
            //ONLY raise the scissor if the feed is DOWN
            
            
            //===== 
            
            //2) Act
            
            
            //Finally update driver station
            updateRobotState();
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
        
    }
    
    //Stuff to do when the robot is disabled
    public void disabled() {
        
    }
    
    //Support Functions
    private void updateRobotState() {
        //This will update all the robot state that we need on the driver station
        //==== Feed Mechanism Status ====
        SmartDashboard.putBoolean("Feed Arm Up", feedMechanism.isArmUp());
        SmartDashboard.putBoolean("Feed Arm In Transit", feedMechanism.isArmInTransit());
        SmartDashboard.putBoolean("Can Raise Scissor", !feedMechanism.isArmUp());
        
        //==== Encoder Status ====
        SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.getDistance());
    }
}

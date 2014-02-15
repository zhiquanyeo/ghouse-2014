/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.ghouse.positional.DeadReckoningEngine;
import edu.ghouse.positional.DeadReckoningEngine.Position;
import edu.ghouse.robot2014.FeedMechanism;
import edu.ghouse.robot2014.ScissorMechanism;
import edu.ghouse.robot2014.ShooterMechanism;
import edu.ghouse.vision.RobotCamera;
import edu.ghouse.vision.RobotCamera.TargetReport;
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
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

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
    
    private final long TELEOP_SAFE_TIME = 118000; //118 seconds, switch to 119 if Cliff is not happy
        
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
    private final int SCISSOR_UP_CH = 4; //Pneumatics Slot
    private final int SCISSOR_DOWN_CH = 5; //Pneumatics Slot
    private final int SCISSOR_INSIDE_SENSOR_CH = 8; //Digital IO
    private final int SCISSOR_OUTSIDE_SENSOR_CH = 9; //Digital IO
    
    //==== Shooter Mechanism Constants ====
    //Shooter Motor
    private final int SHOOTER_MOTOR_CH = 2; //PWM Port
    private final int SHOOTER_LIMIT_SWITCH_CH = 11; //Digital IO
    
    
    //==== Human Interface Button Constants ====
    private final int FEED_ARM_TOGGLE_BUTTON = 2;
    private final int SPEED_CHANGE_BUTTON = 8;
    private final int SCISSOR_TOGGLE_BUTTON = 4;
    
    private final int DRIVER_FIRE_BUTTON = 7;
    private final int SHOOTER_FIRE_BUTTON = 8;
    private final int DRIVER_SHOOT_OVERRIDE_BUTTON = 5;
    private final int SHOOTER_SHOOT_OVERRIDE_BUTTON = 6;
    
    /*** General Components ***/
    private Compressor compressor = new Compressor(PRESSURE_SWITCH_CH, COMPRESSOR_RELAY_CH);
    
    /*** Drivetrain Components ***/
    private final int leftControllerChannels[] = {11, 19};
    private final int rightControllerChannels[] = {16, 13};
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
    private DigitalInput shooterLimitSwitch = new DigitalInput(SHOOTER_LIMIT_SWITCH_CH);
    private ShooterMechanism shooterMechanism = new ShooterMechanism(shooterMotor, shooterLimitSwitch);
    
    //Support objects
    private final double AXLE_LENGTH = 2.359375; //feet
    private DeadReckoningEngine deadReckoningEngine = new DeadReckoningEngine(AXLE_LENGTH, leftEncoder, rightEncoder);
    private RobotCamera robotCamera;
    
    /*** Human Interface Components ***/
    Joystick driveStick = new Joystick(1);
    Joystick shooterStick = new Joystick(2);
    
    //test code only
    private DigitalInput num12 = new DigitalInput(12);
    private DigitalInput num13 = new DigitalInput(13);
    
    private boolean safeToOperate = true;
    private boolean useSafety = true;
    
    //Grab data from the SmartDashboard (sent via roborealm)
    private NetworkTable server = NetworkTable.getTable("SmartDashboard");
    
    
    public void robotInit() {
        System.out.println("Booting up");
        System.out.println("Initializing camera...");
        robotCamera = new RobotCamera();
        
        
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
        
        double distanceToTarget;
        boolean targetIsHot = false;
        
        Position currPos = deadReckoningEngine.getCurrentPosition();
        
        SmartDashboard.putString("Estimated Position", currPos.toString());
        
        while (isEnabled() && isAutonomous()) {
            //Figure out where we are
            deadReckoningEngine.updateState();
            
            //We want to make sure we are going straight...
            currPos = deadReckoningEngine.getCurrentPosition();
            SmartDashboard.putString("Estimated Position", currPos.toString());
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
        
        //register our start time for safety
        long teleopStartTime = System.currentTimeMillis();
        System.out.println("Teleop Start: " + teleopStartTime);
        safeToOperate = true;
        
        long lastCameraTime = System.currentTimeMillis();
        long CAMERA_TIMEOUT = 1000; //5 times per second
        
        TargetReport report = robotCamera.getTargetReport();
        
        double distanceToTarget;
        boolean targetIsHot = false;
        
        Position currPos = deadReckoningEngine.getCurrentPosition();
        
        SmartDashboard.putString("Estimated Position", currPos.toString());
        
        
        //Default loop
        while (isEnabled() && isOperatorControl()) {
            //1) Sense
            //do a safety check
//            if (safeToOperate && System.currentTimeMillis() - teleopStartTime > TELEOP_SAFE_TIME) {
//                System.out.println("Switching to UNSAFE");
//                safeToOperate = false;
//                SmartDashboard.putBoolean("Safe To Operate", safeToOperate);
//                //this was the initial time that we were set into unsafe mode
//                //trigger a fire if we are armed
//                if (shooterMechanism.isArmed()) {
//                    shooterMechanism.safeFire();
//                }
//            }
            
            //Grab data off the network tables
            try {
                distanceToTarget = server.getNumber("DIST_TO_TARGET", -1.000);
            }
            catch (TableKeyNotDefinedException e) {
                distanceToTarget = -1.00;
            }
            
            try {
                targetIsHot = server.getBoolean("IS_HOT_TARGET", false);
            }
            catch (TableKeyNotDefinedException e) {
                targetIsHot = false;
            }
            
            //DEPRECATE 
            //Camera stuff
//            if (System.currentTimeMillis() - lastCameraTime > CAMERA_TIMEOUT) {
//                report = robotCamera.getTargetReport();
//                lastCameraTime = System.currentTimeMillis();
//            }
//            
//            if (report != null) {
//                SmartDashboard.putBoolean("Hot Target", report.Hot);
//                SmartDashboard.putNumber("Target Distance", report.distance);
//            }
            
            //update the feedMechanism
            feedMechanism.updateState();
            scissorMechanism.updateState();
            shooterMechanism.updateState();
            if (scissorMechanism.isScissorUp()) {
                feedMechanism.setMotorEnabled(false);
            }
            else {
                feedMechanism.setMotorEnabled(true);
            }
            
            chassis.arcadeDrive(driveStick.getY(), -driveStick.getZ(), true);
            
            //positional
            deadReckoningEngine.updateState();
            currPos = deadReckoningEngine.getCurrentPosition();
            SmartDashboard.putString("Estimated Position", currPos.toString());
            
            //===== SPEED CHANGE DECISIONS =====
            //Take note of whether or not the trigger is pressed
            //If so, activate the speed change. The trigger is button 1
            if (driveStick.getRawButton(SPEED_CHANGE_BUTTON) == true) { //pressed
                speedChangeSolenoid.set(true);
            }
            else {
                speedChangeSolenoid.set(false);
            }
            
            
            if (shooterStick.getRawButton(FEED_ARM_TOGGLE_BUTTON)) {
                //button was pressed. are we up or down?
                //don't do anything while we are in transit
                if (!feedMechanism.isArmInTransit()) {
                    //If the arm is up, we lower it
                    if (feedMechanism.isArmUp()) {
                        feedMechanism.lowerArm();
                    }
                    else {
                        //DO NOT RAISE ARM WHEN SCISSOR IS UP!
                        //VERY IMPORTANT! KITTENS WILL DIE
                        if (!scissorMechanism.isScissorUp() && !scissorMechanism.isScissorInTransit()) {
                            feedMechanism.raiseArm();
                        }
                    }
                }
            }
            
            //Scissor mechanism
            if (shooterStick.getRawButton(SCISSOR_TOGGLE_BUTTON)) {
                if (!scissorMechanism.isScissorInTransit()) {
                    if (scissorMechanism.isScissorUp()) {
                        //we can just lower
                        scissorMechanism.lowerScissor();
                        feedMechanism.setMotorEnabled(true);
                    }
                    else {
                        //IMPORTANT!!! MUST CHECK FOR ARM POSITION
                        //DO NOT UNDER ANY CIRCUMSTANCES RAISE THE SCISSOR WHEN 
                        //THE ARM IS UP. BAD THINGS WILL HAPPEN
                        if (!feedMechanism.isArmUp() && !feedMechanism.isArmInTransit()) {
                            scissorMechanism.raiseScissor();
                            //disable the motor on the feed system
                            feedMechanism.setMotorEnabled(false);
                        }
                    }
                }
            }
            
            
            
            //===== SHOOTER =====
            //Both the shooter stick and driver stick have the ability to fire
            //left trigger on the drive stick
            //right trigger on the shooter stick
            if (safeToOperate && (driveStick.getRawButton(DRIVER_FIRE_BUTTON) || shooterStick.getRawButton(SHOOTER_FIRE_BUTTON)) 
                    && (!driveStick.getRawButton(DRIVER_SHOOT_OVERRIDE_BUTTON) && !shooterStick.getRawButton(SHOOTER_SHOOT_OVERRIDE_BUTTON))) {
                if (!shooterMechanism.isArmed())
                    shooterMechanism.arm();
                else {
                    shooterMechanism.fire();
                }
            }
            
            //override
            if (safeToOperate && (driveStick.getRawButton(DRIVER_SHOOT_OVERRIDE_BUTTON) || shooterStick.getRawButton(SHOOTER_SHOOT_OVERRIDE_BUTTON))) {
                shooterMechanism.setOverride(true);
                shooterMotor.set(0.3);
            }
            else {
                if (shooterMechanism.getOverride()) {
                    shooterMechanism.setOverride(false);
                    shooterMotor.stopMotor();
                }
                
            }
            
            
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
        
        //==== Scissor Mechanism Status ====
        SmartDashboard.putBoolean("Scissor Up", scissorMechanism.isScissorUp());
        SmartDashboard.putBoolean("Scissor In Transit", scissorMechanism.isScissorInTransit());
        SmartDashboard.putBoolean("Can Raise Arm", !scissorMechanism.isScissorUp());
        
        //==== Shooter Mechanism Status ====
        SmartDashboard.putBoolean("Shooter Armed", shooterMechanism.isArmed());
        
        //==== Encoder Status ====
        SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.getDistance());
        
        SmartDashboard.putBoolean("Safe To Operate", safeToOperate);
        
    }
}

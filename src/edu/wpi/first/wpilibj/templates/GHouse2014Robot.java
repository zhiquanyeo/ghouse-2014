/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import com.sun.squawk.util.MathUtils;
import com.sun.squawk.vm.CS;
import edu.ghouse.drivesystem.MultiCANJaguar;
import edu.ghouse.positional.DeadReckoningEngine;
import edu.ghouse.positional.DeadReckoningEngine.Position;
import edu.ghouse.robot2014.FeedMechanism;
import edu.ghouse.robot2014.ScissorMechanism;
import edu.ghouse.robot2014.ShooterMechanism;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
    
    /*** Human Interface Components ***/
    Joystick driveStick = new Joystick(1);
    Joystick shooterStick = new Joystick(2);
    
    //test code only
    private DigitalInput num12 = new DigitalInput(12);
    private DigitalInput num13 = new DigitalInput(13);
    
    private boolean safeToOperate = true;
    private boolean useSafety = true; //used to verify that safety mode is on
    
    //Grab data from the SmartDashboard (sent via roborealm)
    private NetworkTable server = NetworkTable.getTable("SmartDashboard");
    private double distanceToTarget = -1;
    private boolean targetIsHot = false;
    
    //Robot preferences
    Preferences prefs = Preferences.getInstance();
    
    //Driving control mode
    private final int CTRL_MODE_DUAL_STICK_ARCADE = 1;
    private final int CTRL_MODE_SINGLE_STICK_ARCADE = 2;
    private final int CTRL_MODE_TANK = 3;
    private final int CTRL_MODE_DUAL_STICK_ARCADE_LEFTY = 4;
    private int controlMode = CTRL_MODE_DUAL_STICK_ARCADE; //the default
    
    private boolean useExponentialScaling = false;
    private final double DEFAULT_EXPO_VALUE = 4.0;
    private double expoValue = DEFAULT_EXPO_VALUE;
    
    //Driver station messages
    DriverStationLCD driverStation = DriverStationLCD.getInstance();
    
    //Exponential scaling function
    public double expo(double input, double expoValue) {
        double multiplier = 1.0;
        if (input < 0) {
            input = input * -1.0;
            multiplier = -1.0;
        }
        double yVal = (MathUtils.exp(expoValue * input) - 1) / (MathUtils.exp(expoValue) - 1);
        return multiplier * yVal;
    }
    
    //Helper function to print current control mode
    private String printCurrentControlMode() {
        switch (controlMode) {
            case CTRL_MODE_DUAL_STICK_ARCADE:
                return "Dual Stick Arcade Drive";
            case CTRL_MODE_SINGLE_STICK_ARCADE:
                return "Single Stick Arcade Drive";
            case CTRL_MODE_TANK:
                return "Tank Drive";
            case CTRL_MODE_DUAL_STICK_ARCADE_LEFTY:
                return "Dual Stick Arcde Drive (Lefty)";
        }
        return "Dual Stick Arcade Drive";
    }
    
    //Helper functions to get RoboRealm vision data
    private double getVisualDistanceToTarget() {
        double dist = -1.0;
        try {
            dist = server.getNumber("DIST_TO_TARGET", -1.000);
        }
        catch (TableKeyNotDefinedException e) {
            dist = -1.00;
        }
        return dist;
    }
    
    private boolean getIsTargetHot() {
        boolean isHot = false;
        try {
            isHot = server.getBoolean("IS_HOT_TARGET", false);

        }
        catch (TableKeyNotDefinedException e) {
            System.out.println("WARNING TableKeyNotDefined");
            e.printStackTrace();
            isHot = false;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return isHot;
    }
    
    //Helper function to take into account the control mode and expo steering
    private void doDrive() {
        double xVal, yVal, zVal, throttleVal;
        xVal = driveStick.getX();
        yVal = driveStick.getY();
        zVal = driveStick.getZ();
        throttleVal = driveStick.getThrottle();
        boolean useSquare = true;
        
        if (useExponentialScaling) {
            xVal = expo(xVal, expoValue);
            yVal = expo(yVal, expoValue);
            zVal = expo(zVal, expoValue);
            throttleVal = expo(throttleVal, expoValue);
            useSquare = false;
        }
        
        if (controlMode == CTRL_MODE_DUAL_STICK_ARCADE) {
            chassis.arcadeDrive(yVal, -zVal, useSquare);
        }
        else if (controlMode == CTRL_MODE_SINGLE_STICK_ARCADE) {
            chassis.arcadeDrive(throttleVal, -zVal, useSquare);
        }
        else if (controlMode == CTRL_MODE_DUAL_STICK_ARCADE_LEFTY) {
            chassis.arcadeDrive(throttleVal, -xVal, useSquare);
        }
        else {
            //We need to swap the sticks for tankDrive
            chassis.tankDrive(throttleVal, yVal, useSquare);
        }
        
    }
    
    public void robotInit() {
        System.out.println("Booting up");
        System.out.println("Initializing camera...");
        
        //Preferences Default
        boolean prefsWritten = false;
        if (!prefs.containsKey("control_mode")) {
            prefs.putString("control_mode", "dualarcade");
            prefsWritten = true;
        }
        if (!prefs.containsKey("use_exponential")) {
            prefs.putBoolean("use_exponential", false);
            prefsWritten = true;
        }
        if (!prefs.containsKey("exponential_value")) {
            prefs.putDouble("exponential_value", DEFAULT_EXPO_VALUE);
            prefsWritten = true;
        }
        if (!prefs.containsKey("use_safety")) {
            prefs.putBoolean("use_safety", true);
            prefsWritten = true;
        }
        
        if (prefsWritten) {
            prefs.save();
        }
        
        //Figure out our control mode
        System.out.println("Reading control_mode from Preferences");
        String controlModeStr = prefs.getString("control_mode", "dualarcade");
        if ("dualarcade".equals(controlModeStr)) {
            controlMode = CTRL_MODE_DUAL_STICK_ARCADE;
        }
        else if ("singlearcade".equals(controlModeStr)) {
            controlMode = CTRL_MODE_SINGLE_STICK_ARCADE;
        }
        else if ("tank".equals(controlModeStr)) {
            controlMode = CTRL_MODE_TANK;
        }
        else if ("dualarcadelefty".equals(controlModeStr)) {
            controlMode = CTRL_MODE_DUAL_STICK_ARCADE_LEFTY;
        }
        else {
            controlMode = CTRL_MODE_DUAL_STICK_ARCADE;
        }
        SmartDashboard.putString("Current Control Mode", printCurrentControlMode());
        
        //Figure out if we need to use exponential scaling
        System.out.println("Reading use_exponential from Preferences");
        useExponentialScaling = prefs.getBoolean("use_exponential", false);
        SmartDashboard.putBoolean("Use Exponential Inputs", useExponentialScaling);
        
        System.out.println("Reading exponential_value from Preferences");
        expoValue = prefs.getDouble("exponential_value", DEFAULT_EXPO_VALUE);
        if (useExponentialScaling) {
            SmartDashboard.putNumber("Exponential Value", expoValue);
        }
        
        System.out.println("Reading use_safety from Preferences");
        useSafety = prefs.getBoolean("use_safety", true);
        SmartDashboard.putBoolean("Teleop Safety Mode", useSafety);
        
        //Set up the distance per pulse for the encoder
        leftEncoder.setDistancePerPulse(WHEEL_DIAMETER / ENCODER_TOTAL_PULSES_PER_ROTATION * 2);
        rightEncoder.setDistancePerPulse(WHEEL_DIAMETER / ENCODER_TOTAL_PULSES_PER_ROTATION * 2);
        
        try {
            leftController = new MultiCANJaguar(leftControllerChannels);
            rightController = new MultiCANJaguar(rightControllerChannels);
            
            chassis = new RobotDrive(leftController, rightController);
            //chassis = new DebugRobotDrive(leftController, rightController);
            
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
        
        double Kp = 0.03; //K factor
        final double AUTO_DRIVE_SPEED = 0.6;
        
        //Autonomous State Machine
        final int AUTO_STATE_DRIVE_FWD = 0;
        final int AUTO_STATE_ALIGNMENT = 1;
        final int AUTO_STATE_LOWER_GATE = 2;
        final int AUTO_STATE_RAISE_SCISSOR = 3;
        final int AUTO_STATE_READY_TO_FIRE = 4;
        final int AUTO_STATE_FIRE = 5;
        final int AUTO_STATE_STANDBY = 6;
        final int AUTO_STATE_STANDBY2 = 7;
        
        int currentAutoState = AUTO_STATE_DRIVE_FWD;
        
        
        final int PRE_FIRE_TIME = 500;
        final int POST_FIRE_TIME = 500;
        double preFireStart = -1;
        double postFireStart = -1;
        
        //Test code
        long autoStartTime = System.currentTimeMillis();
        shooterMechanism.arm();
        
        boolean isOffsetLeft = false;
        
        targetIsHot = getIsTargetHot();
        if (targetIsHot) {
            System.out.println("Running HOT target program");
            while (isEnabled() && isAutonomous()) {
                deadReckoningEngine.updateState();
                feedMechanism.updateState();
                scissorMechanism.updateState();
                shooterMechanism.updateState();
                
                //update our current position on the field
                currPos = deadReckoningEngine.getCurrentPosition();
                SmartDashboard.putString("Estimated Position", currPos.toString());
                
                switch (currentAutoState) {
                    case AUTO_STATE_DRIVE_FWD: {
                        //Do straight line driving... or something resembling it
                        double angle = currPos.theta;
                        chassis.drive(-AUTO_DRIVE_SPEED, -angle * Kp);

                        //Transition Check: Drive 14.5 feet
                        if (currPos.x > 7) {
                            feedMechanism.lowerArm();
                            //Stop the motors
                            chassis.drive(0, 0);
                            //switch state to alignment mode
                            currentAutoState = AUTO_STATE_READY_TO_FIRE;
                            System.out.println("Stopping drive");
                            System.out.println("Transitioning to READY_TO_FIRE");
                            preFireStart = System.currentTimeMillis();
                        }
                    } break;
                    case AUTO_STATE_READY_TO_FIRE: {
                        if (System.currentTimeMillis() - preFireStart > PRE_FIRE_TIME && shooterMechanism.isArmed()) {
                            //FIRE!
                            shooterMechanism.fire();
                            currentAutoState = AUTO_STATE_FIRE;
                            System.out.println("Firing");
                            System.out.println("Transitioning to FIRE");

                        }
                    } break;
                    case AUTO_STATE_FIRE: {
                        if (shooterMechanism.isArmed() && postFireStart == -1) {
                            postFireStart = System.currentTimeMillis();
                            System.out.println("Setting Post Fire Start:");
                        }
                        if (shooterMechanism.isArmed() && System.currentTimeMillis() - postFireStart > 1000) {
                            //Lower the scissor
                            currentAutoState = AUTO_STATE_ALIGNMENT;
                            System.out.println("Lowering Arm");
                            System.out.println("Transitioning to ALIGNMENT");
                            //decide which direction we are offset
                            isOffsetLeft = currPos.theta > 0;
                        }
                    } break;
                    case AUTO_STATE_ALIGNMENT: {
                        if (isOffsetLeft) {
                            chassis.drive(0, -0.2);
                            if (currPos.theta < 0) {
                                //we crossed
                                chassis.drive(0,0);
                                currentAutoState = AUTO_STATE_STANDBY;
                                System.out.println("Transitioning to STANDBY");
                            }
                        }
                        else {
                            chassis.drive(0, 0.2);
                            if (currPos.theta > 0) {
                                //we crossed
                                chassis.drive(0,0);
                                currentAutoState = AUTO_STATE_STANDBY;
                                System.out.println("Transitioning to STANDBY");
                            }
                        }
                        
                    } break;
                    case AUTO_STATE_STANDBY: {
                        //Do straight line driving... or something resembling it
                        double angle = currPos.theta;
                        chassis.drive(AUTO_DRIVE_SPEED, -angle * Kp);

                        //Transition Check: Drive 14.5 feet
                        if (currPos.x < 0) {
                            //Stop the motors
                            chassis.drive(0, 0);
                            //switch state to alignment mode
                            currentAutoState = AUTO_STATE_STANDBY2;
                            System.out.println("Stopping drive");
                            System.out.println("Transitioning to STANDBY2");
                        }
                    } break;
                    
                }
            }
        }
        else {
            System.out.println("NOT using HOT program");
            while (isEnabled() && isAutonomous()) {
                //Update the component states
                deadReckoningEngine.updateState();
                feedMechanism.updateState();
                scissorMechanism.updateState();
                shooterMechanism.updateState();

                //update our current position on the field
                currPos = deadReckoningEngine.getCurrentPosition();
                SmartDashboard.putString("Estimated Position", currPos.toString());

                //TODO: Maybe take final adjustment measurements from the camera?

                //Actual state machine implementation
                switch(currentAutoState) {
                    case AUTO_STATE_DRIVE_FWD: {
                        //Do straight line driving... or something resembling it
                        double angle = currPos.theta;
                        chassis.drive(-AUTO_DRIVE_SPEED, -angle * Kp);

                        //Transition Check: Drive 14.5 feet
                        if (currPos.x > 14.5) {
                            //Stop the motors
                            chassis.drive(0, 0);
                            //switch state to alignment mode
                            currentAutoState = AUTO_STATE_ALIGNMENT;
                            System.out.println("Stopping drive");
                            System.out.println("Transitioning to ALIGNMENT");

                        }
                    } break;
                    case AUTO_STATE_ALIGNMENT: {
                        //TODO if we need to do anything here, do it

                        //Transition to next state
                        feedMechanism.lowerArm();
                        currentAutoState = AUTO_STATE_LOWER_GATE;
                        System.out.println("Lowering Arm");
                        System.out.println("Transitioning to LOWER_GATE");

                    } break;
                    case AUTO_STATE_LOWER_GATE: {
                        //Transition Check: Arm MUST be down and not in transit
                        if (!feedMechanism.isArmUp() && !feedMechanism.isArmInTransit()) {
                            scissorMechanism.raiseScissor();
                            currentAutoState = AUTO_STATE_RAISE_SCISSOR;
                            System.out.println("Raising Scissor");
                            System.out.println("Transitioning to RAISE_SCISSOR");

                        }
                    } break;
                    case AUTO_STATE_RAISE_SCISSOR: {
                        //Transition Check: Scissor must be up and not in transit
                        if (scissorMechanism.isScissorUp() && !scissorMechanism.isScissorInTransit()) {
                            //make sure we are armed, then transition
                            shooterMechanism.arm();
                            currentAutoState = AUTO_STATE_READY_TO_FIRE;
                            System.out.println("Arming Shooter");
                            System.out.println("Transitioning to READY_TO_FIRE");
                            preFireStart = System.currentTimeMillis();
                        }
                    } break;
                    case AUTO_STATE_READY_TO_FIRE: {
                        //Transition Check: firing mechanism must be armed
                        if (System.currentTimeMillis() - preFireStart > PRE_FIRE_TIME && shooterMechanism.isArmed()) {
                            //FIRE!
                            shooterMechanism.fire();
                            currentAutoState = AUTO_STATE_FIRE;
                            System.out.println("Firing");
                            System.out.println("Transitioning to FIRE");

                        }
                    } break;
                    case AUTO_STATE_FIRE: {
                        //Transition Check: Back to armed
                        if (shooterMechanism.isArmed() && postFireStart == -1) {
                            postFireStart = System.currentTimeMillis();
                            System.out.println("Setting Post Fire Start:");
                        }
                        if (shooterMechanism.isArmed() && System.currentTimeMillis() - postFireStart > POST_FIRE_TIME) {
                            //Lower the scissor
                            scissorMechanism.lowerScissor();
                            currentAutoState = AUTO_STATE_STANDBY;
                            System.out.println("Lowering Scissor");
                            System.out.println("Transitioning to STANDBY");

                        }
                    } break;
                    case AUTO_STATE_STANDBY: {
                        //TODO do nothing?
                    } break;
                }
            } 
        }
        deadReckoningEngine.stop();
        driverStation.clear();
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
        deadReckoningEngine.start();
        
        //register our start time for safety
        long teleopStartTime = System.currentTimeMillis();
        System.out.println("Teleop Start: " + teleopStartTime);
        safeToOperate = true;
        
        Position currPos = deadReckoningEngine.getCurrentPosition();
        
        SmartDashboard.putString("Estimated Position", currPos.toString());
        
        //Super Shoot Mode: 
        //Drop the arm, Raise the scissor and Fire
        boolean superShootMode = false;
        String superShootState = "init"; //init, gate_lowering, gate_down, scissor_rising, scissor_up, firing, fired
        long preFireStart = -1, postFireStart = -1;
        
        //Default loop
        while (isEnabled() && isOperatorControl()) {
            //1) Sense
            //do a safety check
            if (useSafety && safeToOperate && System.currentTimeMillis() - teleopStartTime > TELEOP_SAFE_TIME) {
                System.out.println("Switching to UNSAFE");
                safeToOperate = false;
                SmartDashboard.putBoolean("Safe To Operate", safeToOperate);
                //this was the initial time that we were set into unsafe mode
                //trigger a fire if we are armed
                if (shooterMechanism.isArmed()) {
                    shooterMechanism.safeFire();
                }
            }
            
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
            
            //Check Super Shoot Mode
            if (superShootMode) {
                if (superShootState.equals("init")) {
                    //drop the gate
                    feedMechanism.lowerArm();
                    superShootState = "gate_lowering";
                }
                else if (superShootState.equals("gate_lowering")) {
                    if (!feedMechanism.isArmUp() && !feedMechanism.isArmInTransit()) {
                        superShootState = "gate_down";
                    }
                }
                else if (superShootState.equals("gate_down")) {
                    scissorMechanism.raiseScissor();
                    superShootState = "scissor_raising";
                }
                else if (superShootState.equals("scissor_raising")) {
                    if (scissorMechanism.isScissorUp() && !scissorMechanism.isScissorInTransit()) {
                        superShootState = "scissor_up";
                        preFireStart = System.currentTimeMillis();
                    }
                }
                else if (superShootState.equals("scissor_up")) {
                    if (System.currentTimeMillis() - preFireStart > 200) {
                        shooterMechanism.fire();
                        superShootState = "firing";
                    }
                }
                else if (superShootState.equals("firing")) {
                    if (shooterMechanism.isArmed()) {
                        superShootState = "fired";
                    }
                }
                else if (superShootState.equals("fired")) {
                    superShootState = "init";
                    superShootMode = false;
                }
            }
            
            //chassis.arcadeDrive(driveStick.getY(), -driveStick.getZ(), true);
            //chassis.tankDrive(driveStick.getY(), driveStick.getThrottle(), true);
            //Use to doDrive method to control our driving
            //we can handle all the exponential stuff there too
            doDrive();
            
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
            if (!superShootMode && safeToOperate && (driveStick.getRawButton(DRIVER_FIRE_BUTTON) || shooterStick.getRawButton(SHOOTER_FIRE_BUTTON)) 
                    && (!driveStick.getRawButton(DRIVER_SHOOT_OVERRIDE_BUTTON) && !shooterStick.getRawButton(SHOOTER_SHOOT_OVERRIDE_BUTTON))) {
                if (!shooterMechanism.isArmed())
                    shooterMechanism.arm();
                else {
                    shooterMechanism.fire();
                }
            }
            
            //override
            if (!superShootMode && safeToOperate && (driveStick.getRawButton(DRIVER_SHOOT_OVERRIDE_BUTTON) || shooterStick.getRawButton(SHOOTER_SHOOT_OVERRIDE_BUTTON))) {
                shooterMechanism.setOverride(true);
                shooterMotor.set(0.3);
            }
            else {
                if (shooterMechanism.getOverride()) {
                    shooterMechanism.setOverride(false);
                    shooterMotor.stopMotor();
                }
                
            }
            
            //Super Shoot Mode
            if (shooterStick.getRawButton(7) && !superShootMode) {
                superShootMode = true; //Put in the superShootMode request
            }
            
            //Finally update driver station
            updateRobotState();
            
            SmartDashboard.putBoolean("Super Shoot Mode", superShootMode);
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

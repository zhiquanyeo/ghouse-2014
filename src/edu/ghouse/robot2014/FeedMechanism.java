/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.robot2014;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author Developer
 */
public class FeedMechanism {
    private DoubleSolenoid solenoid;
    private Victor motor;
    private DigitalInput insideSensor;
    private DigitalInput outsideSensor;
    
    //State variables
    private boolean armUp = true;
    private boolean armInTransit = false;
    
    //to keep track of when we should shut off the  solenoid
    private long stopRequestedTime;
    private boolean stopRequested = false;
    private final int SHUTOFF_TIME = 2000; //Shutoff time in ms
    
    private boolean motorEnabled = true;
    
    public FeedMechanism(DoubleSolenoid solenoid, Victor motor, 
                        DigitalInput insideSensor, DigitalInput outsideSensor) {
        this.solenoid = solenoid;
        this.motor = motor;
        this.insideSensor = insideSensor;
        this.outsideSensor = outsideSensor;
    
        //we can turn off safety for the motor since it's not a drive
        this.motor.setSafetyEnabled(false);
    }
    
    public void raiseArm() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        stopRequested = true;
        stopRequestedTime = System.currentTimeMillis();
    }
    
    public void lowerArm() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
        stopRequested = true;
        stopRequestedTime = System.currentTimeMillis();
    }
    
    public void setMotorEnabled(boolean enabled) {
        this.motorEnabled = enabled;
    }
    
    /**
     * This is where the magic happens
     * We have to keep track of the arm state
     * and update accordingly
     */
    public void updateState() {
        //Handle any solenoid stop orders first
        if (stopRequested) {
            if (System.currentTimeMillis() - stopRequestedTime > SHUTOFF_TIME) {
                stopRequested = false;
                this.solenoid.set(DoubleSolenoid.Value.kOff);
            }
        }
        
        //Check the state of the arm
        if (insideSensor.get() && !outsideSensor.get()) {
            this.armUp = false;
            this.armInTransit = false;
            if (this.motorEnabled) {
                this.motor.set(1);
            }
            else {
                this.motor.stopMotor();
            }
        }
        else if (!insideSensor.get() && outsideSensor.get()) {
            this.armUp = true;
            this.armInTransit = false;
            this.motor.stopMotor();
        }
        else {
            this.armInTransit = true;
            this.motor.stopMotor();
        }
    }
    
    public boolean isArmUp() {
        return this.armUp;
    }
    
    public boolean isArmInTransit() {
        return this.armInTransit;
    }
}

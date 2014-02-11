/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.robot2014;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 * @author Developer
 */
public class ScissorMechanism {
    private DoubleSolenoid solenoid;
    private DigitalInput insideSensor;
    private DigitalInput outsideSensor;
    
    //State variables
    private boolean scissorUp = false;
    private boolean scissorInTransit = false;
    
    //Keep track of when we should shut off
    private long stopRequestedTime;
    private boolean stopRequested = false;
    private final int SHUTOFF_TIME = 3000; //ms
    
    public ScissorMechanism(DoubleSolenoid solenoid, DigitalInput insideSensor, DigitalInput outsideSensor) {
        this.solenoid = solenoid;
        this.insideSensor = insideSensor;
        this.outsideSensor = outsideSensor;
    }
    
    public void raiseScissor() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        stopRequested = true;
        stopRequestedTime = System.currentTimeMillis();
    }
    
    public void lowerScissor() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
        stopRequested = true;
        stopRequestedTime = System.currentTimeMillis();
    }
    
    public void updateState() {
        if (stopRequested) {
            if (System.currentTimeMillis() - stopRequestedTime > SHUTOFF_TIME) {
                stopRequested = false;
                this.solenoid.set(DoubleSolenoid.Value.kOff);
            }
        }
        
        if (!insideSensor.get() && outsideSensor.get()) {
            this.scissorUp = true;
            this.scissorInTransit = false;
        }
        else if (insideSensor.get() && !outsideSensor.get()) {
            this.scissorUp = false;
            this.scissorInTransit = false;
        }
        else {
            this.scissorInTransit = true;
        }
    }
    
    public boolean isScissorUp() {
        return this.scissorUp;
    }
    
    public boolean isScissorInTransit() {
        return this.scissorInTransit;
    }
}

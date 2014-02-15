/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.robot2014;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author Developer
 */
public class ShooterMechanism {
    private Victor motor;
    private DigitalInput shooterSwitch;
    
    private boolean isArmed = false;
    private boolean armRequested = false;
    private boolean fireRequested = false;
    private boolean safeFireRequested = false;
    private long fireRequestedTime;
    private long safeFireRequestedTime;
    private final int FIRE_TIME = 1000;
    private final int SAFE_FIRE_TIME = 500;
    
    private boolean isOverride = false;
    
    public ShooterMechanism(Victor motor, DigitalInput shooterSwitch) {
        this.motor = motor;
        this.shooterSwitch = shooterSwitch;
    }
    
    public void updateState() {
        if (!shooterSwitch.get()) {
            isArmed = true;
        }
        else {
            isArmed = false;
        }
        
        if (armRequested) {
            if (!isArmed) {
                motor.set(0.4);
            }
            else {
                motor.stopMotor();
                armRequested = false;
            }
        }
        
        if (fireRequested) {
            if (System.currentTimeMillis() - fireRequestedTime > FIRE_TIME) {
                motor.stopMotor();
                fireRequested = false;
                arm();
            }
        }
        
        if (safeFireRequested) {
            if (System.currentTimeMillis() - safeFireRequestedTime > SAFE_FIRE_TIME) {
                motor.stopMotor();
                safeFireRequested = false;
            }
        }
    }
    
    public boolean isArmed() {
        return this.isArmed;
    }
    
    public boolean armInProgress() {
        return armRequested;
    }
    
    public boolean fireInProgress() {
        return fireRequested;
    }
    
    public void arm() {
        if (!fireRequested)
            armRequested = true;
    }
    
    public void fire() {
        if (!armRequested && isArmed) {
            fireRequested = true;
            fireRequestedTime = System.currentTimeMillis();
            motor.set(0.4);
        }
    }
    
    public void safeFire() {
        if (!armRequested && isArmed) {
            safeFireRequested = true;
            safeFireRequestedTime = System.currentTimeMillis();
            motor.set(0.4);
        }
    }
    
    public void setOverride(boolean override) {
        this.isOverride = override;
        if (override) {
            motor.stopMotor();
            armRequested = false;
            fireRequested = false;
        }
    }
    
    public boolean getOverride() {
        return this.isOverride;
    }
}

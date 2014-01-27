/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.drivesystem;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 *
 * @author Robotics
 * 
 * A class to control multiple CAN Jaguars as one unit
 */
public class MultiCANJaguar implements SpeedController, LiveWindowSendable, MotorSafety, PIDOutput, Sendable {
    private CANJaguar controllers[];
    private double currentSpeed = 0.0;
    
    /**
     * Constructor. Takes in an array of CAN channel numbers. Defaults to percent Vbus mode
     * @param channels An array of CAN channel numbers
     * @throws CANTimeoutException 
     */
    public MultiCANJaguar(int channels[]) throws CANTimeoutException {
        this(channels, CANJaguar.ControlMode.kPercentVbus);
    }
    
    /**
     * Constructor. Takes in an array of CAN channel numbers.
     * @param channels An array of CAN channel numbers
     * @param controlMode
     * @throws CANTimeoutException 
     */
    public MultiCANJaguar(int channels[], CANJaguar.ControlMode controlMode) throws CANTimeoutException {
        controllers = new CANJaguar[channels.length];
        
        for (int i = 0; i < channels.length; i++) {
            controllers[i] = new CANJaguar(channels[i], controlMode);
        }
    }

    public double get() {
        return currentSpeed;
    }

    public void set(double outputValue, byte syncGroup) {
        currentSpeed = outputValue;
        try {
            for (int i = 0; i < controllers.length; i++) {
                controllers[i].setX(outputValue, syncGroup);
            }
        } catch (CANTimeoutException e) {
            e.printStackTrace();
        }
    }

    public void set(double outputValue) {
        set(outputValue, (byte)0);
    }

    public void disable() {
        try {
            for (int i = 0; i < controllers.length; i++) {
                controllers[i].disableControl();
            }
        } catch (CANTimeoutException e) {
            e.printStackTrace();
        }
    }

    public void pidWrite(double outputValue) {
        set(outputValue);
    }

    public void updateTable() {
        if (controllers.length > 0) {
            controllers[0].updateTable();
        }
    }

    public void startLiveWindowMode() {
        if (controllers.length > 0) {
            controllers[0].startLiveWindowMode();
        }
    }

    public void stopLiveWindowMode() {
        if (controllers.length > 0) {
            controllers[0].stopLiveWindowMode();
        }
    }

    public void initTable(ITable itable) {
        if (controllers.length > 0) {
            controllers[0].initTable(itable);
        }
    }

    public ITable getTable() {
        if (controllers.length > 0) {
            return controllers[0].getTable();
        }
        return null;
    }

    public String getSmartDashboardType() {
        if (controllers.length > 0) {
            return controllers[0].getSmartDashboardType();
        }
        return "UNKNOWN TYPE";
    }

    public void setExpiration(double timeout) {
        for (int i = 0; i < controllers.length; i++) {
            controllers[i].setExpiration(timeout);
        }
    }

    public double getExpiration() {
        if (controllers.length > 0) {
            return controllers[0].getExpiration();
        }
        return 0;
    }

    public boolean isAlive() {
        boolean isAlive = true;
        if (controllers.length == 0)
            isAlive = false;
        else {
            for (int i = 0; i < controllers.length; i++) {
                isAlive &= controllers[i].isAlive();
            }
        }
        return isAlive;
    }

    public void stopMotor() {
        try {
            for (int i = 0; i < controllers.length; i++) {
                controllers[i].disableControl();
            }
        } catch (CANTimeoutException e) {
            e.printStackTrace();
        }
    }

    public void setSafetyEnabled(boolean safetyEnabled) {
        for (int i = 0; i < controllers.length; i++) {
            controllers[i].setSafetyEnabled(safetyEnabled);
        }
    }

    public boolean isSafetyEnabled() {
        boolean isEnabled = true;
        if (controllers.length == 0)
            isEnabled = false;
        else {
            for (int i = 0; i < controllers.length; i++) {
                isEnabled &= controllers[i].isSafetyEnabled();
            }
        }
        return isEnabled;
    }

    public String getDescription() {
        return "Multi CAN Jaguar Speed Controller";
    }
    
}

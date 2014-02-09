/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.positional;

import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author Developer
 */
public class DeadReckoningEngine {
    //Helper class for position
    public class Position {
        public double x;
        public double y;
        public double theta;
    }
    
    private Position currentPosition = new Position();
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    
    private double lastLeftValue = 0;
    private double lastRightValue = 0;
    
    private double axleLength;
    
    private boolean isRunning = false;
    
    public DeadReckoningEngine(double axleLength, Encoder leftEnc, Encoder rightEnc) {
        this.leftEncoder = leftEnc;
        this.rightEncoder = rightEnc;
        
        this.axleLength = axleLength;
        
        //Force a reset
        reset();
    }
    
    public void reset() {
        this.currentPosition.x = 0;
        this.currentPosition.y = 0;
        this.currentPosition.theta = 0;
        
        this.lastLeftValue = this.leftEncoder.getDistance();
        this.lastRightValue = this.rightEncoder.getDistance();
    }
    
    public void start() {
        isRunning = true;
    }
    
    public void stop() {
        isRunning = false;
    }
    
    public void updateState() {
        if (!isRunning) {
            return;
        }
        
        double currLeftReading = this.leftEncoder.getDistance();
        double currRightReading = this.rightEncoder.getDistance();
        
        double leftDist = currLeftReading - this.lastLeftValue;
        double rightDist = currRightReading - this.lastRightValue;
        
        this.lastLeftValue = currLeftReading;
        this.lastRightValue = currRightReading;
        
        double cosCurrent = Math.cos(this.currentPosition.theta);
        double sinCurrent = Math.sin(this.currentPosition.theta);
        
        if (leftDist == rightDist) {
            //straight line
            this.currentPosition.x += leftDist * cosCurrent;
            this.currentPosition.y += leftDist * sinCurrent;
        }
        else {
            double expr1 = this.axleLength * (rightDist + leftDist) / 2.0 / (rightDist - leftDist);
            double right_minus_left = rightDist - leftDist;
            this.currentPosition.x += expr1 * (Math.sin(right_minus_left / this.axleLength + this.currentPosition.theta) - sinCurrent);
            this.currentPosition.y += expr1 * (Math.cos(right_minus_left / this.axleLength + this.currentPosition.theta) - cosCurrent);
            this.currentPosition.theta += right_minus_left / this.axleLength;
            
            //Normalize to -PI,+PI 
            while (this.currentPosition.theta > Math.PI) {
                this.currentPosition.theta -= (2.0 * Math.PI);
            }
            while (this.currentPosition.theta < -Math.PI) {
                this.currentPosition.theta += (2.0 * Math.PI);
            }
        }
    }
    
    public Position getCurrentPosition() {
        return this.currentPosition;
    }
}

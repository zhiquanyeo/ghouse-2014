/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Programmer
 */
public class DebugRobotDrive extends RobotDrive {

    public DebugRobotDrive(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
    }

    public void tankDrive(double leftValue, double rightValue, boolean squaredInputs) {
        // square the inputs (while preserving the sign) to increase fine control while permitting full power
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);
        if(squaredInputs) {
            if (leftValue >= 0.0) {
                leftValue = (leftValue * leftValue);
            } else {
                leftValue = -(leftValue * leftValue);
            }
            if (rightValue >= 0.0) {
                rightValue = (rightValue * rightValue);
            } else {
                rightValue = -(rightValue * rightValue);
            }
        }
        System.out.println("Left Val: " + leftValue + ", Right Val: " + rightValue);
        SmartDashboard.putNumber("DEBUG_DRIVE_LEFT_VALUE", leftValue);
        SmartDashboard.putNumber("DEBUG_DRIVE_RIGHT_VALUE", rightValue);
        setLeftRightMotorOutputs(leftValue, rightValue);
    }
}

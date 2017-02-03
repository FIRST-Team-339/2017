/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Autonomous.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for teleop mode
// should go here. Will be called each time the robot enters
// teleop mode.
// -----------------------------------------------------
// Periodic() - Periodic code for teleop mode should
// go here. Will be called periodically at a regular rate while
// the robot is in teleop mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.Transmission.MotorDirection;
import org.usfirst.frc.team339.Utils.CANPIDTuner;
import org.usfirst.frc.team339.Utils.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Teleop
{
/**
 * User Initialization code for teleop mode should go here. Will be called
 * once when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void init ()
{
    // --------------------------------------
    // initialize all encoders here
    // --------------------------------------
    Hardware.rightRearEncoder.reset();
    Hardware.leftRearEncoder.reset();
    // --------------------------------------
    // initialize all motors here
    // --------------------------------------
    Hardware.leftRearMotor.set(0.0);
    Hardware.rightRearMotor.set(0.0);
    Hardware.rightFrontMotor.set(0.0);
    Hardware.leftFrontMotor.set(0.0);

    Hardware.rightFrontMotor.setInverted(true); // TODO takeout
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    // Hardware.leftRearMotor.setInverted(true);
    Hardware.mecanumDrive.setDirectionalDeadzone(0.2, 0);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);

    Hardware.isUsingMecanum = true;
    Hardware.twoJoystickControl = false;

    // Hardware.tankDrive.setGear(Hardware.tankDrive.getMaxGear());
    // Hardware.leftUS.setScalingFactor(.13);
    // Hardware.leftUS.setOffsetDistanceFromNearestBummper(0);
    // Hardware.rightUS.setScalingFactor(.13); TODO
    // Hardware.rightUS.setOffsetDistanceFromNearestBummper(0);
    // Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(1);
    // Hardware.leftUS.setNumberOfItemsToCheckBackwardForValidity(1);
    // Hardware.LeftUS.setConfidenceCalculationsOn(false);
    // Hardware.RightUS.setConfidenceCalculationsOn(false);

    Hardware.tankDrive.setRightMotorDirection(MotorDirection.REVERSED);
    testTuner.setPID(.15, 0.00065, 0.9);
    testTuner.setupMotorController(
            FeedbackDevice.CtreMagEncoder_Relative,
            TalonControlMode.Speed, 1024, true);
    testTuner.setupDashboard();
    testTuner.setSetpoint(0);
    testTuner.setErrorThreshold(15);
    testTuner.update();
    SmartDashboard.putNumber("F", 0.0);
    SmartDashboard.putNumber("Threshold",
            testTuner.getErrorThreshold());
} // end Init

public static CANPIDTuner testTuner = new CANPIDTuner(1,
        Hardware.shooterMotor, true,
        10);

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
static double currentLargestDrop = 0;

static boolean errorDetected = false;

public static void periodic ()
{
    testTuner.update();
    testTuner.setErrorThreshold(SmartDashboard.getNumber("Threshold",
            testTuner.getErrorThreshold()));
    Hardware.shooterMotor.setF(SmartDashboard.getNumber("F", 0.0));
    // System.out.println("Output voltage: "
    // + Hardware.shooterMotor.getOutputVoltage());
    // System.out.println("P, I, D: " + Hardware.shooterMotor.getP() + ", "
    // + Hardware.shooterMotor.getI() + ", "
    // + Hardware.shooterMotor.getD());
    // System.out.println(
    // "Setpoint: " + Hardware.shooterMotor.getSetpoint());
    // System.out.println("Error: " + Hardware.shooterMotor.getError());
    // System.out.println("Speed: " + Hardware.shooterMotor.getSpeed());
    if (Math.abs(Hardware.shooterMotor.getError()) > testTuner
            .getErrorThreshold() * 4.0 && errorDetected == false)
        {
        System.out.println("Error detected: timing.");
        errorDetected = true;
        Hardware.kilroyTimer.reset();
        Hardware.kilroyTimer.start();
        if (Math.abs(
                Hardware.shooterMotor.getError()) > currentLargestDrop)
            {
            currentLargestDrop = Math
                    .abs(Hardware.shooterMotor.getError());
            }
        }
    else if (Math.abs(Hardware.shooterMotor.getError()) < testTuner
            .getErrorThreshold() * 4.0 && errorDetected == true)
        {
        errorDetected = false;
        System.out.println("Took " + Hardware.kilroyTimer.get()
                + " seconds to correct");
        System.out.println("Largest error: " + currentLargestDrop);
        currentLargestDrop = 0.0;
        }

    // if (Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // Hardware.ringlightRelay.set(Relay.Value.kOn);
    // }
    // else
    // {
    // Hardware.ringlightRelay.set(Relay.Value.kOff);
    // }

    // Print out any data we want from the hardware elements.
    printStatements();

    // =================================================================
    // Driving code
    // =================================================================

    // if (Hardware.rightDriver.getTrigger())
    // {
    // rotationValue = Hardware.rightDriver.getTwist();
    // }
    // else
    // {
    // rotationValue = 0.0;
    // }
    //
    // if (!isAligning)
    // if (Hardware.isUsingMecanum == true
    // && Hardware.twoJoystickControl == false)
    // {
    // Hardware.mecanumDrive.drive(
    // Hardware.rightDriver.getMagnitude(),
    // Hardware.rightDriver.getDirectionDegrees(),
    // rotationValue, Hardware.rightDriver.getY(),
    // Hardware.rightDriver.getX());
    // }
    // else if (Hardware.isUsingMecanum == true
    // && Hardware.twoJoystickControl == true)
    // {
    // Hardware.mecanumDrive.drive(
    // Hardware.rightDriver.getMagnitude(),
    // Hardware.rightDriver.getDirectionDegrees(),
    // Hardware.leftDriver.getX(),
    // Hardware.rightDriver.getY(),
    // Hardware.rightDriver.getX());
    // }
    // else
    // {
    // Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
    // Hardware.leftDriver.getY());
    // }
    // // System.out.println(Hardware.rightDriver.getTwist());
    // // System.out.println(Hardware.rightDriver.getMagnitude());
    //
    // // Testing turn by degrees
    // if (Hardware.leftDriver.getRawButton(2))
    // {
    // turnDegrees = 90;
    // isTurning = true;
    // // Hardware.driveGyro.reset();
    // // TODO
    // }
    //
    // if (isTurning)
    // {
    // isTurning = !Hardware.autoDrive.turnDegrees(turnDegrees);
    // }
    //
    //
    // // Testing driveInches
    // if (Hardware.rightDriver.getRawButton(2))
    // {
    // isDrivingInches = true;
    // }
    //
    // if (isDrivingInches)
    // {
    // Hardware.autoDrive.driveInches(24, .4);
    // }
    //
    //
    // // =================================================================
    // // CAMERA CODE
    // // =================================================================
    //
    // // "Cancel basically everything" button
    // if (Hardware.leftOperator.getRawButton(7)
    // || Hardware.leftOperator.getRawButton(6))
    // {
    // System.out.println("Cancelling everything");
    // isAligning = false;
    // isStrafingToTarget = false;
    // isDrivingInches = false;
    // isTurning = false;
    // }
    //
    // // Testing aligning to target
    // if (Hardware.leftOperator.getRawButton(8))
    // isAligning = true;
    //
    // if (isAligning)
    // {
    // alignValue = Hardware.autoDrive.alignToGear(CAMERA_ALIGN_CENTER,
    // CAMERA_ALIGN_SPEED, CAMERA_ALIGN_DEADBAND);
    // if (alignValue == Drive.AlignReturnType.ALIGNED)
    // {
    // System.out.println("We are aligned!");
    // isAligning = false;
    // }
    // else if (alignValue == Drive.AlignReturnType.MISALIGNED)
    // {
    // System.out.println("We are not aligned!");
    // }
    // else if (alignValue == Drive.AlignReturnType.NO_BLOBS)
    // {
    // System.out.println("We don't see anything!");
    // }
    // }
    // // Testing Strafe to target
    // if (Hardware.rightOperator.getRawButton(8))
    // isStrafingToTarget = true;
    //
    // if (isStrafingToTarget)
    // {
    // alignValue = Hardware.autoDrive.strafeToGear(.4, .2,
    // CAMERA_ALIGN_DEADBAND, CAMERA_ALIGN_CENTER, 20);
    // if (alignValue == Drive.AlignReturnType.ALIGNED)
    // {
    // System.out.println("We are aligned!");
    // }
    // else if (alignValue == Drive.AlignReturnType.MISALIGNED)
    // {
    // System.out.println("WE are NOT aligned!");
    // }
    // else if (alignValue == Drive.AlignReturnType.NO_BLOBS)
    // {
    // System.out.println("We have no blobs!");
    // }
    // else if (alignValue == Drive.AlignReturnType.CLOSE_ENOUGH)
    // {
    // System.out.println("We are good to go!");
    // isStrafingToTarget = false;
    // }
    // }
    //
    // Hardware.axisCamera
    // .takeSinglePicture(Hardware.leftOperator.getRawButton(8));TODO
} // end Periodic

// private static boolean isSpeedTesting = false;


private static double rotationValue = 0.0;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static boolean isAligning = false;

private static boolean isTurning = false;

private static double turnDegrees = 0.0;

private static boolean isStrafingToTarget = false;

private static boolean isDrivingInches = false;




/**
 * stores print statements for future use in the print "bank", statements
 * are commented out when not in use, when you write a new print
 * statement, "deposit" the statement in the correct "bank"
 * do not "withdraw" statements, unless directed to.
 * 
 * NOTE: Keep the groupings below, which coorespond in number and
 * order as the hardware declarations in the HARDWARE class
 * 
 * @author Ashley Espeland
 * @written 1/28/16
 * 
 *          Edited by Ryan McGee
 * 
 */
public static void printStatements ()
{
    // =================================
    // Motor controllers
    // prints value of the motors
    // =================================

    // =================================
    // CAN items
    // prints value of the CAN controllers
    // =================================
    // Hardware.CAN.printAllPDPChannels();

    // =================================
    // Relay
    // prints value of the relay states
    // =================================

    // =================================
    // Digital Inputs
    // =================================
    // ---------------------------------
    // Switches
    // prints state of switches
    // ---------------------------------

    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------
    // System.out.println("Right Encoder: "
    // + Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Left Encoder: "
    // + Hardware.autoDrive.getLeftRearEncoderDistance());

    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------

    // =================================
    // Pneumatics
    // =================================
    // ---------------------------------
    // Compressor
    // prints information on the compressor
    // ---------------------------------

    // ---------------------------------
    // Solenoids
    // prints the state of solenoids
    // ---------------------------------

    // =================================
    // Analogs
    // =================================

    // System.out.println("LeftUS = "
    // + Hardware.leftUS.getDistanceFromNearestBumper());
    //
    // System.out.println("RightUS = "
    // + Hardware.rightUS.getDistanceFromNearestBumper());


    // ---------------------------------
    // pots
    // where the pot is turned to
    // ---------------------------------

    // =================================
    // Connection Items
    // =================================
    // ---------------------------------
    // Cameras
    // prints any camera information required
    // ---------------------------------

    // =================================
    // Driver station
    // =================================
    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------
    // System.out.println("Left Joystick: " +
    // Hardware.leftDriver.getDirectionDegrees());
    // System.out.println("Twist: " + Hardware.leftDriver.getTwist());
    // System.out
    // .println("Right Joystick: " + Hardware.rightDriver.getY());
    // System.out
    // .println("Left Operator: " + Hardware.leftOperator.getY());
    // System.out.println(
    // "Right Operator: " + Hardware.rightOperator.getY());

    // =================================
    // Kilroy ancillary items
    // =================================
    // ---------------------------------
    // timers
    // what time does the timer have now
    // ---------------------------------

} // end printStatements

/*
 * =============================================== Constants
 * ===============================================
 */
private final static double CAMERA_ALIGN_SPEED = .5;

//// The dead zone for the aligning TODO
private final static double CAMERA_ALIGN_DEADBAND = 10.0
        / Hardware.axisCamera.getHorizontalResolution();

private final static double CAMERA_ALIGN_CENTER = 271.8;

// ==========================================
// TUNEABLES
// ==========================================

} // end class

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

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import edu.wpi.first.wpilibj.Relay;

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


    Hardware.tankDrive.setGear(Hardware.tankDrive.getMaxGear());
    Hardware.leftUS.setScalingFactor(.13);
    Hardware.leftUS.setOffsetDistanceFromNearestBummper(0);
    Hardware.rightUS.setScalingFactor(.13);
    Hardware.rightUS.setOffsetDistanceFromNearestBummper(0);
    Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(1);
    Hardware.leftUS.setNumberOfItemsToCheckBackwardForValidity(1);
    // Hardware.LeftUS.setConfidenceCalculationsOn(false);
    // Hardware.RightUS.setConfidenceCalculationsOn(false);



} // end Init

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{
    if (Hardware.ringlightSwitch.isOnCheckNow())
        {
        Hardware.ringlightRelay.set(Relay.Value.kOn);
        }
    else
        {
        Hardware.ringlightRelay.set(Relay.Value.kOff);
        }

    // Print out any data we want from the hardware elements.
    printStatements();

    // =================================================================
    // Driving code
    // =================================================================


    if (Hardware.rightDriver.getTrigger())
        {
        rotationValue = Hardware.rightDriver.getTwist();
        }
    else
        {
        rotationValue = 0.0;
        }


    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    if (Hardware.isUsingMecanum == true
            && Hardware.twoJoystickControl == false)
        {
        Hardware.mecanumDrive.drive(Hardware.rightDriver.getMagnitude(),
                Hardware.rightDriver.getDirectionDegrees(),
                rotationValue, Hardware.rightDriver.getY(),
                Hardware.rightDriver.getX());
        }
    else if (Hardware.isUsingMecanum == true
            && Hardware.twoJoystickControl == true)
        {
        Hardware.mecanumDrive.drive(Hardware.rightDriver.getMagnitude(),
                Hardware.rightDriver.getDirectionDegrees(),
                Hardware.leftDriver.getX(),
                Hardware.rightDriver.getY(),
                Hardware.rightDriver.getX());
        }
    else
        {
        Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
                Hardware.leftDriver.getY());
        }
    System.out.println(Hardware.rightDriver.getDirectionDegrees());
    // System.out.println(Hardware.rightDriver.getTwist());
    // System.out.println(Hardware.rightDriver.getMagnitude());


    // Testing turn by degrees
    if (Hardware.leftDriver.getRawButton(2))
        {
        turnDegrees = 90;
        isTurning = true;
        }

    if (isTurning)
        {
        isTurning = !Hardware.autoDrive.turnDegrees(turnDegrees);
        }

    // Testing driveInches
    if (Hardware.rightDriver.getRawButton(2))
        {
        isDrivingInches = true;
        }

    if (isDrivingInches)
        {
        Hardware.autoDrive.driveInches(24, .4);
        }





} // end Periodic

// private static boolean isSpeedTesting = false;


public static void alignToGearPeg ()
{

        isAligning = false;
        isStrafingToTarget = false;
        isDrivingInches = false;
        isTurning = false;//TODO Not sure what the above block is doing, just make sure it's supposed to be there
    // Testing strafe to target on button 8 on the RIGHT operator
    if (Hardware.rightOperator.getRawButton(8))
        isStrafingToTarget = true;

    if (isStrafingToTarget)
        {
        alignValue = Hardware.autoDrive.strafeToGear(.4, .2,
                CAMERA_ALIGN_DEADBAND, CAMERA_ALIGN_CENTER, 20);
        if (alignValue == Drive.AlignReturnType.ALIGNED)
            {
            System.out.println("We are aligned!");
            isStrafingToTarget = true;
            }
        else if (alignValue == Drive.AlignReturnType.MISALIGNED)
            {
            System.out.println("WE are NOT aligned!");
            isStrafingToTarget = true;
            }
        else if (alignValue == Drive.AlignReturnType.NO_BLOBS)
            {
            System.out.println("We have no blobs!");
            isStrafingToTarget = true;
            }
        else if (alignValue == Drive.AlignReturnType.CLOSE_ENOUGH)
            {
            System.out.println("We are good to go!");
            isStrafingToTarget = false;
            }
        }

    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8));


}


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
    System.out.println("Right Encoder: "
            + Hardware.autoDrive.getRightRearEncoderDistance());
    System.out.println("Left Encoder: "
            + Hardware.autoDrive.getLeftRearEncoderDistance());

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
    System.out.println("LeftUS = "
            + Hardware.leftUS.getDistanceFromNearestBumper());

    System.out.println("RightUS = "
            + Hardware.rightUS.getDistanceFromNearestBumper());

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

// The dead zone for the aligning
private final static double CAMERA_ALIGN_DEADBAND = 10.0
        / Hardware.axisCamera.getHorizontalResolution();

private final static double CAMERA_ALIGN_CENTER = 271.8;

// ==========================================
// TUNEABLES
// ==========================================

} // end class

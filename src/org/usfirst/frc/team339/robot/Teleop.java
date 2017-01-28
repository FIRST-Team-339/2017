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

    Hardware.mecanumDrive.setMecanumJoystickReversed(false);

    Hardware.tankDrive.setGear(Hardware.tankDrive.getMaxGear());

    Hardware.mecanumDrive.setDirectionalDeadzone(0.2, 0);

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

    // creating new instance of Transmission Mecanum

    if (!isAligning)
        if (Hardware.usingMecanum == true)
            {
            Hardware.mecanumDrive.drive(
                    Hardware.rightDriver.getMagnitude(),
                    Hardware.rightDriver.getDirectionDegrees(),
                    rotationValue);
            }
        else
            {
            Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
                    Hardware.leftDriver.getY());
            }


    // -----------------------------------------------------------------


    // =================================================================
    // Camera Code
    // =================================================================

    // Starts the aligning process with button 8 on the left operator
    if (Hardware.leftOperator.getRawButton(8))
        {
        isAligning = true;

        }

    // Determines whether or not we are aligned yet, and stops when we are.
    // If we don't see any blobs, tell us and continue aligning.
    if (isAligning)
        {
        alignValue = Hardware.autoDrive.alignToGear(
                CAMERA_ALIGN_CENTER,
                CAMERA_ALIGN_SPEED,
                CAMERA_ALIGN_DEADBAND);

        if (alignValue == Drive.AlignReturnType.MISALIGNED)
            {
            isAligning = true;
            System.out.println("we are misaligned!!");
            }
        else if (alignValue == Drive.AlignReturnType.ALIGNED)
            {
            isAligning = false;
            System.out.println("We are aligned!!");
            }
        else if (alignValue == Drive.AlignReturnType.NO_BLOBS)
            {
            System.out.println("We don't see anything!");
            isAligning = true;
            }
        }

    // Cancel auto aligning
    if (isAligning && Hardware.leftOperator.getRawButton(7))
        {
        isAligning = false;
        }

    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8));



    // -----------------------------------------------------------------
} // end Periodic

private static double rotationValue = 0.0;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static boolean isAligning = false;



/**
 * stores print statements for future use in the print "bank", statements
 * are commented out when not in use, when you write a new print statement,
 * "deposit" the statement in the correct "bank" do not "withdraw"
 * statements, unless directed to.
 * 
 * NOTE: Keep the groupings below, which coorespond in number and order as
 * the hardware declarations in the HARDWARE class
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
    // + Hardware.rightRearEncoder.getDistance());

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

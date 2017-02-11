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
// written. Some of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// -----------------------------------------------------
// Periodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Drive.AlignReturnType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay.Value;


/**
 * An Autonomous class.
 * This class <b>beautifully</b> uses state machines in order to periodically
 * execute instructions during the Autonomous period.
 * 
 * This class contains all of the user code for the Autonomous part
 * of the
 * match, namely, the Init and Periodic code
 * 
 * 
 * @author Michael Andrzej Klaczynski
 * @written at the eleventh stroke of midnight, the 28th of January,
 *          Year of our LORD 2016. Rewritten ever thereafter.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Autonomous
{
// ==========================================
// AUTO STATES
// ==========================================
/**
 * The overarching states of autonomous mode.
 * Each state represents a set of instructions the robot must execute
 * periodically at a given time.
 * These states are mainly used in the runMainStateMachine() method
 * 
 */
private static enum MainState
    {
    INIT,

    DELAY_BEFORE_START,

    DRIVE_FORWARD_TO_CENTER_SLOW,

    DRIVE_FORWARD_TO_CENTER_MED,

    DRIVE_FORWARD_TO_CENTER,

    DRIVE_FORWARD_TO_SIDES_SLOW,

    DRIVE_FORWARD_TO_SIDES_MED,

    DRIVE_FORWARD_TO_SIDES,

    TURN_TO_GEAR_PEG,

    DRIVE_TO_GEAR_WITH_CAMERA,

    DRIVE_CAREFULLY_TO_PEG,

    WIGGLE_WIGGLE,

    WAIT_FOR_GEAR_EXODUS,

    DELAY_AFTER_GEAR_EXODUS,

    DRIVE_AWAY_FROM_PEG,

    TURN_TO_HOPPER,

    DRIVE_UP_TO_HOPPER,

    DRIVE_BACKWARDS_TO_FIRERANGE,

    DRIVE_INTO_RANGE_WITH_CAMERA,

    ALIGN_TO_FIRE,

    FIRE,

    DONE
    }

private static enum AutoProgram
    {
    INIT,

    CENTER_GEAR_PLACEMENT,

    RIGHT_PATH,

    LEFT_PATH,

    DELAY_AFTER_GEAR_EXODUS, DONE
    }



// ==================================
// VARIABLES
// ==================================

private static Drive.AlignReturnType cameraState = Drive.AlignReturnType.NO_BLOBS;

// ==========================================
// TUNEABLES
// ==========================================

private static final double ALIGN_CORRECT_SPEED = .2;

private static final double ALIGN_DRIVE_SPEED = .3;

private static final double ALIGN_DEADBAND = 10 // +/- pixels
        / Hardware.axisCamera.getHorizontalResolution();

private static final double ALIGN_ACCEPTED_CENTER = .5; // Relative coordinates

private static final int ALIGN_DISTANCE_FROM_GOAL = 20;

/**
 * User-Initialization code for autonomous mode should go here. Will be
 * called once when the robot enters autonomous mode.
 *
 * @author Nathanial Lydick
 *
 * @written Jan 13, 2015
 */

public static void init ()
{
    // Sets the scaling factor and general ultrasonic stuff
    Hardware.rightUS.setScalingFactor(.13);
    Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
    Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);
} // end Init

/**
 * User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{

    switch (autoPath)
        {

        case INIT:
            // get the auto program we want to run, get delay pot.
            if (Hardware.enableAutonomous.isOn())
                {
                delayTime = Hardware.delayPot.get() * (5 / 270);
                if (Hardware.driverStation
                        .getAlliance() == Alliance.Red)
                    {
                    isRedAlliance = true;
                    }
                if (Hardware.pathSelector.isOn())
                    {
                    autoPath = AutoProgram.CENTER_GEAR_PLACEMENT;
                    break;
                    }
                if (Hardware.rightPath.isOn())
                    {
                    autoPath = AutoProgram.RIGHT_PATH;
                    break;
                    }
                if (Hardware.leftPath.isOn())
                    {
                    autoPath = AutoProgram.LEFT_PATH;
                    break;
                    }
                autoPath = AutoProgram.DONE;
                }
            else
                {
                autoPath = AutoProgram.DONE;
                }
            break;
        case CENTER_GEAR_PLACEMENT:
            if (placeCenterGearPath())
                autoPath = AutoProgram.DONE;
            break;
        case RIGHT_PATH:
            if (rightSidePath())
                autoPath = AutoProgram.DONE;
            break;
        case LEFT_PATH:
            if (leftSidePath())
                autoPath = AutoProgram.DONE;
            break;
        case DONE:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            Hardware.ringlightRelay.set(Value.kOff);
            break;
        }

} // end Periodic


private static MainState currentState = MainState.INIT;

private static AutoProgram autoPath = AutoProgram.INIT;

private static double delayTime = 5;

private static boolean isRedAlliance = false;

private static boolean goForFire = false;

private static boolean goForHopper = false;

private static boolean driveToTargetFirstStart = true;

private static boolean placeCenterGearPath ()
{
    System.out.println("CurrentState = " + currentState);
    System.out.println("Right US: "
            + Hardware.rightUS.getDistanceFromNearestBumper());
    switch (currentState)
        {
        case INIT:
            // zero out all the sensors, reset timers, etc.
            Hardware.autoStateTimer.start();
            Hardware.ringlightRelay.set(Value.kOn);
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            // stop all the motors to feed the watchdog
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // wait for timer to run out
            if (Hardware.autoStateTimer.get() >= delayTime)
                {
                currentState = MainState.DRIVE_FORWARD_TO_CENTER_SLOW;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case DRIVE_FORWARD_TO_CENTER_SLOW:
            // drive at our slow speed. wait a certain time. Could also check
            // distance for safety
            Hardware.mecanumDrive.drive(.4, 0, 0);// TODO magic number
            if (Hardware.autoStateTimer.get() >= .3)
                {
                currentState = MainState.DRIVE_FORWARD_TO_CENTER_MED;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case DRIVE_FORWARD_TO_CENTER_MED:
            // drive at our medium speed. wait a certain time to move on.
            Hardware.mecanumDrive.drive(.6, 0, 0);
            if (Hardware.autoStateTimer.get() >= .3)
                {
                currentState = MainState.DRIVE_FORWARD_TO_CENTER;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }

            break;
        case DRIVE_FORWARD_TO_CENTER:
            // If we see blobs, hand over control to camera, otherwise, go
            // forward. Check to make sure we haven't gone too far.
            Hardware.imageProcessor.processImage();
            if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
                {
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
                }
            else
                currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            break;
        case DRIVE_TO_GEAR_WITH_CAMERA:
            // Get our return type from the strafe to gear.

            // NOTE: if the constructor for autoDrive uses a mecanum
            // transmission,
            // we will strafe. If it uses a four wheel transmission, it will
            // wiggle wiggle on it's way to the peg
            cameraState = Hardware.autoDrive.strafeToGear(
                    ALIGN_DRIVE_SPEED, ALIGN_CORRECT_SPEED,
                    ALIGN_DEADBAND, ALIGN_ACCEPTED_CENTER,
                    ALIGN_DISTANCE_FROM_GOAL);
            System.out.println(
                    Hardware.rightUS.getDistanceFromNearestBumper());

            if (cameraState == AlignReturnType.NO_BLOBS)
                // If we don't see anything, just drive forwards till we are
                // close enough
                currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            else if (cameraState == AlignReturnType.CLOSE_ENOUGH)
                // If we are close enough to the wall, stop.
                currentState = MainState.WAIT_FOR_GEAR_EXODUS;
            else
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;

            break;
        case DRIVE_CAREFULLY_TO_PEG:

            if (Hardware.rightUS
                    .getDistanceFromNearestBumper() >= ALIGN_DISTANCE_FROM_GOAL)
                {
                Hardware.autoDrive.drive(.5, 0);
                }
            else
                {
                // desired distance from wall when we start
                currentState = MainState.WAIT_FOR_GEAR_EXODUS;
                }
            break;
        case WAIT_FOR_GEAR_EXODUS:
            Hardware.ringlightRelay.set(Value.kOff);
            if (Hardware.gearLimitSwitch.isOn() == false)
                {
                Hardware.autoDrive.drive(0.0, 0.0);
                currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            if (Hardware.autoStateTimer.get() >= 1.5)
                {
                Hardware.autoDrive.resetEncoders();
                currentState = MainState.DRIVE_AWAY_FROM_PEG;
                }
            break;
        case DRIVE_AWAY_FROM_PEG:
            if (Hardware.autoDrive.driveInches(36.0, -.5))
                {
                currentState = MainState.DONE;
                }

            break;
        default:
        case DONE:
            return true;
        }
    return false;
}

private static boolean rightSidePath ()
{
    System.out.println("Current State = " + currentState);
    switch (currentState)
        {
        case INIT:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            Hardware.autoStateTimer.reset();
            Hardware.autoStateTimer.start();
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            if (Hardware.autoStateTimer.get() > delayTime)
                currentState = MainState.DRIVE_FORWARD_TO_SIDES_SLOW;
            break;
        case DRIVE_FORWARD_TO_SIDES_SLOW:
            currentState = MainState.DRIVE_FORWARD_TO_SIDES_MED;
            break;
        case DRIVE_FORWARD_TO_SIDES_MED:
            currentState = MainState.DRIVE_FORWARD_TO_SIDES;
            break;
        case DRIVE_FORWARD_TO_SIDES:
            currentState = MainState.TURN_TO_GEAR_PEG;
            break;
        case TURN_TO_GEAR_PEG:
            // turn left on both red and blue
            if (false)
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
            currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            break;
        case DRIVE_TO_GEAR_WITH_CAMERA:
            if (false)
                {
                currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
                }
            break;
        case DRIVE_CAREFULLY_TO_PEG:
            currentState = MainState.WIGGLE_WIGGLE;
            break;
        case WIGGLE_WIGGLE:
            currentState = MainState.WAIT_FOR_GEAR_EXODUS;
            break;
        case WAIT_FOR_GEAR_EXODUS:
            currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            currentState = MainState.DRIVE_AWAY_FROM_PEG;
            break;
        case DRIVE_AWAY_FROM_PEG:
            if (isRedAlliance && goForFire)
                {
                currentState = MainState.DRIVE_BACKWARDS_TO_FIRERANGE;
                }
            else if (goForHopper)
                {
                currentState = MainState.TURN_TO_HOPPER;
                }
            else
                {
                currentState = MainState.DONE;
                }
        case DRIVE_BACKWARDS_TO_FIRERANGE:
            if (false)
                currentState = MainState.DRIVE_INTO_RANGE_WITH_CAMERA;
            // TODO random number I selected
            if (Hardware.autoDrive.driveInches(6, .6))
                currentState = MainState.ALIGN_TO_FIRE;
            break;
        case TURN_TO_HOPPER:
            // TODO random magic numbers I selected
            if (Hardware.autoDrive.turnDegrees(isRedAlliance ? 12 : 90))
                {
                currentState = MainState.DRIVE_UP_TO_HOPPER;
                }
            break;
        case DRIVE_UP_TO_HOPPER:
            // TODO see above todo.
            if (Hardware.autoDrive.driveInches(isRedAlliance ? 12 : 90,
                    .6))
                {
                currentState = MainState.DONE;
                }
        case ALIGN_TO_FIRE:
            if (false)
                {
                // align By camera, probably in a firemech
                currentState = MainState.FIRE;
                }
            currentState = MainState.DONE;
            break;
        case FIRE:
            currentState = MainState.DONE;
            break;
        default:
            currentState = MainState.DONE;
        }
    return false;
}

private static boolean isDrivingByCamera = false;

private static boolean leftSidePath ()
{
    switch (currentState)
        {
        case INIT:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            Hardware.autoStateTimer.reset();
            Hardware.autoStateTimer.start();
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            if (Hardware.autoStateTimer.get() > delayTime)
                currentState = MainState.DRIVE_FORWARD_TO_SIDES_SLOW;
            break;
        case DRIVE_FORWARD_TO_SIDES_SLOW:
            currentState = MainState.DRIVE_FORWARD_TO_SIDES_MED;
            break;
        case DRIVE_FORWARD_TO_SIDES_MED:
            currentState = MainState.DRIVE_FORWARD_TO_SIDES;
            break;
        case DRIVE_FORWARD_TO_SIDES:
            currentState = MainState.TURN_TO_GEAR_PEG;
            break;
        case TURN_TO_GEAR_PEG:
            // turn left on both red and blue
            if (false)
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
            currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            break;
        case DRIVE_TO_GEAR_WITH_CAMERA:
            if (false)
                {
                currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
                }
            break;
        case DRIVE_CAREFULLY_TO_PEG:
            currentState = MainState.WIGGLE_WIGGLE;
            break;
        case WIGGLE_WIGGLE:
            currentState = MainState.WAIT_FOR_GEAR_EXODUS;
            break;
        case WAIT_FOR_GEAR_EXODUS:
            currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            currentState = MainState.TURN_TO_HOPPER;
            break;
        case TURN_TO_HOPPER:
            currentState = MainState.DRIVE_UP_TO_HOPPER;
            break;
        case DRIVE_UP_TO_HOPPER:
            // currentState = MainState.




        }
    return false;
}

/**
 * reset all the sensors and timers, in preparation for an autonomous program.
 */
private static void initializeDriveProgram ()
{
    Hardware.autoStateTimer.stop();
    Hardware.autoStateTimer.reset();
    Hardware.driveGyro.calibrate();
    Hardware.driveGyro.reset();
    Hardware.autoDrive.resetEncoders();
    Hardware.mecanumDrive.drive(0, 0, 0);
}

} // end class

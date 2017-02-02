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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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


// ==========================================
// TUNEABLES
// ==========================================

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
            delayTime = Hardware.delayPot.get() * (5 / 270);
            if (Hardware.driverStation.getAlliance() == Alliance.Red)
                isRedAlliance = true;
            autoPath = AutoProgram.RIGHT_PATH;
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
            break;
        }

} // end Periodic


private static MainState currentState = MainState.INIT;

private static AutoProgram autoPath = AutoProgram.INIT;

private static double delayTime = 5;

private static boolean isRedAlliance = false;

private static boolean goForFire = false;

private static boolean goForHopper = false;

private static boolean placeCenterGearPath ()
{
    System.out.println("CurrentState = " + currentState);
    switch (currentState)
        {
        case INIT:
            // zero out all the sensors, reset timers, etc.
            Hardware.autoStateTimer.start();
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
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
            // forward. Check to make sure we haven't gone too far. TODO
            // if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
            // {
            // currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
            // }
            // currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            // break;
        case DRIVE_TO_GEAR_WITH_CAMERA:
            // if (Hardware.imageProcessor.getNthSizeBlob(1) == null)
            // currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
            // Hardware.autoDrive.strafeToGear(.6, 25, .1, 271.8, 10);// TODO
            // magic
            // // numbers TODO
            // break;
        case DRIVE_CAREFULLY_TO_PEG:
            if ((Hardware.rightUS.getDistanceFromNearestBumper()
                    + Hardware.leftUS.getDistanceFromNearestBumper())
                    / 2 <= 6)// desired distance from wall when we start
                currentState = MainState.WIGGLE_WIGGLE;
            break;
        case WIGGLE_WIGGLE:
            // Find out how to wiggle wiggle.
            currentState = MainState.WAIT_FOR_GEAR_EXODUS;
            break;
        case WAIT_FOR_GEAR_EXODUS:
            if (Hardware.gearLimitSwitch.isOn() == false)
                {
                currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            if (Hardware.autoStateTimer.get() >= 1.5)
                {
                // Hardware.autoDrive.resetEncoders(); TODO
                // currentState = MainState.DRIVE_AWAY_FROM_PEG;
                }
            break;
        case DRIVE_AWAY_FROM_PEG:
            // if (Hardware.autoDrive.driveInches(-36.0, .5))
            // currentState = MainState.DONE; TODO
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
            Hardware.autoStateTimer.reset();
            Hardware.autoStateTimer.start();
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
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
            // if (Hardware.autoDrive.driveInches(6, .6)) TODO
            // currentState = MainState.ALIGN_TO_FIRE;
            break;
        case TURN_TO_HOPPER:
            // TODO random numbers I selected
            // if (Hardware.autoDrive.turnDegrees(isRedAlliance ? 12 : 90))
            // {
            // currentState = MainState.DRIVE_UP_TO_HOPPER;
            // } TODO
            break;
        case DRIVE_UP_TO_HOPPER:
            // TODO see above todo.
            // if (Hardware.autoDrive.driveInches(isRedAlliance ? 12 : 90,
            // .6))
            // {
            // currentState = MainState.DONE;
            // } TODO
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

private static boolean leftSidePath ()
{
    switch (currentState)
        {

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
    // Hardware.driveGyro.calibrate();
    // Hardware.driveGyro.reset(); TODO
    // Hardware.autoDrive.resetEncoders();
    Hardware.mecanumDrive.drive(0, 0, 0);
}

} // end class

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

    DRIVE_FORWARD_TO_SIDES,

    TURN_TO_GEAR_PEG,

    DRIVE_TO_GEAR_WITH_CAMERA,

    DRIVE_CAREFULLY_TO_PEG,

    WIGGLE_WIGGLE,

    WAIT_FOR_GEAR_EXODUS,

    DELAY_AFTER_GEAR_EXODUS,

    DRIVE_BACKWARDS_TO_FIRERANGE,

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

    DONE
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
            break;
        }

} // end Periodic

private static MainState currentState;

private static AutoProgram autoPath = AutoProgram.INIT;

private static boolean placeCenterGearPath ()
{
    switch (currentState)
        {
        case INIT:
            // zero out all the sensors, reset timers, etc.
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            // wait for timer to run out
            currentState = MainState.DRIVE_FORWARD_TO_CENTER_SLOW;
            break;
        case DRIVE_FORWARD_TO_CENTER_SLOW:
            // drive at our slow speed. wait a certain time. Could also check
            // distance for safety
            currentState = MainState.DRIVE_FORWARD_TO_CENTER_MED;
            break;
        case DRIVE_FORWARD_TO_CENTER_MED:
            // drive at our medium speed. wait a certain time to move on.
            currentState = MainState.DRIVE_FORWARD_TO_CENTER;
            break;
        case DRIVE_FORWARD_TO_CENTER:
            // If we see blobs, hand over control to camera, otherwise, go
            // forward. Check to make sure we haven't gone too far.
            if (false)
                {
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
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
    switch (currentState)
        {

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


} // end class

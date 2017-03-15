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
import org.usfirst.frc.team339.Utils.Shooter.turnReturn;
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
    /**
     * Reset Encoder Values, determine alliance, determine delay time,
     * determine path
     */
    INIT,
    /**
     * Delays anywhere between 0 and 5 seconds, controlled by a potentiometer on
     * the robot
     */
    DELAY_BEFORE_START,
    /**
     * Accelerates so we don't jerk our encoders.
     */
    ACCELERATE,
    /**
     * Drives up the the gear.
     */
    DRIVE_FORWARD_TO_CENTER,
    /**
     * Drive to the side of the airship at normal speed. See
     * DRIVE_FORWARD_TO_CENTER.
     */
    DRIVE_FORWARD_TO_SIDES,
    /**
     * Stops the robot after we've driven to the side of the airship, before we
     * turn to face the gear peg
     */
    BRAKE_BEFORE_TURN_TO_GEAR_PEG,
    /**
     * Turn towards the peg deposit place on the airship
     */
    TURN_TO_GEAR_PEG,
    /**
     * Stops our angular motion after we turn the the gear so we're at least
     * partially aligned to the gear
     */
    BRAKE_AFTER_TURN_TO_GEAR_PEG,
    /**
     * Align with the vision strips to deposit gear
     */
    DRIVE_TO_GEAR_WITH_CAMERA,

    /**
     * Drive up to the gear peg when we don't see any blobs.
     */
    DRIVE_CAREFULLY_TO_PEG,
    /**
     * Stops us moving up to the peg so we don't spear ourselves on it.
     */
    BRAKE_UP_TO_PEG,
    /**
     * 
     */
    TURN_TURRET_OUT_OF_THE_WAY,
    /**
     * Currently unused, may be used to regain vision targets, or make sure the
     * gear is on the spring. It's a bit of a wildcard at the moment.
     */
    WIGGLE_WIGGLE,
    /**
     * We've got the gear on the peg (probably), but we're waiting for the human
     * player to pull the gear out of our thingy.
     */
    WAIT_FOR_GEAR_EXODUS,
    /**
     * Wait after we detect the gear leaving our mechanism to make sure it's
     * clear before we try and drive back.
     */
    DELAY_AFTER_GEAR_EXODUS,
    /**
     * Back away from whatever peg we're currently on.
     */
    DRIVE_AWAY_FROM_PEG,
    /**
     * On paths where we go for the hopper, Turn so we're facing it
     */
    TURN_TO_HOPPER,
    /**
     * On paths where we go for the hopper, drive until we slam into it.
     */
    DRIVE_UP_TO_HOPPER,
    /**
     * Since we drive backwards to get into range, we need to turn around to
     * fire.
     */
    TURN_TO_FACE_GOAL,
    /**
     * On paths where we fire, back away from the gear peg and towards the
     * boiler until we're in range to fire.
     */
    DRIVE_TO_FIRERANGE,
    /**
     * Use the camera to figure out if we're in range of the top boiler
     */
    DRIVE_INTO_RANGE_WITH_CAMERA,
    /**
     * Turn the turret so it's facing the boiler
     */
    ALIGN_TO_FIRE,
    /**
     * Empty out the hopper or fire until time out
     */
    FIRE,
    /**
     * Quit the path.
     */
    DONE
    }

/**
 * The state machine controlling which auto path we are taking
 * 
 * @author Alex Kneipp
 */
private static enum AutoProgram
    {
    /**
     * The state we start in, runs once.
     */
    INIT,

    /**
     * The path where we start in the middle and (try to) place the gear on the
     * center peg.
     */
    CENTER_GEAR_PLACEMENT,

    /**
     * The path where we start on the right side of the field and drive
     * forwards, turn and place the gear, and try and turn around to attempt to
     * fire. (Based on whether or not we are on the red or blue alliance)
     */
    SIDE_GEAR_PATH,

    /**
     * The path where we start on the left side of the field and drive
     * forwards, turn and place the gear, and try and turn around to attempt to
     * fire. (Based on whether or not we are on the red or blue alliance)
     */
    BASELINE_PATH,

    /**
     * We are done with this auto path! Yay! (or aww depending on if it
     * worked...)
     */
    DONE
    }



// ==================================
// VARIABLES
// ==================================
/**
 * Temporarily holds each state for the return type for Strafing to Gear
 * 
 * Found in DRIVE_TO_GEAR_WITH_CAMERA states.
 */
private static Drive.AlignReturnType cameraState = Drive.AlignReturnType.NO_BLOBS;

// ==========================================
// TUNEABLES
// ==========================================

/**
 * If we are using mecanum, this is the number of DEGREES the robot will offset
 * while using StrafeToGear.
 * 
 * If we are using Tank drive (for some strange reason) this should be changed
 * to a percentage that will be offset for each side of the robot.
 */
private static final double ALIGN_CORRECT_VAR = 30;// 45

/**
 * How fast we will be driving during all of auto, in percent.
 */
private static final double DRIVE_SPEED = .6;// .45

/**
 * Determines what value we set the motors backwards to in order to brake, in
 * percentage.
 */
private static final double BRAKE_SPEED = .18;

/**
 * Determines how long we should set the motors backwards in order to brake
 * using timeBrake in the Drive class, in percentage.
 */
private static final double BRAKE_TIME = .2;

/**
 * The deadband for considering whether or not we are aligned to the target(s).
 * Change the pixel value, and the /getHorizontalResolution will change it to
 * relative coordinates.
 */
private static final double ALIGN_DEADBAND = 7 // +/- pixels
        / Hardware.axisCamera.getHorizontalResolution();

/**
 * Where we say the "center" of the camera is / where we want to correct TO.
 */
private static final double ALIGN_ACCEPTED_CENTER = .5; // Relative coordinates;
                                                        // half the screen = .5

/**
 * How far away we are in inches when we stop moving towards the goal
 * 
 * Used in DRIVE_CARFULLY_TO_GEAR as well as StrafeToGear
 */
private static final int STOP_DISTANCE_TO_GEAR = 14;

/**
 * How fast we should accelerate in the accelerate state, in seconds.
 */
private static final double TIME_TO_ACCELERATE = .4;

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
    // reset encoders
    Hardware.leftFrontEncoder.reset();
    Hardware.leftRearEncoder.reset();
    Hardware.rightFrontEncoder.reset();
    Hardware.rightRearEncoder.reset();
    // motors
    Hardware.leftRearMotor.setInverted(true);
    Hardware.intakeMotor.setInverted(true);
    // mecanum drive
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    Hardware.mecanumDrive.setFirstGearPercentage(1.0);
    Hardware.mecanumDrive.setGear(1);
    Hardware.mecanumDrive
            .setFirstGearPercentage(
                    Robot.KILROY_XVIII_FIRST_GEAR_PERCENTAGE);
    // Sets the scaling factor and general ultrasonic stuff
    // Hardware.ultraSonic.setScalingFactor(.13);
    // Hardware.ultraSonic.setOffsetDistanceFromNearestBummper(3);
    // Hardware.ultraSonic.setNumberOfItemsToCheckBackwardForValidity(3);

    Hardware.cameraservoX.setAngle(190);
    // if running on kilroy XVIII use certain value and different for XVII
    // TODO WHY was this gone in git?
    if (Hardware.isRunningOnKilroyXVIII)
        {
        robotSpeedScalar = KILROY_XVIII_DEFAULT_SPEED;
        }
    else
        {
        robotSpeedScalar = KILROY_XVII_DEFAULT_SPEED;
        }
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
    // The main state machine for figuring out which path we are following
    System.out.println("AutoPath: " + autoPath);
    switch (autoPath)
        {

        case INIT:
            // get the auto program we want to run, get delay pot.
            if (Hardware.enableAutonomous.isOn())
                {
                delayBeforeAuto = Hardware.delayPot.get(0.0, 5.0);
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
                if (Hardware.sideGearPath.isOn())
                    {
                    autoPath = AutoProgram.SIDE_GEAR_PATH;
                    break;
                    }
                if (Hardware.autoBaseLinePath.isOn())
                    {
                    autoPath = AutoProgram.BASELINE_PATH;
                    break;
                    }
                autoPath = AutoProgram.DONE;
                }
            else
                {
                autoPath = AutoProgram.DONE;
                }
            break;
        // Drives towards the center gear peg
        case CENTER_GEAR_PLACEMENT:
            if (placeCenterGearPath() == true)
                autoPath = AutoProgram.DONE;
            break;
        // Drives towards the right gear peg and turns around to fire
        case SIDE_GEAR_PATH:
            if (baselinePath() == true)
                autoPath = AutoProgram.DONE;
            break;
        // Drives towards the left gear peg and turns around to fire
        case BASELINE_PATH:
            if (sideGearPath() == true)
                autoPath = AutoProgram.DONE;
            break;
        // We are done with the auto program!
        case DONE:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            Hardware.ringlightRelay.set(Value.kOff);
            break;
        // If we made a horrible mistake in the state machine, default to DONE
        // (Stop the program)
        default:
            autoPath = AutoProgram.DONE;
            break;
        }

} // end Periodic

/**
 * Which state we are in in the main state machine. Starts at init.
 */
private static MainState currentState = MainState.INIT;

/**
 * A control enum for choosing which accelerate we are doing (only for left and
 * right paths)
 */
private static MainState postAccelerateState = MainState.DONE;

/**
 * Determines which position we are on the field to control
 * which auto we use
 */
private static AutoProgram autoPath = AutoProgram.INIT;

/**
 * Amount of time we delay before starting the auto program
 */
private static double delayBeforeAuto = 5;

/**
 * Whether or not we are the red alliance. If not, we are the Blue alliance.
 */
private static boolean isRedAlliance = false;

/**
 * Determines whether or not we choose to go fire after placing the gear.
 * Will not work if we choose to go to the hopper in goForHopper.
 */
private static boolean goForFire = false;

/**
 * Determines whether or not we choose to go to the hopper after placing the
 * gear
 */
private static boolean goForHopper = false;

/**
 * The auto path where we start in the center position, and try and strafe
 * towards the camera if we can. If we can't we just use the ultrasonic to
 * "carefully drive to peg", and back up (if the switch is flicked to true)
 * 
 * @return
 */
private static boolean placeCenterGearPath ()
{
    System.out.println("CurrentState = " + currentState);
    System.out.println("Right US: "
            + Hardware.ultraSonic.getDistanceFromNearestBumper());
    System.out.println(Hardware.autoDrive.getAveragedEncoderValues());
    System.out.println(
            "Gear " + Hardware.mecanumDrive.getCurrentGearPercentage());
    switch (currentState)
        {
        case INIT:
            // zero out all the sensors, reset timers, etc.
            initializeDriveProgram();
            Hardware.autoStateTimer.start();
            Hardware.ringlightRelay.set(Value.kOn);
            if (Hardware.backupOrFireOrHopper.isOn())
                {
                goForFire = true;
                }
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            // stop all the motors to feed the watchdog
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // wait for timer to run out
            if (Hardware.autoStateTimer.get() >= delayBeforeAuto)
                {
                // Hardware.axisCamera.saveImagesSafely();
                currentState = MainState.ACCELERATE;
                postAccelerateState = MainState.DRIVE_FORWARD_TO_CENTER;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case ACCELERATE:
            // Accelerate towards the center gear peg, to save the encoders
            cameraState = AlignReturnType.WAITING;
            if (Hardware.autoDrive.accelerate(
                    getRealSpeed(DRIVE_SPEED),
                    TIME_TO_ACCELERATE) == true)
                {
                // Not using this in this state machine, only for left/right
                // side
                currentState = postAccelerateState;
                Hardware.axisCamera.saveImagesSafely();
                }
            // Purge the ultrasonic of it's current values while we are
            // accelerating
            Hardware.ultraSonic.getDistanceFromNearestBumper();
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
                    getRealSpeed(DRIVE_SPEED),
                    ALIGN_CORRECT_VAR,
                    ALIGN_DEADBAND, ALIGN_ACCEPTED_CENTER,
                    STOP_DISTANCE_TO_GEAR, 1, .9);

            System.out.println("strafeToGear state: " + cameraState);


            if (cameraState == AlignReturnType.NO_BLOBS)
                {
                // If we don't see anything, just drive forwards till we are
                // close enough
                currentState = MainState.DRIVE_CAREFULLY_TO_PEG;
                }
            if (cameraState == AlignReturnType.CLOSE_ENOUGH)
                {
                // If we are close enough to the wall, stop.
                currentState = MainState.BRAKE_UP_TO_PEG;
                }
            break;
        case DRIVE_CAREFULLY_TO_PEG:
            // Drives straight to the wall using the encoder, and picks back up
            // with drive with camera if we can see blobs again
            if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
                {
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
                }
            else
                {
                if (Hardware.ultraSonic
                        .getDistanceFromNearestBumper() >= STOP_DISTANCE_TO_GEAR)
                    {
                    Hardware.autoDrive.driveNoDeadband(DRIVE_SPEED, 0.0,
                            0);
                    }
                else
                    {
                    currentState = MainState.BRAKE_UP_TO_PEG;
                    }
                }
            break;
        case BRAKE_UP_TO_PEG:
            // Brakes using the Time Brake function in the Drive class.
            if (Hardware.autoDrive.timeBrake(BRAKE_SPEED, BRAKE_TIME))
                {
                currentState = MainState.TURN_TURRET_OUT_OF_THE_WAY;
                }
            break;
        case TURN_TURRET_OUT_OF_THE_WAY:
            // Turns the turret out of the way so that when the human player
            // pulls up the gear, it does not hit the camera
            if (Hardware.shooter
                    .turnToBearing(
                            Hardware.shooter.MIN_GIMBALING_ANGLE) == turnReturn.SUCCESS)
                {
                Hardware.shooter.stopGimbal();
                currentState = MainState.WAIT_FOR_GEAR_EXODUS;
                }// TODO THIS is where I left off commenting.
            break;
        case WAIT_FOR_GEAR_EXODUS:
            Hardware.ringlightRelay.set(Value.kOff);
            // if (Hardware.autoStateTimer.get() >= 1.25)
            // {
            // runWiggleWiggleSetup = true;
            // currentState = MainState.WIGGLE_WIGGLE;
            // }
            if (Hardware.gearLimitSwitch.isOn() == false)
                {
                Hardware.autoDrive.driveNoDeadband(0.0, 0.0, 0.0);
                currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case WIGGLE_WIGGLE:
            if (runWiggleWiggleSetup)
                {
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                runWiggleWiggleSetup = false;
                wiggleWiggleCount++;
                }
            if (wiggleWiggleCount == 1)
                {
                if (Hardware.autoStateTimer.get() >= .5)
                    {
                    Hardware.autoDrive.drive(.5, 90);
                    }
                else
                    {
                    currentState = MainState.WAIT_FOR_GEAR_EXODUS;
                    }
                }
            else if (wiggleWiggleCount == 2)
                {
                if (Hardware.autoStateTimer.get() >= .1)
                    {
                    Hardware.autoDrive.drive(.5, -90);
                    }
                else
                    {
                    currentState = MainState.WAIT_FOR_GEAR_EXODUS;
                    }
                }
            else
                {
                currentState = MainState.DELAY_AFTER_GEAR_EXODUS;
                }
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            if (Hardware.autoStateTimer.get() >= 1.5)
                {
                Hardware.axisCamera.saveImagesSafely();
                Hardware.autoDrive.resetEncoders();
                if (goForFire)
                    {
                    currentState = MainState.DRIVE_AWAY_FROM_PEG;
                    }
                else
                    {
                    currentState = MainState.DONE;
                    }
                }
            break;
        case DRIVE_AWAY_FROM_PEG:
            if (Hardware.autoDrive.driveInches(36.0, getRealSpeed(-.5)))
                {
                currentState = MainState.DONE;
                }

            break;
        default:
        case DONE:
            Hardware.axisCamera.saveImagesSafely();
            return true;
        }
    return false;
}

private static int wiggleWiggleCount = 0;

private static boolean runWiggleWiggleSetup = true;

private static boolean baselinePath ()
{
    System.out.println("Current State = " + currentState);
    System.out.println("Encoder Val: "
            + Hardware.autoDrive.getAveragedEncoderValues());
    // System.out.println("Delayval: " + delayBeforeAuto);
    // System.out.println("Timer val: " + Hardware.autoStateTimer.get());
    switch (currentState)
        {
        case INIT:
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            Hardware.autoStateTimer.reset();
            Hardware.autoStateTimer.start();
            initializeDriveProgram();
            Hardware.autoStateTimer.start();
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            // stop all the motors to feed the watchdog
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // wait for timer to run out
            if (Hardware.autoStateTimer.get() >= delayBeforeAuto)
                {
                Hardware.axisCamera.saveImagesSafely();
                currentState = MainState.ACCELERATE;
                postAccelerateState = MainState.DRIVE_FORWARD_TO_CENTER;
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                }
            break;
        case ACCELERATE:
            if (Hardware.autoDrive.accelerate(DRIVE_SPEED, .5) == true)
                {
                currentState = postAccelerateState;
                }
            break;
        case DRIVE_FORWARD_TO_CENTER:
            // baseline is 94 inches from wall, so drive a little bit further
            if (Hardware.autoDrive.driveInches(115,
                    DRIVE_SPEED) == true)
                {
                currentState = MainState.BRAKE_UP_TO_PEG;
                }
            break;
        case BRAKE_UP_TO_PEG:
            if (Hardware.autoDrive.brakeToZero(.3))
                {
                currentState = MainState.DONE;
                }
            break;
        default:
        case DONE:
            Hardware.autoDrive.drive(0, 0, 0);
            Hardware.ringlightRelay.set(Value.kOff);
            return true;
        }
    return false;
}

private static int fireCount = 0;

private static boolean sideGearPath ()
{
    System.out.println("Current State = " + currentState);
    switch (currentState)
        {
        case INIT:
            // Stop all the motors, for safety
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // Resets the encoders, gyro, motors, and timer.
            // leaves the timer stopped.
            initializeDriveProgram();
            // Start the timer again
            Hardware.autoStateTimer.start();
            // Turn on the ringlight for our eventual vision tracking
            Hardware.ringlightRelay.set(Value.kOn);
            currentState = MainState.DELAY_BEFORE_START;
            break;
        case DELAY_BEFORE_START:
            /*
             * stop all the motors to feed the watchdog, and for safety and
             * stuff.
             */
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // wait for timer to run out
            if (Hardware.autoStateTimer.get() >= delayBeforeAuto)
                {
                // Start accelerating towards the left side of the goal.
                currentState = MainState.ACCELERATE;
                // Tell the accelerate state that we want to drive to the sides
                // after it's done.
                postAccelerateState = MainState.DRIVE_FORWARD_TO_SIDES;
                }
            break;
        case ACCELERATE:
            // accelerate to our target drive speed over .4 seconds
            if (Hardware.autoDrive.accelerate(DRIVE_SPEED,
                    TIME_TO_ACCELERATE))
                {
                // go to the state the state that I came from told me to
                // once I was done.
                currentState = postAccelerateState;
                }
            break;
        case DRIVE_FORWARD_TO_SIDES:
            // System.out.println("Encoders: "
            // + Hardware.autoDrive.getAveragedEncoderValues());
            if (isRedAlliance == false)
                {
                // According to Cole's numbers, we drive forward 77.9 inches as
                // the first step in our auto program.
                if (Hardware.autoDrive.driveStraightInches(77.9,
                        DRIVE_SPEED, .2) == false)
                    {
                    // keep going
                    currentState = MainState.BRAKE_BEFORE_TURN_TO_GEAR_PEG;
                    }
                }
            else
                {
                // if we're red we have to drive backwards.
                if (Hardware.autoDrive.driveStraightInches(77.9,
                        -DRIVE_SPEED, .2) == false)
                    {
                    currentState = MainState.BRAKE_BEFORE_TURN_TO_GEAR_PEG;
                    }
                }
            break;
        case BRAKE_BEFORE_TURN_TO_GEAR_PEG:
            // TODO check to make sure this works
            // If we're done stopping
            if (Hardware.autoDrive.brakeToZero(BRAKE_SPEED) == true)
                {
                // move on to the turn
                currentState = MainState.TURN_TO_GEAR_PEG;
                }
            break;
        case TURN_TO_GEAR_PEG:
            // If we're done turning.// turn right on both red and blue
            if (isRedAlliance == true)
                {
                if (Hardware.autoDrive.turnDegrees(-30, .4) == true)
                    {
                    currentState = MainState.BRAKE_AFTER_TURN_TO_GEAR_PEG;
                    }
                }
            else if (isRedAlliance == false)
                {
                if (Hardware.autoDrive.turnDegrees(30, .4))
                    {
                    currentState = MainState.BRAKE_AFTER_TURN_TO_GEAR_PEG;
                    }
                }
            break;
        case BRAKE_AFTER_TURN_TO_GEAR_PEG:
            // If we're done stopping
            if (Hardware.autoDrive.brakeToZero(BRAKE_SPEED) == true)
                {
                // Take a picture and filter the bottom 10% of it out to drop
                // the gear blobs
                Hardware.imageProcessor.processImage();
                // TODO may be able to remove this in the future, given
                // our new camera position
                Hardware.imageProcessor.filterBlobsInYRange(1, .9);
                currentState = MainState.DRIVE_TO_GEAR_WITH_CAMERA;
                }
            break;
        case DRIVE_TO_GEAR_WITH_CAMERA:
            // Make sure we have the most recent blob info
            Hardware.imageProcessor.processImage();
            // Eliminate the gear from the blob count
            // TODO may be able to remove this in the future, given
            // our new camera position
            Hardware.imageProcessor.filterBlobsInYRange(1, .9);
            // If at any time we lose our target blob number
            if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
                {
                // TODO magic numbers and need to be tuned.
                // If we're close enough to the gear wall to quit...
                // TODO find a new relative center
                if (Hardware.autoDrive.driveToGear(DRIVE_SPEED,
                        DRIVE_SPEED, 0, .05,
                        10) == AlignReturnType.CLOSE_ENOUGH)
                    {
                    // Stop and wait for Mr. Human player to pull out our gear.
                    Hardware.autoDrive.drive(0.0, 0.0, 0.0);
                    currentState = MainState.WAIT_FOR_GEAR_EXODUS;
                    }
                }
            break;
        case WAIT_FOR_GEAR_EXODUS:
            // If the gear limit switch is of (e.g. we don't have the gear
            // anymore)
            if (Hardware.gearLimitSwitch.isOn() == false)
                {
                // reset the time for the delay after this and start delaying
                Hardware.autoStateTimer.reset();
                Hardware.autoStateTimer.start();
                currentState = MainState.DONE;
                }
            break;
        case DELAY_AFTER_GEAR_EXODUS:
            // Stop the motors cause we're not moving and we want to avoid
            // !!FUN!!
            Hardware.leftRearMotor.set(0);
            Hardware.leftFrontMotor.set(0);
            Hardware.rightRearMotor.set(0);
            Hardware.rightFrontMotor.set(0);
            // If we've been waiting for more than 1.5 seconds after the human
            // player releases the switch
            if (Hardware.autoStateTimer.get() >= 1.5)// TODO magic number
                {
                // If the switches tell us to backup..
                if (Hardware.backupOrFireOrHopper.isOn() == true)
                    {
                    // backup
                    currentState = MainState.DRIVE_AWAY_FROM_PEG;
                    }
                else// The switches tell us to stay put
                    {
                    // We're done with Auto!
                    currentState = MainState.DONE;
                    }
                }
            break;
        default:
            currentState = MainState.DONE;
        case DONE:
            return true;
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

/**
 * Corrects the speed based on which robot we're using. Uses the
 * isRunningOnKilroyXVIII boolean to decide which speed scalar to use.
 * 
 * @param fakeSpeed
 *            The speed you want the robot to use, don't worry about it.
 * @return
 *         The correct speed for whichever robot we're on.
 */
private static double getRealSpeed (double fakeSpeed)
{
    return fakeSpeed * robotSpeedScalar;
}

private static double robotSpeedScalar = 1.0;

private static final double KILROY_XVIII_DEFAULT_SPEED = 1.0;

private static final double KILROY_XVII_DEFAULT_SPEED = .7;



} // end class

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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
int control = 1;

public static void init ()
{
    // --------------------------------------
    // initialize all encoders here
    // --------------------------------------
    Hardware.leftFrontEncoder.reset();
    Hardware.rightFrontEncoder.reset();
    Hardware.rightRearEncoder.reset();
    Hardware.leftRearEncoder.reset();
    // --------------------------------------
    // initialize all motors here
    // --------------------------------------
    Hardware.leftRearMotor.set(0.0);
    Hardware.rightRearMotor.set(0.0);
    Hardware.rightFrontMotor.set(0.0);
    Hardware.leftFrontMotor.set(0.0);

    // ---------------------------------------
    // Servo init
    // ---------------------------------------
    Hardware.cameraServo.setAngle(LOWER_CAMERASERVO_POSITION);


    // Hardware.rightFrontMotor.setInverted(true); // TODO takeout
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    // Hardware.leftRearMotor.setInverted(true);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // Hardware.tankDrive.setGear(1);
    // Hardware.leftUS.setScalingFactor(.13);
    // Hardware.leftUS.setOffsetDistanceFromNearestBummper(0);
    // Sets the scaling factor and general ultrasonic stuff
    Hardware.rightUS.setScalingFactor(.13);
    Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
    Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);
    // Hardware.leftUS.setNumberOfItemsToCheckBackwardForValidity(1);
    // Hardware.LeftUS.setConfidenceCalculationsOn(false);
    // Hardware.RightUS.setConfidenceCalculationsOn(false);

    // Hardware.tankDrive.setRightMotorDirection(MotorDirection.REVERSED);
    // boolean testchoosers = true;

    // SendableChooser sendablechoosetest;
    // sendablechoosetest = new SendableChooser();
    // sendablechoosetest.addDefault("default", testchoosers);

    // Sendable testsendable = ;
    // SmartDashboard.putData("teleoptest", testsendable);

    Hardware.tankDrive.setGear(1);

    isAligning = false;
    isStrafingToTarget = false;
    Hardware.autoDrive.setDriveCorrection(.3);
    Hardware.autoDrive.setEncoderSlack(1); // TODO
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
    Command chooseTrueorFalse;
    SendableChooser testbool;
    testbool = new SendableChooser();
    testbool.addDefault("true", Hardware.changeBool.equals(true));
    testbool.addObject(" false", Hardware.changeBool.equals(false));
    int control = 1;

    SmartDashboard.putData("testbool", testbool);
    switch (control)
        {
        case 1:

            // System.out.println(Hardware.changeBool.getClass());
            control = 1;
            break;
        case 2:

            break;

        }

    if (Hardware.leftDriver.getTrigger() && !previousFireButton)
        {
        firing = !firing;
        }
    if (firing)
        Hardware.shooter.fire();


    // previousFireButton = Hardware.leftDriver.getTrigger();
    //
    // if (fireCount > 0)
    // {
    // if (Hardware.shooter.fire())
    // {
    // fireCount--;
    // }
    // }
    if (preparingToFire == false)
        Hardware.shooter.stopFlywheelMotor();
    /*
     * System.out.println("Firecount: " + fireCount);
     * 
     * System.out.println(
     * "Flywheel speed: " + Hardware.shooterMotor.getSpeed());
     */


    // TODO Figure out why the ring light is flickering
    if (Hardware.ringlightSwitch.isOnCheckNow())
        {
        Hardware.ringlightRelay.set(Relay.Value.kOn);
        }
    else
        {
        Hardware.ringlightRelay.set(Relay.Value.kOff);
        }
    // Hardware.gearServo.setAngle(200);
    // Hardware.gearServo.getAngle();
    // Print out any data we want from the hardware elements.
    printStatements();

    // TESTING CODE:

    if (Hardware.rightOperator.getRawButton(2))
        Hardware.intake.startIntake();
    else if (Hardware.rightOperator.getRawButton(3))
        Hardware.intake.reverseIntake();
    else
        Hardware.intake.stopIntake();

    // =================================================================
    // Driving code
    // =================================================================

    if (Hardware.leftDriver.getTrigger())
        {
        rotationValue = Hardware.leftDriver.getTwist();
        System.out.println("Twist: " + Hardware.leftDriver.getTwist());
        }
    else
        rotationValue = 0.0;

    if (!isAligning && !isStrafingToTarget)// TODO remove isAccing
                                           // stuff
        {
        if (Hardware.isUsingMecanum == true)
            Hardware.mecanumDrive.drive(
                    Hardware.leftDriver.getMagnitude(),
                    Hardware.leftDriver.getDirectionDegrees(),
                    rotationValue);
        else
            Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
                    Hardware.leftDriver.getY());
        }



    // =================================================================
    // CAMERA CODE
    // =================================================================
    // "Cancel basically everything" button
    // if (Hardware.leftOperator.getRawButton(7)
    // || Hardware.leftOperator.getRawButton(6))
    // {
    // System.out.println("Cancelling everything");
    // isAligning = false;
    // isStrafingToTarget = false;
    // }
    //
    // // Testing aligning to target
    // if (Hardware.rightOperator.getRawButton(4))
    // isAligning = true;
    //
    // if (isAligning)
    // {
    // alignValue = Hardware.autoDrive.alignToGear(CAMERA_ALIGN_CENTER,
    // movementSpeed, CAMERA_ALIGN_DEADBAND);
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
    // alignValue = Hardware.autoDrive.strafeToGear(movementSpeed, .2,
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

    // Testing good speed values
    // if (Hardware.leftOperator.getRawButton(4) && !hasPressedFive)
    // {
    // // adds .05 to movement speed then prints movementSpeed
    // movementSpeed += .05;
    // System.out.println(movementSpeed);
    // }
    // hasPressedFour = Hardware.leftOperator.getRawButton(4);

    // if (Hardware.leftOperator.getRawButton(5) && !hasPressedFour)
    // {
    // // subtracts .05 from movement speed then prints movementSpeed
    // movementSpeed -= .05;
    // System.out.println(movementSpeed);
    // }
    // hasPressedFive = Hardware.leftOperator.getRawButton(5);


    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8)
                    || Hardware.rightOperator.getRawButton(8));

    // Written by Ashley Espeland, has not been tested
    // cameraServo code setting to either the higher or the lower angle position
    if (Hardware.rightDriver.getRawButton(4)
            && fourHasBeenPressed == false)
        {
        if (Hardware.cameraServo
                .getAngle() == LOWER_CAMERASERVO_POSITION)
            {
            Hardware.cameraServo.setAngle(HIGHER_CAMERASERVO_POSITION);
            }
        else if (Hardware.cameraServo
                .getAngle() == HIGHER_CAMERASERVO_POSITION)
            {
            Hardware.cameraServo.setAngle(LOWER_CAMERASERVO_POSITION);
            }
        fourHasBeenPressed = true;
        }

} // end
  // Periodic

// private static boolean isSpeedTesting = false;


private static double rotationValue = 0.0;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static boolean isAligning = false;

private static boolean isStrafingToTarget = false;

private static double movementSpeed = 0.3;

private static boolean hasPressedFour = false;

private static boolean hasPressedFive = false;

private static boolean previousFireButton = false;

private static boolean preparingToFire = false;

private static boolean firing = false;



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
 *          Also Edited by Josef Liebl
 * 
 */
public static void printStatements ()
{
    // =================================
    // Motor controllers
    // prints value of the motors
    // =================================
    // System.out.println("Right Front Motor Controller: "
    // + Hardware.rightFrontMotor.get());
    // System.out.println("Left Front Motor Controller: " +
    // Hardware.leftFrontMotor.get());
    // System.out.println("Right Rear Motor Controller: " +
    // Hardware.rightRearMotor.get());
    // System.out.println("Left Rear Motor Controller: " +
    // Hardware.leftRearMotor.get());

    // System.out
    // .println("Flywheel Motor: " + Hardware.shooterMotor.get());
    //
    // System.out.println("Intake Motor: " + Hardware.intakeMotor.get());

    // =================================
    // CAN items
    // prints value of the CAN controllers
    // =================================
    // Hardware.CAN.printAllPDPChannels();

    // =================================
    // Relay
    // prints value of the relay states
    // =================================
    // if (Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // System.out.println("Ring light relay is on");
    // }
    // else if (!Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // System.out.println("Ring light relay is off");
    // }

    // =================================
    // Digital Inputs
    // =================================
    // ---------------------------------
    // Switches
    // prints state of switches
    // ---------------------------------
    // System.out.println("Gear Limit Switch: "
    // + Hardware.gearLimitSwitch.isOn());

    // System.out.println("Backup or fire: " + Hardware.backupOrFire.isOn());
    // System.out.println("Enable Auto: " + Hardware.enableAutonomous.isOn());

    // System.out.println(
    // "Path Selector: " + Hardware.pathSelector.getPosition());


    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------
    System.out.println("Right Front Encoder: "
            + Hardware.rightFrontEncoder.get());
    System.out.println("Right Front Encoder Distance: "
            + Hardware.autoDrive.getRightRearEncoderDistance());
    System.out.println("Right Rear Encoder: "
            + Hardware.rightRearEncoder.get());
    System.out.println("Right Rear Encoder Distance: "
            + Hardware.autoDrive.getRightRearEncoderDistance());
    System.out.println("Left Front Encoder: "
            + Hardware.leftFrontEncoder.get());
    System.out.println("Left Front Encoder Distance: "
            + Hardware.autoDrive.getLeftFrontEncoderDistance());
    System.out.println("Left Rear Encoder: "
            + Hardware.leftRearEncoder.get());
    System.out.println("Left Rear Encoder Distance: "
            + Hardware.autoDrive.getLeftFrontEncoderDistance());

    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------
    // if (Hardware.ballLoaderSensor.isOn() == true)
    // {
    // System.out.println("Ball IR Sensor is On");
    // }
    // else if (Hardware.ballLoaderSensor.isOn() == false)
    // {
    // System.out.println("Ball IR Sensor is Off");
    // }

    // =================================
    // Pneumatics
    // =================================
    // ---------------------------------
    // Compressor
    // prints information on the compressor
    // ---------------------------------
    // There isn't one

    // ---------------------------------
    // Solenoids
    // prints the state of solenoids
    // ---------------------------------
    // There are none

    // =================================
    // Analogs
    // =================================
    //
    // We don't want the print statements to flood everything and go ahhhhhhhh
    //
    // if (Hardware.rightOperator.getRawButton(11))
    // System.out.println("LeftUS = "
    // + Hardware.leftUS.getDistanceFromNearestBumper());
    // System.out.println("RightUS = "
    // + Hardware.rightUS.getDistanceFromNearestBumper());
    // System.out.println("Delay Pot: " + Hardware.delayPot.get());


    // ---------------------------------
    // pots
    // where the pot is turned to
    // ---------------------------------
    // System.out.println("Delay Pot Degrees" + Hardware.delayPot.get());


    // =================================
    // Connection Items
    // =================================
    // ---------------------------------
    // Cameras
    // prints any camera information required
    // ---------------------------------

    // System.out.println("Expected center: " + CAMERA_ALIGN_CENTER);
    //
    // Hardware.imageProcessor.processImage();
    // System.out.println("Number of blobs: " + Hardware.imageProcessor
    // .getParticleAnalysisReports().length);
    // if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
    // System.out
    // .println("Actual center: " + ((Hardware.imageProcessor
    // .getNthSizeBlob(0).center_mass_x
    // + Hardware.imageProcessor
    // .getNthSizeBlob(1).center_mass_x)
    // / 2.0)
    // / Hardware.axisCamera
    // .getHorizontalResolution());
    //
    // System.out.println("Deadband: " + CAMERA_ALIGN_DEADBAND);

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

    // System.out.println("Left Joystick: " + Hardware.leftDriver.getY());
    // System.out.println("Right Joystick: " + Hardware.rightDriver.getY());
    // System.out.println("Left Operator: " + Hardware.leftOperator.getY());
    // System.out.println("Right Operator: " + Hardware.rightOperator.getY());

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
private final static double CAMERA_ALIGN_DEADBAND = 10.0 // +/- Pixels
        / Hardware.axisCamera.getHorizontalResolution();

private final static double CAMERA_ALIGN_CENTER = .478;  // Relative coordinates


// ==========================================
// TUNEABLES
// ==========================================
private final static double LOWER_CAMERASERVO_POSITION = 65;// TODO find actual
                                                            // value

private final static double HIGHER_CAMERASERVO_POSITION = 90;// TODO find actual
                                                             // value

public static boolean fourHasBeenPressed = true;

} // end class

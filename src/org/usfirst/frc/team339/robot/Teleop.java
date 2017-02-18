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
import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Shooter;
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

    Hardware.gimbalMotor
            .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    Hardware.gimbalMotor.setEncPosition(0);
    if (Hardware.isRunningOnKilroyXVIII == true)
        {
        Hardware.rightFrontEncoder.setReverseDirection(true);
        Hardware.rightRearEncoder.setReverseDirection(false);
        Hardware.leftFrontEncoder.setReverseDirection(true);
        Hardware.leftRearEncoder.setReverseDirection(false);
        Hardware.rightRearEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.leftRearEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.rightFrontEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.leftFrontEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.tankDrive.setGearPercentage(1, Robot.FIRST_GEAR);
        Hardware.tankDrive.setGearPercentage(2, Robot.SECOND_GEAR);
        Hardware.mecanumDrive.setFirstGearPercentage(Robot.FIRST_GEAR);
        Hardware.rightFrontMotor.setInverted(false);
        Hardware.rightRearMotor.setInverted(false);
        Hardware.leftFrontMotor.setInverted(false);
        Hardware.leftRearMotor.setInverted(true);
        Hardware.intakeMotor.setInverted(true);
        Hardware.mecanumDrive
                .setDeadbandPercentageZone(Hardware.joystickDeadzone);
        Hardware.mecanumDrive.setMecanumJoystickReversed(false);
        // Hardware.rightFrontMotorSafety.setExpiration(.5);
        // Hardware.rightRearMotorSafety.setExpiration(.5);
        // Hardware.leftFrontMotorSafety.setExpiration(.5);
        // Hardware.leftRearMotorSafety.setExpiration(1.0);
        Hardware.rightUS.setScalingFactor(.13);
        Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
        Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);
        Hardware.tankDrive.setGear(1);
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);
        // Hardware.mecanumDrive.setDirectionalDeadzone(0.2);
        // Hardware.tankDrive.setRightMotorDirection(MotorDirection.REVERSED);
        }
    else
        {
        // Hardware.rightFrontEncoder.setReverseDirection(true);
        // Hardware.rightRearEncoder.setReverseDirection(false);
        // Hardware.leftFrontEncoder.setReverseDirection(true);
        // Hardware.leftRearEncoder.setReverseDirection(false);
        Hardware.rightRearEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.leftRearEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.leftFrontEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.rightFrontEncoder.setDistancePerPulse(
                Robot.ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        }
    // Hardware.tankDrive.setGearPercentage(1, FIRST_GEAR);
    // Hardware.tankDrive.setGearPercentage(2, SECOND_GEAR);
    // Hardware.mecanumDrive.setFirstGearPercentage(FIRST_GEAR);
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

    Hardware.gimbalMotor
            .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);

    isAligning = false;
    isStrafingToTarget = false;
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
    // Command chooseTrueorFalse;
    // SendableChooser testbool;
    // testbool = new SendableChooser();
    // testbool.addDefault("true", Hardware.changeBool.equals(true));
    // testbool.addObject(" false", Hardware.changeBool.equals(false));
    // int control = 1;
    //
    // SmartDashboard.putData("testbool", testbool);
    // switch (control)
    // {
    // case 1:
    //
    // // System.out.println(Hardware.changeBool.getClass());
    // control = 1;
    // break;
    // case 2:
    //
    // break;
    //
    // }

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

    if (Hardware.rightOperator.getRawButton(2))
        Hardware.intake.startIntake();
    else if (Hardware.rightOperator.getRawButton(3))
        Hardware.intake.reverseIntake();
    else
        Hardware.intake.stopIntake();

    // =================================================================
    // Driving code
    // =================================================================

    // rotate only when we are pulling the trigger
    if (Hardware.leftDriver.getTrigger())
        {
        rotationValue = Hardware.leftDriver.getTwist();
        System.out.println("Twist: " + Hardware.leftDriver.getTwist());
        }
    else
        rotationValue = 0.0;

    if (!isAligning && !isStrafingToTarget)
        if (Hardware.isUsingMecanum == true)
            Hardware.mecanumDrive.drive(
                    Hardware.leftDriver.getMagnitude(),
                    Hardware.leftDriver.getDirectionDegrees(),
                    rotationValue);
        else
            Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
                    Hardware.leftDriver.getY());


    if (Hardware.leftDriver.getRawButton(9) == true)
        {
        // Hardware.autoDrive.brake(.1, .1);
        Hardware.autoDrive.stopMovement();
        }
    // =================================================================

    // OPERATOR CONTROLS
    // =================================================================

    if (Hardware.rightOperator.getRawButton(2))
        Hardware.intake.startIntake();
    else if (Hardware.rightOperator.getRawButton(3))
        Hardware.intake.reverseIntake();
    else
        Hardware.intake.stopIntake();


    if (Hardware.leftOperator.getRawButton(2)
            && Math.abs(Hardware.leftOperator.getX()) > .2)
        Hardware.shooter.turnGimbalSlow(
                Hardware.leftOperator.getX() > 0 ? -1 : 1);
    else
        Hardware.shooter.stopGimbal();

    if (Hardware.rightOperator.getRawButton(11))

        {
        if (Hardware.rightOperator.getRawButton(10))
            isTurningGimbal = true;

        if (isTurningGimbal)
            {
            turnValue = Hardware.shooter.turnToBearing(0);
            if (turnValue == Shooter.turnReturn.SUCCESS)
                {
                System.out.println("We are at 0!");
                isTurningGimbal = false;
                }
            else if (turnValue == Shooter.turnReturn.TOO_FAR)
                {
                System.out.println("We are too far!");
                isTurningGimbal = true;
                }
            else if (turnValue == Shooter.turnReturn.WORKING)
                {
                System.out.println("We are turning!");
                isTurningGimbal = true;
                }
            }
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

    if (Hardware.leftOperator.getRawButton(8))
        {
        isAligning = true;
        }

    // Written by Ashley Espeland, has not been tested
    // cameraServo code setting to either the higher or the lower angle position

    // if button 9 equals true and this method had not been called
    // since the last time the button read false (cameraPositionHasChanged)
    if (Hardware.rightOperator.getRawButton(5) == true
            && cameraPositionHasChanged == false)
        {
        // set changeCameraServoPosition to true
        changeCameraServoPosition = true;
        }
    // if changeCamerServoPosition equals true
    if (changeCameraServoPosition == true)
        {
        // if the servo is in the lower position
        if (Hardware.cameraServo
                .getAngle() == LOWER_CAMERASERVO_POSITION)
            {
            // then set the servo to the higher position and change
            // cameraPositionHasChanged to true
            Hardware.cameraServo
                    .setAngle(HIGHER_CAMERASERVO_POSITION);
            cameraPositionHasChanged = true;
            }
        // if the cameraServo is in the higher position
        else if (Hardware.cameraServo
                .getAngle() == HIGHER_CAMERASERVO_POSITION)
            {
            // set the servo to the lower position and change
            // cameraPositionHasChanged to true
            Hardware.cameraServo
                    .setAngle(LOWER_CAMERASERVO_POSITION);
            cameraPositionHasChanged = true;
            }

        }
    // if camera servo position has been changed and the button equals false
    if (changeCameraServoPosition == true
            && Hardware.leftOperator.getRawButton(4) == false)
        {
        // set the cameraPositionHasChanged to false to be able to change
        // the position again when the button is pushed again
        cameraPositionHasChanged = false;
        }


    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8)
                    ||
                    Hardware.rightOperator.getRawButton(8)
                    || Hardware.leftOperator.getRawButton(11));

}
// Hardware.axisCamera.takeSinglePicture(Hardware.leftOperator.getRawButton(8));
// // end
// Hardware.axisCamera.takeSinglePicture(Hardware.rightOperator.getRawButton(8));
// Periodic

// private static boolean isSpeedTesting = false;


private static double rotationValue = 0.0;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static Shooter.turnReturn turnValue = Shooter.turnReturn.SUCCESS;

private static boolean isAligning = false;

private static boolean isTurningGimbal = false;

private static boolean hasPressedFour = false;

private static boolean hasPressedFive = false;

private static boolean previousFireButton = false;

private static boolean isStrafingToTarget = false;

private static boolean preparingToFire = false;

private static double movementSpeed = 0.3;

private static boolean firing = false;

/**
 * stores print statements for future use in the print "bank", statements
 * are commented out when not in use, when you write a new print
 * statement, "deposit" the statement in the correct "bank"
 * do not "withdraw" statements, unless directed to.
 * 
 * NOTE: Keep the groupings below, which correspond in number and
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

    // System.out.println("Flywheel Motor: " + Hardware.shooterMotor.get());
    //
    // System.out.println("Intake Motor: " + Hardware.intakeMotor.get());
    // if (Hardware.rightOperator.getRawButton(11))
    // {
    // Hardware.elevatorMotor.setSpeed(1);
    // System.out.println(
    // "Elevator Motor: " + Hardware.elevatorMotor.get());
    // }
    // System.out.println("Turret Spark: " + Hardware.gimbalMotor.get());

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
    // System.out.println("Ring light relay is On");
    // }
    // else if (!Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // System.out.println("Ring light relay is Off");
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

    // System.out.println("Path Selector: " +
    // Hardware.pathSelector.getPosition());

    // System.out.println("Right UltraSonic distance from bumper: "
    // + Hardware.rightUS.getDistanceFromNearestBumper());
    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------

    /*
     * System.out.println("Right Front Encoder: "
     * + Hardware.rightFrontEncoder.get());
     * System.out.println("Right Rear Encoder: "
     * + Hardware.rightRearEncoder.get());
     * System.out.println("Left Front Encoder: "
     * + Hardware.leftFrontEncoder.get());
     * System.out.println("Left Rear Encoder: "
     * + Hardware.leftRearEncoder.get());
     */

    /*
     * System.out.println("Right Front Encoder Distance: "
     * + Hardware.autoDrive.getRightRearEncoderDistance());
     * System.out.println("Right Rear Encoder Distance: "
     * + Hardware.autoDrive.getRightRearEncoderDistance());
     * System.out.println("Left Front Encoder Distance: "
     * + Hardware.autoDrive.getLeftFrontEncoderDistance());
     * System.out.println("Left Rear Encoder Distance: "
     * + Hardware.autoDrive.getLeftFrontEncoderDistance());
     */

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
    // There isn't one at the moment

    // ---------------------------------
    // Solenoids
    // prints the state of solenoids
    // ---------------------------------
    // There are none at the moment
    // That seems familiar...

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
    // System.out.println("Right Joystick: " +
    // Hardware.rightDriver.getDirectionDegrees());
    // System.out.println("Left Operator: " +
    // Hardware.leftOperator.getDirectionDegrees());
    // System.out.println("Right Operator: " +
    // Hardware.rightOperator.getDirectionDegrees());
    // System.out.println("Twist: " + Hardware.leftDriver.getTwist());


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
private final static double CAMERA_ALIGN_DEADBAND = 10.0                                                                                                                                   // +/-
                                                                                                                                                                                           // Pixels
        / Hardware.axisCamera.getHorizontalResolution();

private final static double CAMERA_ALIGN_CENTER = .478;  // Relative coordinates


// ==========================================
// TUNEABLES
// ==========================================
private final static double LOWER_CAMERASERVO_POSITION = 65;// TODO find

// actual

private final static double HIGHER_CAMERASERVO_POSITION = 90;// TODO find
// actual
// actual value
// public static boolean gearPositionHasChanged = false;

public static boolean changeCameraServoPosition = false;

// public static boolean changeGearServoPosition = false;

public static boolean cameraPositionHasChanged = false;

public static boolean cancelAgitator = false;

public static boolean hasCanceledAgitator = false;

} // end class

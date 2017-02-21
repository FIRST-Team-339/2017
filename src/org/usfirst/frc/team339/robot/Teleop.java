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
        Hardware.tankDrive.setGearPercentage(1,
                Robot.KILROY_XVIII_FIRST_GEAR_PERCENTAGE);
        Hardware.tankDrive.setGearPercentage(2,
                Robot.KILROY_XVIII_SECOND_GEAR_PERCENTAGE);
        Hardware.mecanumDrive.setFirstGearPercentage(
                Robot.KILROY_XVIII_FIRST_GEAR_PERCENTAGE);
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
        Hardware.rightUS.setScalingFactor(
                Hardware.KILROY_XVIII_US_SCALING_FACTOR);
        Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
        Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);
        Hardware.tankDrive.setGear(2);
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);
        Hardware.mecanumDrive.setGear(1);
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

        Hardware.tankDrive.setGearPercentage(1,
                Robot.KILROY_XVII_FIRST_GEAR_PERCENTAGE);
        Hardware.tankDrive.setGearPercentage(2,
                Robot.KILROY_XVII_SECOND_GEAR_PERCENTAGE);
        Hardware.mecanumDrive
                .setFirstGearPercentage(
                        Robot.KILROY_XVII_FIRST_GEAR_PERCENTAGE);
        // Hardware.tankDrive.setRightMotorDirection(MotorDirection.REVERSED);
        Hardware.rightFrontMotor.setInverted(true);
        Hardware.rightRearMotor.setInverted(false);
        Hardware.leftFrontMotor.setInverted(false);
        Hardware.leftRearMotor.setInverted(false);
        Hardware.intakeMotor.setInverted(true);
        Hardware.mecanumDrive
                .setDeadbandPercentageZone(
                        Hardware.KILROY_XVII_JOYSTICK_DEADZONE);
        Hardware.mecanumDrive.setMecanumJoystickReversed(false);
        Hardware.rightUS
                .setScalingFactor(
                        Hardware.KILROY_XVII_US_SCALING_FACTOR);
        Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
        Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);
        Hardware.tankDrive.setGear(1);
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);

        Hardware.mecanumDrive.setDirectionalDeadzone(
                Hardware.KILROY_XVII_JOYSTICK_DIRECTIONAL_DEADZONE);

        }

    SmartDashboard.putNumber("P", Robot.shooterP);
    SmartDashboard.putNumber("I", Robot.shooterI);
    SmartDashboard.putNumber("D", Robot.shooterD);
    SmartDashboard.putNumber("Setpoint", tempSetpoint);
    SmartDashboard.putNumber("Err",
            Hardware.shooterMotor.getError());
} // end Init

static double tempSetpoint = 0.0;

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void periodic ()
{
    Robot.shooterP = SmartDashboard.getNumber("P", Robot.shooterP);
    Robot.shooterI = SmartDashboard.getNumber("I", Robot.shooterI);
    Robot.shooterD = SmartDashboard.getNumber("D", Robot.shooterD);
    tempSetpoint = SmartDashboard.getNumber("Setpoint", tempSetpoint);
    SmartDashboard.putNumber("Err", Hardware.shooterMotor.getError());
    Hardware.shooterMotor.setPID(Robot.shooterP, Robot.shooterI,
            Robot.shooterD);
    Hardware.shooterMotor
            .set(tempSetpoint);

    // previousFireButton = Hardware.leftDriver.getTrigger();
    //
    // if (fireCount > 0)
    // {
    // if (Hardware.shooter.fire())
    // {
    // fireCount--;
    // }
    // }
    // else if (preparingToFire == false)
    // Hardware.shooter.stopFlywheelMotor();
    /*
     * System.out.println("Firecount: " + fireCount);
     * 
     * System.out.println( "Flywheel speed: " +
     * Hardware.shooterMotor.getSpeed());
     */

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

    // =================================================================
    // Driving code
    // =================================================================

    // rotate only when we are pulling the trigger
    if (Hardware.leftDriver.getTrigger())
        {
        rotationValue = Hardware.leftDriver.getTwist();
        }
    else
        rotationValue = 0.0;


    if (!isAligning && !isStrafingToTarget)  // Main
                                             // driving
                                             // function
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
    // Test code for break



    if (Hardware.leftDriver.getRawButton(9) == true)
        {

        if (Hardware.autoDrive.driveInches(12, .3))
            {
            System.out.println("We drove");
            // if (Hardware.autoDrive.isStopped(Hardware.leftFrontEncoder,
            // Hardware.rightFrontEncoder))
            // System.out.println("We Aren't Stopped");
            // {
            // Hardware.leftRearMotor.set(0);
            // Hardware.leftFrontMotor.set(0);
            // Hardware.rightRearMotor.set(0);
            // Hardware.rightFrontMotor.set(0);
            // System.out.println("We stopped now");
            // }
            }

        }

    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // INTAKE CONTROLS
    if (Hardware.leftOperator.getRawButton(2))
        Hardware.intake.startIntake();
    else if (Hardware.leftOperator.getRawButton(3))
        Hardware.intake.reverseIntake();
    else
        Hardware.intake.stopIntake();
    // END INTAKE CONTROLS

    // TURRET OVERRIDE
    if (Hardware.rightOperator.getRawButton(2)
            && Math.abs(Hardware.rightOperator.getX()) > .2)
        Hardware.shooter
                .turnGimbalSlow(
                        Hardware.rightOperator.getX() > 0 ? -1 : 1);
    else
        Hardware.shooter.stopGimbal();
    // END TURRET OVERRIDE

    // ELEVATOR OVERRIDE
    if (Hardware.rightOperator.getRawButton(3))
        Hardware.shooter.loadBalls();
    else if (Hardware.rightOperator.getRawButton(4))
        Hardware.shooter.reverseLoader();
    else
        Hardware.shooter.stopLoader();
    // END ELEVATOR OVERRIDE
    // TESTING SHOOTER
    if (Hardware.rightOperator.getTrigger())
        {
        Hardware.shooter.fire(-200 * Hardware.rightOperator.getZ());
        System.out.println(
                "Shooter motor: " + Hardware.shooterMotor.get());
        /*
         * Hardware.shooter.loadBalls();
         * Hardware.shooterMotor
         * .set(1850.0
         * Hardware.shooter.calculateRPMToMakeGoal(9.25)
         * / 2.0
         * );
         */
        }
    else
        {
        // Hardware.shooter.stopFlywheelMotor();
        }

    // END SHOOTER TESTING
    // =================================================================
    // CAMERA CODE
    // =================================================================

    // Sorry ash, this is all you needed.
    if (Hardware.cameraServoSwitch.isOnCheckNow())
        {
        Hardware.cameraServo.setAngle(190);
        }
    else
        {
        Hardware.cameraServo.setAngle(0);
        }

    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8)
                    || Hardware.rightOperator.getRawButton(8)
                    || Hardware.leftOperator.getRawButton(11));

} // end
  // Periodic

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static Shooter.turnReturn turnValue = Shooter.turnReturn.SUCCESS;

private static double rotationValue = 0.0;

private static boolean isTurningGimbal = false;

private static boolean isAligning = false;

private static boolean previousFireButton = false;

private static boolean isStrafingToTarget = false;

private static boolean preparingToFire = false;

private static double movementSpeed = 0.3;

private static boolean firing = false;

private static boolean hasPressedFour = false;

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
 *          Edited by Ryan McGee Also Edited by Josef Liebl
 * 
 */
public static void printStatements ()
{
    // =================================
    // Motor controllers
    // prints value of the motors
    // =================================
    System.out.println("Delay Pot: " + Hardware.delayPot.get(0, 5));
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
    // if (Hardware.rightOperator.getRawButton(11)) {
    // Hardware.elevatorMotor.setSpeed(1);
    // System.out.println("Elevator Motor: " +
    // Hardware.elevatorMotor.get());
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
    // System.out.println("Backup or fire: " +
    // Hardware.backupOrFire.isOn());
    // System.out.println("Enable Auto: " +
    // Hardware.enableAutonomous.isOn());
    // System.out.println(
    // "Path Selector: " + Hardware.pathSelector.getPosition());

    // System.out.println("Right UltraSonic distance from bumper: "
    // + Hardware.rightUS.getDistanceFromNearestBumper());
    // System.out.println("Right UltraSonic refined distance: "
    // + Hardware.rightUS.getRefinedDistanceValue());
    // System.out.println("Right UltraSonic raw distance: "
    // + Hardware.rightUS.getValue());

    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------

    // System.out.println("Right Front Encoder: " +
    // Hardware.rightFrontEncoder.get());
    // System.out.println("Right Front Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Right Rear Encoder: " +
    // Hardware.rightRearEncoder.get());
    // System.out.println("Right Rear Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Left Front Encoder: " +
    // Hardware.leftFrontEncoder.get());
    // System.out.println("Left Front Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // System.out.println("Left Rear Encoder: " +
    // Hardware.leftRearEncoder.get());
    // System.out.println("Left Rear Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // System.out.println("Right Front Encoder: " +
    // Hardware.rightFrontEncoder.get());
    // System.out.println("Right Front Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Right Rear Encoder: " +
    // Hardware.rightRearEncoder.get());
    // System.out.println("Right Rear Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Left Front Encoder: " +
    // Hardware.leftFrontEncoder.get());
    // System.out.println("Left Front Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // System.out.println("Left Rear Encoder: " +
    // Hardware.leftRearEncoder.get());
    // System.out.println("Left Rear Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------
    // System.out.println("Ball IR: " + Hardware.ballLoaderSensor.isOn());
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

    // GYRO
    // System.out.println("Gyro: " + Hardware.driveGyro.getAngle());

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
    // System.out.println("Left Joystick Direction: " +
    // Hardware.leftDriver.getDirectionDegrees());
    // if (Hardware.leftDriver.getTrigger())
    // {
    // System.out.println("Twist: " + Hardware.leftDriver.getTwist());
    // }
    // System.out.println("Left Joystick: " + Hardware.leftDriver.getY());
    // System.out.println("Right Joystick: " + Hardware.rightDriver.getY());
    // System.out.println("Left Operator: " + Hardware.leftOperator.getY());
    // System.out.println("Right Operator: " +
    // Hardware.rightOperator.getY());
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
private static double LFVal = Hardware.autoDrive
        .getLeftFrontEncoderDistance();

private final static double CAMERA_ALIGN_SPEED = .5;

//// The dead zone for the aligning TODO
private final static double CAMERA_ALIGN_DEADBAND = 10.0 // +/- Pixels
        / Hardware.axisCamera.getHorizontalResolution();

private final static double CAMERA_ALIGN_CENTER = .478;  // Relative coordinates


// ==========================================
// TUNEABLES
// ==========================================
private final static double LOWER_CAMERASERVO_POSITION = 65;                                                                                                                                            // TODO
                                                                                                                                                                                                        // find
// actual value

private final static double HIGHER_CAMERASERVO_POSITION = 90;// TODO find
// actual
// actual value

public static boolean changeCameraServoPosition = false;

public static boolean changeGearServoPosition = false;

public static boolean cameraPositionHasChanged = false;

public static boolean cancelAgitator = false;

public static boolean hasCanceledAgitator = false;

private static boolean isTestingDriveCode = true;

} // end class

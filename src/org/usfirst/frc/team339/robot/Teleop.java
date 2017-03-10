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
import org.usfirst.frc.team339.Utils.Shooter.turnToGoalReturn;
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
    Hardware.cameraServo.setAngle(HIGHER_CAMERASERVO_POSITION);
    // gimbal motors
    Hardware.gimbalMotor
            .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    // Hardware.gimbalMotor.setEncPosition(0); //THIS WILL RESET AFTER AUTO
    // (dont do that)
    // mecanum
    Hardware.mecanumDrive
            .setFirstGearPercentage(
                    Robot.KILROY_XVIII_FIRST_GEAR_PERCENTAGE);

    if (Hardware.isRunningOnKilroyXVIII == true)
        {
        // tank drive values
        Hardware.tankDrive.setGear(2);
        // auto drive values
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);
        // mecanum values
        Hardware.mecanumDrive.setGear(1);
        }
    else
    // kilroy XVII
        {
        // tank drive values
        Hardware.tankDrive.setGear(1);
        // auto drive values
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);
        }
    // PID smartdashboard
    if (tunePIDLoop == true)
        {
        SmartDashboard.putNumber("P", Robot.shooterP);
        SmartDashboard.putNumber("I", Robot.shooterI);
        SmartDashboard.putNumber("D", Robot.shooterD);
        SmartDashboard.putNumber("Setpoint", tempSetpoint);
        SmartDashboard.putNumber("Err",
                Hardware.shooterMotor.getError());
        }

    if (Hardware.driveGyro.isConnected())
        {
        Hardware.driveGyro.calibrate();
        Hardware.driveGyro.reset();
        }
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
    // print values from hardware items
    printStatements();

    // tune pid loop
    if (tunePIDLoop == true)
        {
        Robot.shooterP = SmartDashboard.getNumber("P", Robot.shooterP);
        Robot.shooterI = SmartDashboard.getNumber("I", Robot.shooterI);
        Robot.shooterD = SmartDashboard.getNumber("D", Robot.shooterD);
        tempSetpoint = SmartDashboard.getNumber("Setpoint",
                tempSetpoint);
        SmartDashboard.putNumber("Err",
                Hardware.shooterMotor.getError());
        Hardware.shooterMotor.setPID(Robot.shooterP, Robot.shooterI,
                Robot.shooterD);
        Hardware.shooterMotor
                .set(tempSetpoint);
        }


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

    // light on/off
    if (Hardware.ringlightSwitch.isOnCheckNow() == true)
        {
        Hardware.ringlightRelay.set(Relay.Value.kOn);
        }
    else
        {
        Hardware.ringlightRelay.set(Relay.Value.kOff);
        }

    // gear servo set angles
    // Hardware.gearServo.setAngle(200);
    // Hardware.gearServo.getAngle();



    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // rightOperator stuffs

    // TESTING SHOOTER
    if (Hardware.rightOperator.getTrigger() == true)
        {
        isTurningToGoal = true;
        Hardware.shooter.fire(-200 * Hardware.rightOperator.getZ());
        // System.out.println(
        // Hardware.shooterMotor.set(
        // Hardware.shooter.calculateRPMToMakeGoal(12.25) / 2.0);
        }
    else if (Hardware.leftOperator.getTrigger() == true)
        {
        Hardware.shooter.fire(-200 * Hardware.rightOperator.getZ());
        Hardware.shooter.loadBalls();
        }
    else
        {
        Hardware.shooter.stopFlywheelMotor();
        }

    // END SHOOTER TESTING

    // TURRET OVERRIDE
    if (Hardware.rightOperator.getRawButton(2) == true
            && Math.abs(Hardware.rightOperator.getX()) > .2)
        {
        if (Hardware.rightOperator.getX() > 0)
            {
            Hardware.shooter.turnGimbalMedium(-1);
            }
        else
            {
            Hardware.shooter
                    .turnGimbalMedium(1);
            }
        }
    else if (isTurningGimbal == false && isTurningToGoal == false)
        {
        Hardware.shooter.stopGimbal();
        }
    // END TURRET OVERRIDE

    // SET TURRET TO 0
    if (Hardware.rightOperator.getRawButton(5) == true)
        isTurningGimbal = true;

    if (isTurningGimbal == true
            || turnValue == Shooter.turnReturn.WORKING)
        {
        turnValue = Hardware.shooter.turnToBearing(0);
        isTurningGimbal = false;
        }
    // END SET TURRET TO 0

    // ELEVATOR OVERRIDE
    if (Hardware.rightOperator.getRawButton(3) == true)
        Hardware.shooter.loadBalls();
    else if (Hardware.rightOperator.getRawButton(4) == true)
        Hardware.shooter.reverseLoader();
    else
        Hardware.shooter.stopLoader();
    // END ELEVATOR OVERRIDE

    // leftOperator stuffs

    // ALIGN TURRET
    if (Hardware.leftOperator.getRawButton(4) == true)
        isTurningToGoal = true;

    if (isTurningToGoal == true)
        {
        // turnToGoalValue = Hardware.shooter.turnToGoal();
        isTurningToGoal = !Hardware.shooter.turnToGoalRaw();
        }

    // INTAKE CONTROLS
    if (Hardware.leftOperator.getRawButton(2) == true)
        Hardware.intake.startIntake();
    else if (Hardware.leftOperator.getRawButton(3) == true)
        Hardware.intake.reverseIntake();
    else
        Hardware.intake.stopIntake();
    // END INTAKE CONTROLS

    // =================================================================
    // CAMERA CODE
    // =================================================================
    if (Hardware.cameraServoSwitch.isOnCheckNow() == true)
        {
        Hardware.cameraServo.setAngle(HIGHER_CAMERASERVO_POSITION);
        }
    else
        {
        Hardware.cameraServo.setAngle(LOWER_CAMERASERVO_POSITION);
        }

    Hardware.axisCamera
            .takeSinglePicture(Hardware.leftOperator.getRawButton(8)
                    || Hardware.rightOperator.getRawButton(8)
                    || Hardware.leftOperator.getRawButton(11));
    // =================================================================
    // Driving code
    // =================================================================

    // rotate only when we are pulling the trigger
    if (Hardware.leftDriver.getTrigger() == true)
        {
        rotationValue = ROTATION_FACTOR
                * Hardware.leftDriver.getTwist();
        }
    else
        rotationValue = 0.0;

    if (isDrivingStraight == false && isBraking == false
            && isAligning == false
            && isStrafingToTarget == false)

    // main driving function
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
    // Test code for brake
    if (Hardware.leftDriver.getRawButton(9) == true)
        isDrivingStraight = true;

    if (isDrivingStraight == true)
        {
        // System.out.println("We are driving straight inches");
        isDrivingStraight = !Hardware.autoDrive.driveStraightInches(100,
                .4, .2);
        if (isDrivingStraight == false)
            {
            isBraking = false;
            }
        }

    if (isBraking == true)
        {
        isBraking = !Hardware.autoDrive.brakeToZero(.3);
        // isBraking = !Hardware.autoDrive.timeBrake(-.1, .5);
        System.out.println("We are braking");
        // if (Hardware.autoDrive.isStopped(Hardware.leftRearEncoder,
        // Hardware.rightRearEncoder)
        // && Hardware.autoDrive.isStopped(
        // Hardware.leftFrontEncoder,
        // Hardware.rightFrontEncoder))
        // {
        // Hardware.autoDrive.driveNoDeadband(0.0, 0.0);
        // System.out.println("We are at zero");
        //
        // }
        }

    // if (Hardware.brake.get())
    // {
    //
    // }
    // if (Hardware.setMotorsZero.get())
    // {
    // Hardware.autoDrive.drive(0, 0, 0);
    // System.out.println("We zeroed now");
    // }

} // end
  // Periodic


private static boolean isDrivingStraight = false;

private static boolean isBraking = false;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static Shooter.turnReturn turnValue = Shooter.turnReturn.SUCCESS;

private static boolean isTurningToGoal = false;

private static turnToGoalReturn turnToGoalValue = Shooter.turnToGoalReturn.SUCCESS;

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
    // System.out.println("Delay Pot: " + Hardware.delayPot.get(0, 5));
    // System.out.println("Right Front Motor Controller: "
    // + Hardware.rightFrontMotor.get());
    // System.out.println("Left Front Motor Controller: " +
    // Hardware.leftFrontMotor.get());
    // System.out.println("Right Rear Motor Controller: " +
    // Hardware.rightRearMotor.get());
    // System.out.println("Left Rear Motor Controller: " +
    // Hardware.leftRearMotor.get());
    // System.out.println("Flywheel thingy thing: "
    // + Hardware.shooter.calculateRPMToMakeGoal(9.25) * .5);
    // System.out.println("Flywheel thingy thing speed really: "
    // + Hardware.shooterMotor.get());
    // System.out.println(Hardware.backupOrFireOrHopper.isOn());
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
    // Hardware.backupOrFireOrHopper.isOn());
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
    System.out.println("Right Front Distance: " +
            Hardware.autoDrive.getRightFrontEncoderDistance());
    // System.out.println("Right Rear Encoder: " +
    // Hardware.rightRearEncoder.get());
    System.out.println("Right Rear Encoder Distance: " +
            Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Left Front Encoder: " +
    // Hardware.leftFrontEncoder.get());
    System.out.println("Left Front Encoder Distance: " +
            Hardware.autoDrive.getLeftFrontEncoderDistance());
    // System.out.println("Left Rear Encoder: " +
    // Hardware.leftRearEncoder.get());
    System.out.println("Left Rear Encoder Distance: " +
            Hardware.autoDrive.getLeftRearEncoderDistance());
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

    // System.out.println("Ultrasonic = "
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
    // Hardware.imageProcessor.filterBlobsInYRange(1, .9);
    // if (Hardware.imageProcessor.getLargestBlob() != null)
    // {
    // System.out.println("Center of Mass: " + Hardware.imageProcessor
    // .getLargestBlob().center_mass_y);
    // }
    // else
    // {
    // System.out.println("NO BLOBS!");
    // }
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

private final static double ROTATION_FACTOR = .7;

private final static double LOWER_CAMERASERVO_POSITION = 190;

private static boolean tunePIDLoop = false;
// TODO
// find
// actual value

private final static double HIGHER_CAMERASERVO_POSITION = 150;

// TODO find
// actual
// actual value

public static boolean changeCameraServoPosition = false;

public static boolean changeGearServoPosition = false;

public static boolean cameraPositionHasChanged = false;

public static boolean cancelAgitator = false;

public static boolean hasCanceledAgitator = false;

private static boolean isTestingDriveCode = false;

private static boolean prevState = false;

} // end class

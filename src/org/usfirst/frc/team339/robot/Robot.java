/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Kilroy.java (Team 339 - Kilroy)
//
// CREATED ON: Oct 19, 2012
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// autonomousInit() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// disabledInit() - Initialization code for disabled mode should
// go here. This function will be called one time when the
// robot first enters disabled mode.
// robotInit() - Robot-wide initialization code should go here.
// It will be called exactly 1 time.
// teleopInit() - Initialization code for teleop mode should go here.
// Will be called each time the robot enters teleop mode.
// -----------------------------------------------------
// autonomousPeriodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// disabledPeriodic() - Periodic code for disabled mode should go here.
// Will be called periodically at a regular rate while the robot
// is in disabled mode.
// teleopPeriodic() - Periodic code for teleop mode should go here.
// Will be called periodically at a regular rate while the robot
// is in teleop mode.
// -----------------------------------------------------
// autonomousContinuous() - Continuous code for autonomous mode should
// go here. Will be called repeatedly as frequently as possible
// while the robot is in autonomous mode.
// disabledContinuous() - Continuous code for disabled mode should go
// here. Will be called repeatedly as frequently as possible while
// the robot is in disabled mode.
// teleopContinuous() - Continuous code for teleop mode should go here.
// Will be called repeatedly as frequently as possible while the
// robot is in teleop mode.
// -----------------------------------------------------
// Other functions not normally used
// startCompetition() - This function is a replacement for the WPI
// supplied 'main loop'. This should not normally be written or
// used.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.vision.AxisCamera.ExposureControl;
import edu.wpi.first.wpilibj.vision.AxisCamera.Resolution;
import edu.wpi.first.wpilibj.vision.AxisCamera.WhiteBalance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
// -------------------------------------------------------
/**
 * declares all the code necessary to extend the IterativeRobot class. These are
 * all the methods needed to run Kilroy during a match
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
public class Robot extends IterativeRobot
{
boolean testdasboard = true;
// =================================================
// private data for the class
// =================================================

// -------------------------------------------------------
/**
 * Initialization code for autonomous mode should go here. Will be called
 * once when the robot enters autonomous mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void autonomousInit ()
{

    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started AutonousInit().");
    Hardware.mecanumDrive.setFirstGearPercentage(FIRST_GEAR);
    // =========================================================
    // User code goes below here
    // =========================================================

    // Hardware.rightFrontMotor.setInverted(true);
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    Hardware.leftRearMotor.setInverted(true);
    Hardware.intakeMotor.setInverted(true);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // -------------------------------------
    // Call the Autonomous class's Init function,
    // which contains the user code.
    // -------------------------------------
    Autonomous.init();
    // Hardware.mecanumDrive.setDirectionalDeadzone(0.2);
    // Hardware.tankDrive.setRightJoystickReversed(true);
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed AutonousInit().");
} // end autonomousInit

// -------------------------------------------------------
/**
 * Non-User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 * This in turn calls the Autonomous class's Periodic function, which is
 * where the user code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void autonomousPeriodic ()
{

    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    // System.out.println("Started AutonomousPeriodic().");

    // =========================================================
    // User code goes below here
    // =========================================================
    // -------------------------------------
    // Call the Autonomous class's Periodic function,
    // which contains the user code.
    // -------------------------------------\
    Autonomous.periodic();

    // =========================================================
    // User code goes above here
    // =========================================================

    // feed all motor safeties
    Hardware.leftRearMotorSafety.feed();
    Hardware.rightRearMotorSafety.feed();
    Hardware.leftFrontMotorSafety.feed();
    Hardware.rightFrontMotorSafety.feed();
}

// end autonomousPeriodic

// -------------------------------------------------------
/**
 * Initialization code for disabled mode should go here. Will be called once
 * when the robot enters disabled mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started DisabledInit().");

    // =========================================================
    // User code goes below here
    // =========================================================

    // Hardware.rightFrontMotor.setInverted(true);
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    // Hardware.leftRearMotor.setInverted(true);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // =========================================================
    // User code goes above here
    // =========================================================

    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed DisabledInit().");
} // end disabledInit

// -------------------------------------------------------
/**
 * Periodic code for disabled mode should go here. Will be called
 * periodically at a regular rate while the robot is in disabled mode. Code
 * that can be "triggered" by a joystick button can go here. This can set up
 * configuration things at the driver's station for instance before a match.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledPeriodic ()
{
    // -------------------------------------
    // Watch dog code used to go here.
    // -------------------------------------
    // =========================================================
    // User code goes below here
    // =========================================================

    Hardware.leftFrontMotor.set(0);
    Hardware.rightFrontMotor.set(0);
    Hardware.leftRearMotor.set(0);
    Hardware.rightRearMotor.set(0);

    // =========================================================
    // User code goes above here
    // =========================================================

} // end disabledPeriodic

// -------------------------------------------------------
/**
 * This function is run when the robot is first started up and should be
 * used for any initialization code for the robot.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void robotInit ()
{
    // -------------------------------------
    // Watch dog code used to go here.
    // -------------------------------------
    // =========================================================
    // User code goes below here
    // =========================================================
    Hardware.leftRearEncoder.reset();
    Hardware.rightRearEncoder.reset();
    Hardware.tankDrive.setGearPercentage(1, FIRST_GEAR);
    Hardware.tankDrive.setGearPercentage(2, SECOND_GEAR);
    Hardware.mecanumDrive.setFirstGearPercentage(FIRST_GEAR);
    // Hardware.rightFrontMotor.setInverted(true);
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    Hardware.leftRearMotor.setInverted(true);
    Hardware.intakeMotor.setInverted(true);
    // Hardware.mecanumDrive.setDirectionalDeadzone(0.05);
    Hardware.mecanumDrive
            .setDeadbandPercentageZone(Hardware.joystickDeadzone);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // -------------------------------------
    // motor initialization
    // -------------------------------------
    Hardware.leftRearMotorSafety.setSafetyEnabled(true);
    Hardware.rightRearMotorSafety.setSafetyEnabled(true);
    Hardware.leftFrontMotorSafety.setSafetyEnabled(true);
    Hardware.rightFrontMotorSafety.setSafetyEnabled(true);

    Hardware.leftRearMotorSafety.setExpiration(.25);
    Hardware.rightRearMotorSafety.setExpiration(.25);
    Hardware.leftFrontMotorSafety.setExpiration(.25);
    Hardware.rightFrontMotorSafety.setExpiration(.25);

    // Hardware.tankDrive.setRightJoystickReversed(true);

    // initialize PID values and set the motor to 0.0 because it isn't safe if
    // we don't.
    Hardware.shooterMotor.changeControlMode(TalonControlMode.Speed);
    Hardware.shooterMotor.configPeakOutputVoltage(12f, -12f);
    Hardware.shooterMotor.configNominalOutputVoltage(0f, 0f);
    Hardware.shooterMotor
            .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    Hardware.shooterMotor.configEncoderCodesPerRev(1024);
    Hardware.shooterMotor.setPID(shooterP, shooterI, shooterD);
    Hardware.shooterMotor.setSetpoint(0.0);
    Hardware.shooterMotor.reverseSensor(true);

    if (Hardware.runningInLab == true)
        {
        }
    else
        {
        }

    if (Hardware.isRunningOnKilroyXVIII == true)
        {
        Hardware.rightFrontMotor.setInverted(false);
        Hardware.rightRearMotor.setInverted(false);
        Hardware.leftFrontMotor.setInverted(false);
        Hardware.leftRearMotor.setInverted(true);
        }
    else
        {
        Hardware.rightFrontMotor.setInverted(true);
        Hardware.rightRearMotor.setInverted(false);
        Hardware.leftFrontMotor.setInverted(false);
        Hardware.leftRearMotor.setInverted(false);
        }
    // -------------------------------------
    // Camera initialization
    // -------------------------------------

    // Sends video from both USB Cameras to the Smart Dashboard
    // -last edited on 28 Jan 2017 by Cole Ramos

    // CameraServer.getInstance().startAutomaticCapture(Hardware.cam0);
    // CameraServer.getInstance().startAutomaticCapture(Hardware.cam1);
    // Sets the [max?] FPS's for the USB Cameras. The FPS will generally
    // vary between -1 and +1 this amount.
    // -last edited on 28 Jan 2017 by Cole Ramos
    // Hardware.cam0.setFPS(Hardware.USB_FPS);
    // Hardware.cam1.setFPS(Hardware.USB_FPS);
    // Hardware.cam0.setFPS(Hardware.USB_FPS);
    // Hardware.cam1.setFPS(Hardware.USB_FPS);

    CameraServer.getInstance().addAxisCamera("10.3.39.11");
    Hardware.camForward.setResolution(640, 480);
    Hardware.camBackward.setResolution(640, 480);


    Hardware.axisCamera.writeExposureControl(ExposureControl.kHold);
    Hardware.axisCamera.writeBrightness(1);
    Hardware.axisCamera.writeColorLevel(79);
    Hardware.axisCamera.writeResolution(Resolution.k320x240);
    Hardware.axisCamera
            .writeWhiteBalance(WhiteBalance.kFixedFluorescent1);



    // Sets the max FPS of the Axis Camera; also changes the FPS in the
    // firmware/ web browser
    // of the Axis Camera. If the FPS is not set in the code, the firmware
    // will default to unlimited.
    // TODO Fix this. It isn't actually changing the FPS on the Driver's Station
    // but it is changing the website/ firmware
    // Hardware.axisCamera.writeMaxFPS(Hardware.AXIS_FPS);

    Hardware.ringlightRelay.setDirection(Relay.Direction.kForward);
    Hardware.ringlightRelay.set(Relay.Value.kOff);

    // Sets the scaling factor and general ultrasonic stuff
    Hardware.rightUS.setScalingFactor(.13);
    Hardware.rightUS.setOffsetDistanceFromNearestBummper(3);
    Hardware.rightUS.setNumberOfItemsToCheckBackwardForValidity(3);

    if (Hardware.isRunningOnKilroyXVIII)
        {
        Hardware.rightRearEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.leftRearEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.rightFrontEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        Hardware.leftFrontEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII);
        }
    else
        {
        Hardware.rightRearEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.leftRearEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.leftFrontEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        Hardware.rightFrontEncoder.setDistancePerPulse(
                ENCODER_DISTANCE_PER_PULSE_KILROY_XVII);
        }
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println(
            "Kilroy XVIII is started.  All hardware items created.");
    System.out.println();
    System.out.println();
} // end robotInit

// -------------------------------------------------------
/**
 * Non-User initialization code for teleop mode should go here. Will be
 * called once when the robot enters teleop mode, and will call the Teleop
 * class's Init function, where the User code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started teleopInit().");
    Hardware.mecanumDrive.setFirstGearPercentage(FIRST_GEAR);
    // =========================================================
    // User code goes below here
    // =========================================================
    Teleop.init();
    // Hardware.rightFrontMotor.setInverted(true);
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    Hardware.leftRearMotor.setInverted(true);
    Hardware.intakeMotor.setInverted(true);
    // Hardware.mecanumDrive.setDirectionalDeadzone(0.2);
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed TeleopInit().");
} // end teleopInit

// -------------------------------------------------------
/**
 * Non-User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode, and
 * will in turn call the Teleop class's Periodic function.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopPeriodic ()
{
    // -------------------------------------
    // Call the Teleop class's Periodic function,
    // which contains the user code.
    // -------------------------------------
    Teleop.periodic();
    // feed all motor safeties
    Hardware.leftRearMotorSafety.feed();
    Hardware.rightRearMotorSafety.feed();
    Hardware.leftFrontMotorSafety.feed();
    Hardware.rightFrontMotorSafety.feed();
} // end teleopPeriodic

// -------------------------------------------------------
/**
 * Initialization code for test mode should go here. Will be called once
 * when the robot enters test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
 */
@Override
public void testInit ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testInit

// -------------------------------------------------------
/**
 * Periodic code for test mode should go here. Will be called periodically
 * at a regular rate while the robot is in test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
 */
@Override
public void testPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testPeriodic

// ==========================================
// TUNEABLES
// ==========================================


/**
 * The percentage we want the motors to run at while we are in first gear
 */
public static final double FIRST_GEAR = .6;

/**
 * The percentage we want the motors to run at while we are in second gear
 */
public static final double SECOND_GEAR = 1;

public static final double ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII = .5;

public static final double ENCODER_DISTANCE_PER_PULSE_KILROY_XVII = .0197;

public static double shooterP = .07;

public static double shooterI = .00048;

public static double shooterD = .9;

} // end class

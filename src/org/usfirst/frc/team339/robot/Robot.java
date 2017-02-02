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

import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;

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
public class Robot extends IterativeRobot {

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
	public void autonomousInit() {
		// ---------------------------------------
		// start setup - tell the user we are beginning
		// setup
		// ---------------------------------------
		System.out.println("Started AutonousInit().");
		Hardware.mecanumDrive.setFirstGearPercentage(firstGear);
		// =========================================================
		// User code goes below here
		// =========================================================
		// -------------------------------------
		// Call the Autonomous class's Init function,
		// which contains the user code.
		// -------------------------------------
		Autonomous.init();
		Hardware.mecanumDrive.setDirectionalDeadzone(0.2, 0);
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
	public void autonomousPeriodic() {
		// ---------------------------------------
		// start setup - tell the user we are beginning
		// setup
		// ---------------------------------------
		System.out.println("Started AutonomousPeriodic().");

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

	} // end autonomousPeriodic

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
	public void disabledInit() {
		// ---------------------------------------
		// start setup - tell the user we are beginning
		// setup
		// ---------------------------------------
		System.out.println("Started DisabledInit().");

		// =========================================================
		// User code goes below here
		// =========================================================
		Hardware.rightFrontMotor.setInverted(true); // TODO takeout
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
	public void disabledPeriodic() {
		// -------------------------------------
		// Watch dog code used to go here.
		// -------------------------------------
		// =========================================================
		// User code goes below here
		// =========================================================

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
	public void robotInit() {
		// -------------------------------------
		// Watch dog code used to go here.
		// -------------------------------------
		// =========================================================
		// User code goes below here
		// =========================================================
		Hardware.leftRearEncoder.reset();
		Hardware.rightRearEncoder.reset();
		Hardware.mecanumDrive.setFirstGearPercentage(firstGear);
		Hardware.rightFrontMotor.setInverted(true); // TODO takeout
		// Hardware.rightRearMotor.setInverted(true);
		// Hardware.leftFrontMotor.setInverted(true);
		// Hardware.leftRearMotor.setInverted(true);
		Hardware.mecanumDrive.setDirectionalDeadzone(0.2, 0);
		Hardware.mecanumDrive.setMecanumJoystickReversed(false);
		// -------------------------------------
		// motor initialization
		// -------------------------------------
		Hardware.leftRearMotorSafety.setSafetyEnabled(true);
		Hardware.rightRearMotorSafety.setSafetyEnabled(true);

		Hardware.leftRearMotorSafety.setExpiration(.25);
		Hardware.rightRearMotorSafety.setExpiration(.25);

		// Hardware.rightFrontMotor.setInverted(true);

		if (Hardware.runningInLab == true) {
		} else {
		}

		// -------------------------------------
		// Camera initialization
		// -------------------------------------

		// Sends video from both USB Cameras to the Smart Dashboard
		// -last edited on 28 Jan 2017 by Cole Ramos

		//if (Hardware.MAKE_CAMERA_DROPDOWN_APPEAR == true) {
			//CameraServer.getInstance().startAutomaticCapture(0);
			//CameraServer.getInstance().startAutomaticCapture(1);
			//CameraServer.getInstance().addAxisCamera("10.3.39.11");
		//}

		//CameraServer.getInstance().removeServer();
		//CameraServer.getInstance().removeServer(Hardware.cam1.getName());
		
		
		//if (Hardware.MAKE_CAMERA_DROPDOWN_APPEAR == false) {
			CameraServer.getInstance().startAutomaticCapture(Hardware.cam0);
			CameraServer.getInstance().startAutomaticCapture(Hardware.cam1);
			// Sets the [max?] FPS's for the USB Cameras. The FPS will generally
			// vary between -1 and +1 this amount.
			// -last edited on 28 Jan 2017 by Cole Ramos
			Hardware.cam0.setFPS(Hardware.USB_FPS);
			Hardware.cam1.setFPS(Hardware.USB_FPS);
		//}

		// Sets the max FPS of the Axis Camera; also changes the FPS in the
		// firmware/ web browser
		// of the Axis Camera. If the FPS is not set in the code, the firmware
		// will default to unlimited.
		Hardware.axisCamera.writeMaxFPS(Hardware.AXIS_FPS);

		Hardware.ringlightRelay.setDirection(Relay.Direction.kForward);
		Hardware.ringlightRelay.set(Relay.Value.kOff);
				

		// =========================================================
		// User code goes above here
		// =========================================================
		// ---------------------------------------
		// done setup - tell the user we are complete
		// setup
		// ---------------------------------------
		System.out.println("Kilroy XVIII is started.  All hardware items created.");
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
	public void teleopInit() {
		// ---------------------------------------
		// start setup - tell the user we are beginning
		// setup
		// ---------------------------------------
		System.out.println("Started teleopInit().");
		Hardware.mecanumDrive.setFirstGearPercentage(firstGear);
		// =========================================================
		// User code goes below here
		// =========================================================
		Teleop.init();
		Hardware.rightFrontMotor.setInverted(true); // TODO takeout
		// Hardware.rightRearMotor.setInverted(true);
		// Hardware.leftFrontMotor.setInverted(true);
		// Hardware.leftRearMotor.setInverted(true);
		Hardware.mecanumDrive.setDirectionalDeadzone(0.2, 0);
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
	public void teleopPeriodic() {
		// -------------------------------------
		// Call the Teleop class's Periodic function,
		// which contains the user code.
		// -------------------------------------
		Teleop.periodic();
		System.out.println("Camera Name: " + Hardware.cam0.getName());
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
	public void testInit() {
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
	public void testPeriodic() {
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
	public static double firstGear = .7;

} // end class

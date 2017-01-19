// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the system
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.Hardware;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyCamera;
import org.usfirst.frc.team339.HardwareInterfaces.MomentarySwitch;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionFourWheel;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionMecanum;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import org.usfirst.frc.team339.Vision.VisionScript;
import org.usfirst.frc.team339.Vision.operators.ConvexHullOperator;
import org.usfirst.frc.team339.Vision.operators.HSLColorThresholdOperator;
import org.usfirst.frc.team339.Vision.operators.RemoveSmallObjectsOperator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;

// -------------------------------------------------------
/**
 * puts all of the hardware declarations into one place. In addition, it makes
 * them available to both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------

/**
 * denote whether we are running in the lab or not. This will allow us to test
 * in the lab once the robot is bagged
 */
public static boolean runningInLab = false;
// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------

// **********************************************************
// DIGITAL I/O CLASSES
// **********************************************************
// ====================================
// PWM classes
// ====================================

// ------------------------------------
// Jaguar classes
// ------------------------------------

// ------------------------------------
// Talon classes
// ------------------------------------

/**
 * Default motor controller.
 */
public static TalonSRX rightRearMotor = new TalonSRX(2);

public static TalonSRX rightFrontMotor = new TalonSRX(1);

/**
 * Default motor controller.
 */
public static TalonSRX leftRearMotor = new TalonSRX(3);

public static TalonSRX leftFrontMotor = new TalonSRX(4);
// ------------------------------------
// Victor classes
// ------------------------------------

// ====================================
// CAN classes
// ====================================
public static PowerDistributionPanel pdp = new PowerDistributionPanel(
        0);

// public static CANNetwork CAN = new CANNetwork(Hardware.pdp);
// ====================================
// Relay classes
// ====================================

public static Relay ringlightRelay = new Relay(0);

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------
/**
 * Default motor encoder
 */
public static Encoder leftRearEncoder = new Encoder(10, 11);

/**
 * Default motor encoder
 */
public static Encoder rightRearEncoder = new Encoder(12, 13);

// -----------------------
// Wiring diagram
// -----------------------
// Orange - Red PWM 1
// Yellow - White PWM 1 Signal
// Brown - Black PWM 1 (or PWM 2)
// Blue - White PWM 2 Signal
// For the AMT103 Encoders UNVERIFIED
// B - White PWM 2
// 5V - Red PWM 1 or 2
// A - White PWM 1
// X - index channel, unused
// G - Black PWM 1 or 2
// see http://www.cui.com/product/resource/amt10-v.pdf page 4 for Resolution
// (DIP Switch) Settings (currently all are off)

// -------------------------------------
// Red Light/IR Sensor class
// -------------------------------------

// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================
// public static Compressor compressor = new Compressor();

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------

// ------------------------------------
// Single Solenoids
// ------------------------------------

// **********************************************************
// ANALOG I/O CLASSES
// **********************************************************
// ====================================
// Analog classes
// ====================================
// ------------------------------------
// Gyro class
// ------------------------------------

// -------------------------------------
// Potentiometers
// -------------------------------------
// -------------------------------------

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------
public static KilroyCamera axisCamera = new KilroyCamera(true);

public static VisionScript visionScript = new VisionScript(
        new HSLColorThresholdOperator(88, 138, 95, 255, 29, 171),
        new RemoveSmallObjectsOperator(1, true),
        new ConvexHullOperator(false));

public static ImageProcessor imageProcessor = new ImageProcessor(
        axisCamera, visionScript);
// -------------------------------------
// declare the USB camera server and the
// USB camera it serves
// -------------------------------------

// **********************************************************
// DRIVER STATION CLASSES
// **********************************************************

// ------------------------------------
// DriverStations class
// ------------------------------------
/**
 * The software object representing the driver station.
 */
public static final DriverStation driverStation = DriverStation
        .getInstance();

// ------------------------------------
// Joystick classes
// ------------------------------------
/**
 * The left joystick controlling the drive train.
 */
public static Joystick leftDriver = new Joystick(0);

/**
 * The right joystick controlling the drive train.
 */
public static Joystick rightDriver = new Joystick(1);

/**
 * The left joystick controlling misc operations on the robot.
 */
public static Joystick leftOperator = new Joystick(3);

public static MomentarySwitch ringlightSwitch = new MomentarySwitch(
        leftOperator, 2, false);

/**
 * The right joystick controlling misc operations on the robot.
 */
public static Joystick rightOperator = new Joystick(2);

// **********************************************************
// Kilroy's Ancillary classes
// **********************************************************

// -------------------------------------
// PID tuneables
// -------------------------------------

// -------------------------------------
// PID classes
// -------------------------------------

// ------------------------------------
// Transmission class
// ------------------------------------
// public static Transmission_old transmission = new Transmission_old(
// rightRearMotor, leftRearMotor, rightRearEncoder,
// leftRearEncoder);

// ------------------------------------
// Drive system
// ------------------------------------
public static TransmissionMecanum mecanumDrive = new TransmissionMecanum(
        rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor);

public static TransmissionFourWheel tankDrive = new TransmissionFourWheel(
        rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor);

public static boolean usingMecanum = true;
// -------------------
// Assembly classes (e.g. forklift)
// -------------------

// ------------------------------------
// Utility classes
// ------------------------------------
/**
 * Default timer.
 */
public static final Timer kilroyTimer = new Timer();

/**
 * Default motor safety
 */
public static final MotorSafetyHelper leftRearMotorSafety = new MotorSafetyHelper(
        leftRearMotor);

/**
 * Default motor safety
 */
public static final MotorSafetyHelper rightRearMotorSafety = new MotorSafetyHelper(
        rightRearMotor);


public static final MotorSafetyHelper rightFrontMotorSafety = new MotorSafetyHelper(
        rightFrontMotor);

public static final MotorSafetyHelper leftFrontMotorSafety = new MotorSafetyHelper(
        leftFrontMotor);

public static final int MINIMUM_AXIS_CAMERA_BRIGHTNESS = 6;

} // end class

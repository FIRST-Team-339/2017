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

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Shooter;
import org.usfirst.frc.team339.Utils.pidTuning.CanTalonPIDTuner;
import org.usfirst.frc.team339.Utils.pidTuning.SmartDashboardPIDTunerDevice;
import edu.wpi.cscore.VideoCamera;
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
    Hardware.newClimberMotor.set(0.0);
    // ---------------------------------------
    // Servo init
    // ---------------------------------------
    // Hardware.cameraservoX.setAngle(HIGHER_CAMERASERVO_POSITIONX);
    // Hardware.cameraservoY.setAngle(HIGHER_CAMERASERVO_POSITIONY);
    // gimbal motors
    // Hardware.gimbalMotor
    // .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    // Hardware.gimbalMotor.setEncPosition(0);
    // mecanum
    Hardware.mecanumDrive
            .setFirstGearPercentage(
                    Robot.KILROY_XVIII_FIRST_GEAR_PERCENTAGE);
    Hardware.tankDrive
            .setGearPercentage(1,
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
        Hardware.autoDrive.setDebugStatus(false);
        // tank drive values
        Hardware.tankDrive.setGear(1);
        // auto drive values
        Hardware.autoDrive.setDriveCorrection(.3);
        Hardware.autoDrive.setEncoderSlack(1);
        }
    // PID smartdashboard
    // if (tunePIDLoop == true)
    // {
    // SmartDashboard.putNumber("P", Robot.shooterP);
    // SmartDashboard.putNumber("I", Robot.shooterI);
    // SmartDashboard.putNumber("D", Robot.shooterD);
    // SmartDashboard.putNumber("Setpoint", tempSetpoint);
    // // SmartDashboard.putNumber("Err",
    // // Hardware.shooterMotor.getError());
    // }
    // Hardware.shooterMotor.changeControlMode(TalonControlMode.Speed);
    // // put back in once finished testing!!!
    // Hardware.shooterMotor.configPeakOutputVoltage(12f, 0f);
    // Hardware.shooterMotor.configNominalOutputVoltage(0f, 0f);
    // Hardware.shooterMotor
    // .setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    // Hardware.shooterMotor.configEncoderCodesPerRev(1024);
    // Hardware.shooterMotor.setPID(shooterP, shooterI, shooterD);
    // Hardware.shooterMotor.setSetpoint(0.0);
    // Hardware.shooterMotor.reverseSensor(true);
    // testingTalon.setProfile(0);

    CanTuner.setupMotorController(FeedbackDevice.QuadEncoder,
            TalonControlMode.Speed, 1024, false);
    testingTalon.setPID(0.0, 0.0, 0.0);
    testingTalon.setSetpoint(0.0);
    // put stuff on smartdashboard
    SmartDashboard.putNumber("DB/Slider 0", 0);
    SmartDashboard.putNumber("DB/Slider 1", 0);
    SmartDashboard.putNumber("DB/Slider 2", 0);
    SmartDashboard.putNumber("DB/Slider 3", 0);


    // thread1.start();
    //
    // System.out.println("Got here 1");
    //
    // System.out.println("Thread 1 'return' : " + thread1.valueForTeleop);
    //
    // System.out
    // .println("Thread 1 State Pre Join: " + thread1.getState());



    // thread1.join();

    // System.out
    // .println("Thread 1 State Post Join: " + thread1.getState());

    // thread2.start();
    // Hardware.driveGyro.calibrate();
    // Hardware.driveGyro.reset();

} // end Init

static double tempSetpoint = 0.0;

static CANTalon testingTalon = new CANTalon(12);

private static CanTalonPIDTuner CanTuner = new CanTalonPIDTuner(
        testingTalon, 0);

private static SmartDashboardPIDTunerDevice PIDTuner = new SmartDashboardPIDTunerDevice(
        CanTuner);

// tune pid loop
/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void periodic ()
{

    double testDashboard = 0.0;
    double testDashboard2 = 0.0;

    testDashboard = SmartDashboard.getNumber("DB/Slider 0", 0.0);
    testDashboard2 = SmartDashboard.getNumber("DB/Slider 1", 0.0);
    // ADD IN???
    // System.out.println(testDashboard);
    // System.out.println(testDashboard);
    // System.out.println(testDashboard);
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
        // SmartDashboard.putNumber("Err",
        // Hardware.shooterMotor.getError());
        // Hardware.shooterMotor.setPID(Robot.shooterP, Robot.shooterI,
        // Robot.shooterD);
        // Hardware.shooterMotor
        // .set(tempSetpoint);
        }




    // Hardware.shooterMotor
    // .set(tempSetpoint);
    // printStatements();
    // printStatements();

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

    if (tunePIDLoop == true)
        {
        PIDTuner.update();
        System.out.println("Error: " + testingTalon.getError());
        System.out.println("Setpoint: " + testingTalon.getSetpoint());
        System.out.println("Velocity: " + testingTalon.getSpeed());
        System.out.println("P, I, D: " + testingTalon.getP() + ", "
                + testingTalon.getI() + ", " + testingTalon.getD());
        }
    // Hardware.shooterMotor
    // .set(tempSetpoint);

    // gear servo set angles
    // Hardware.gearServo.setAngle(200);
    // Hardware.gearServo.getAngle();


    // TODO delete this shortcut


    // Hardware.leftRearTest.watchJoystick(Hardware.leftOperator.getY());
    // Hardware.leftFrontTest.watchJoystick(Hardware.leftOperator.getY());
    // Hardware.rightRearTest.watchJoystick(Hardware.rightOperator.getY());
    // Hardware.rightFrontTest
    // .watchJoystick(Hardware.rightOperator.getY());



    // System.out.println("Left Rear Amps: " +
    // Hardware.pdp.getCurrent(14));
    // System.out.println("Left Front Amps: " +
    // Hardware.pdp.getCurrent(12));
    // System.out.println("Right Rear Amps: " +
    // Hardware.pdp.getCurrent(15));
    // System.out.println("Right Front Amps: " +
    // Hardware.pdp.getCurrent(13));
    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // @ANE Updated motor names
    // rightOperator stuffs
    // If the operator is pressing right button 10

    // @ANE add back in
    if (Hardware.rightOperator.getTrigger() == true
    /* Hardware.rightOperator.getRawButton(10) */)
        {
        // Climb climb climb!
        Hardware.newClimberMotor.set(climbingSpeed);
        // System.out.println("climber speed = "
        // + Hardware.newClimberMotor.getSpeed());
        }
    else if (Hardware.rightOperator.getRawButton(6) == true
            && Hardware.rightOperator.getRawButton(7) == true)
        {
        Hardware.newClimberMotor.set(reverseClimbingSpeed);
        }
    else// They're not pressing it
        {
        Hardware.newClimberMotor.set(0.0);// STOP THE MOTOR
        }

    // Calibration code

    // Hardware.newClimberMotor.set(Hardware.rightOperator.getY());

    // TESTING SHOOTER
    if (Hardware.rightOperator.getTrigger() == true)
        {
        // Hardware.shooter.turnToGoalRaw();
        // Hardware.shooter.fire(-200 * Hardware.rightOperator.getZ());
        // System.out.println(
        // Hardware.shooterMotor.set(
        // Hardware.shooter.calculateRPMToMakeGoal(12.25) / 2.0);
        }
    else if (Hardware.leftOperator.getTrigger() == true)
        {
        // Hardware.shooter.fire(-200 * Hardware.rightOperator.getZ());
        // Hardware.shooter.loadBalls();
        }
    else
        {
        // Hardware.shooter.stopFlywheelMotor();
        }

    // END SHOOTER TESTING

    // TURRET OVERRIDE
    if (Hardware.rightOperator.getRawButton(2) == true
            && Math.abs(Hardware.rightOperator.getX()) > .2)
        {
        if (Hardware.rightOperator.getX() > 0)
            {
            // Hardware.shooter.turnGimbalSlow(1);
            }
        else
            {
            // Hardware.shooter
            // .turnGimbalSlow(-1);
            }
        }
    else if (isTurningGimbal == false && isTurningToGoal == false)
        {
        // Hardware.shooter.stopGimbal();
        }
    // END TURRET OVERRIDE

    // SET TURRET TO 0
    if (Hardware.rightOperator.getRawButton(5) == true)
        isTurningGimbal = true;

    if (isTurningGimbal == true
            || turnValue == Shooter.turnReturn.WORKING)
        {
        // turnValue = Hardware.shooter.turnToBearing(0);
        isTurningGimbal = false;
        }
    // END SET TURRET TO 0

    // ELEVATOR OVERRIDE
    // if (Hardware.rightOperator.getRawButton(3) == true)
    // Hardware.shooter.loadBalls();
    // else if (Hardware.rightOperator.getRawButton(4) == true)
    // Hardware.shooter.reverseLoader();
    // else if (Hardware.leftOperator.getRawButton(3) == false)
    // Hardware.shooter.stopLoader();
    // END ELEVATOR OVERRIDE

    // leftOperator stuffs

    // ALIGN TURRET
    if (Hardware.leftOperator.getRawButton(4) == true)
        isTurningToGoal = true;

    if (isTurningToGoal == true)
        {
        // turnToGoalValue = Hardware.shooter.turnToGoal();
        // isTurningToGoal = !Hardware.shooter.turnToGoalRaw();
        }

    // @ANE removed for sanity's sake
    // INTAKE CONTROLS
    // if (Hardware.leftOperator.getRawButton(2) == true)
    // Hardware.intake.startIntake();
    // else if (Hardware.leftOperator.getRawButton(3) == true)
    // Hardware.intake.reverseIntake();
    // else if (Hardware.rightOperator.getRawButton(3) == false)
    // Hardware.intake.stopIntake();
    // END INTAKE CONTROLS

    // OLD CLIMBER CODE
    // if (Hardware.rightOperator.getRawButton(10))
    // {
    // Hardware.climberMotor.set(-1);
    // }
    // else
    // {
    // Hardware.climberMotor.set(0);
    // }
    // END CLIMBER
    // =================================================================
    // CAMERA CODE
    // =================================================================

    if (Hardware.rightOperator.getRawButton(11))
        {
        Hardware.testingProcessor.setCameraSettings(0,
                VideoCamera.WhiteBalance.kFixedIndoor, 50);
        }
    else if (Hardware.rightOperator.getRawButton(10))
        {
        Hardware.testingProcessor.setDefaultCameraSettings();
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
        rotationValue = Hardware.leftDriver.getTwist();
        }
    else
        rotationValue = 0.0;


    // TODO Temporarily Commented out for speed testing; uncomment later
    if (isTestingDrive == false && isTestingCamera == false)
    // main driving function
        {
        if (Hardware.isUsingMecanum == true)
            {
            // if (Hardware.leftDriver.getY() > -.5
            // && Hardware.leftDriver.getY() < -0.15)
            // {
            // Hardware.leftFrontMotor.set(Hardware.leftDriver.getY());
            // Hardware.leftRearMotor.set(Hardware.leftDriver.getY());
            // Hardware.rightFrontMotor
            // .set(Hardware.leftDriver.getY());
            // Hardware.rightRearMotor
            // .set(-Hardware.leftDriver.getY());
            // }
            // if (Hardware.leftDriver.getY() <= -0.5)
            // {
            // testingSpeed = .85;
            //
            // Hardware.leftFrontMotor.set(testingSpeed);
            // Hardware.leftRearMotor.set(testingSpeed);
            // Hardware.rightFrontMotor.set(testingSpeed);
            // Hardware.rightRearMotor.set(-testingSpeed);
            // // System.out.println("Right Front draw = "
            // // + Hardware.pdp.getCurrent(1));
            // // System.out.println("Right Rear draw = "
            // // + Hardware.pdp.getCurrent(2));
            // // System.out.println("Left Front draw = "
            // // + Hardware.pdp.getCurrent(4));
            // // System.out.println("Left Rear draw = "
            // // + Hardware.pdp.getCurrent(3));
            // }
            // else
            // {
            // Hardware.leftFrontMotor.set(0.0);
            // Hardware.leftRearMotor.set(0.0);
            // Hardware.rightFrontMotor.set(0.0);
            // Hardware.rightRearMotor.set(0.0);
            // }
            // ADD BACK IN AFTER TESTING IS DONE


            // ASK ASHLEY ESPELAND IF YOU HAVE QUESTIONS @ANE
            // In order to change the top speed for going forward and
            // backward, you must go into Robot.java and change the
            // KILROY_XVIII_FIRST_GEAR_PERCENTAGE which is currently set
            // at .7
            Hardware.mecanumDrive.drive(
                    Hardware.leftDriver.getMagnitude(),
                    Hardware.leftDriver.getDirectionDegrees(),
                    rotationValue);

            // get speed code

            // System.out
            // .println("Left Rear Speed: "
            // + Hardware.leftRearMotor.getSpeed());
            // System.out.println("Left Front Speed: "
            // + Hardware.leftFrontMotor.getSpeed());
            // System.out.println("Right Rear Speed: "
            // + Hardware.rightRearMotor.getSpeed());
            // System.out
            // .println("Right Front Speed: "
            // + Hardware.rightFrontMotor.getSpeed());

            // get speed tester code

            // System.out.println(
            // "Left DRIVER " + Hardware.leftDriver.getY());
            // System.out.println(
            // "Left rear speed tester " + Hardware.leftRearTest
            // .watchJoystick(Hardware.leftDriver.getY(),
            // Hardware.rightDriver.getY()));
            // System.out.println(
            // "Right rear speed tester " + Hardware.rightRearTest
            // .watchJoystick(Hardware.leftDriver.getY(),
            // Hardware.rightDriver.getY()));
            // System.out.prinftln(
            // "Left rear speed tester " + Hardware.leftFrontTest
            // .watchJoystick(Hardware.leftDriver.getY(),
            // Hardware.rightDriver.getY()));
            // System.out.println(
            // "Right front speed tester "
            // + Hardware.rightFrontTest
            // .watchJoystick(
            // Hardware.leftDriver.getY(),
            // Hardware.rightDriver
            // .getY()));

            // Hardware.mecanumDrive.drive(
            // Hardware.leftDriver.getMagnitude(),
            // Hardware.leftDriver.getDirectionDegrees(),
            // rotationValue);
            }
        else
            {
            Hardware.tankDrive.drive(Hardware.rightDriver.getY(),
                    Hardware.leftDriver.getY());
            }



        // ----------------------TESTING DRIVE FUNCTIONS--------------------

        if (Hardware.leftDriver.getRawButton(9) == true)
            {
            isTestingDrive = true;
            }

        if (isTestingDrive == true)
            {

            if (Hardware.leftOperator.getRawButton(6) == true
                    || Hardware.leftOperator.getRawButton(7) == true)
                {
                isTestingDrive = false;
                }
            }
        }

} // end
  // Periodic

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static Shooter.turnReturn turnValue = Shooter.turnReturn.SUCCESS;

private static boolean isTurningToGoal = false;

private static double rotationValue = 0.0;

private static boolean isTurningGimbal = false;

private static boolean isTestingCamera = false;

private static boolean isTestingDrive = false;

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
    // Hardware.imageProcessor.processImage();
    //
    // if (Hardware.imageProcessor.getLargestBlob() != null)
    // System.out.println("Camera: " + Hardware.imageProcessor
    // .getLargestBlob().center_mass_x);

    // =================================
    // Motor controllers
    // prints value of the motors
    // =================================

    // System.out.println("Delay Pot: " + Hardware.delayPot.get(0, 5));
    // System.out.println("Left Front Motor Controller: " +
    // Hardware.leftFrontMotor.get());
    // System.out.println("Right Rear Motor Controller: " +
    // Hardware.rightRearMotor.get());
    // System.out.println("Left Rear Motor Controller: " +
    // Hardware.leftRearMotor.get());
    // System.out.println("Right Front Motor Controller: "
    // + Hardware.rightFrontMotor.get());
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
    //
    // System.out.println(
    // "Base Line Path: " + Hardware.autoBaseLinePath.isOn());
    // System.out
    // .println("Side Gear Path: " + Hardware.sideGearPath.isOn());

    // System.out.println("UltraSonic distance from bumper: "
    // + Hardware.ultraSonic.getDistanceFromNearestBumper());
    // System.out.println("Right UltraSonic refined distance: "
    // + Hardware.ultraSonic.getRefinedDistanceValue());
    // System.out.println("Right UltraSonic raw distance: "
    // + Hardware.ultraSonic.getValue());
    // System.out.println("IR 1: " + Hardware.gearSensor1.isOn());
    // System.out.println("IR 2: " + Hardware.gearSensor2.isOn());
    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------

    // System.out.println("Right Front Encoder: " +
    // Hardware.rightFrontEncoder.get());
    // System.out.println("Right Front Distance: " +
    // Hardware.autoDrive.getRightFrontEncoderDistance());
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
    // Hardware.autoDrive.getLeftRearEncoderDistance());
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

    // =========================
    // Servos
    // =========================
    // System.out.println(
    // "camera servo Y" + Hardware.cameraservoY.getAngle());
    // System.out.println(
    // "camera servo X" + Hardware.cameraservoX.getAngle());
    // ================
    // GYRO
    // System.out.println("Gyro: " + Hardware.driveGyro.getAngle());
    // System.out.println(
    // "Init Gyro Val: " + Hardware.driveGyro.getRate());
    // System.out.println(
    // "Gyro Is Connected: " + Hardware.driveGyro.isConnected());
    // =================
    // System.out.println("Ultrasonic = "
    // + Hardware.ultraSonic.getRefinedDistanceValue());
    // System.out.println("Ultrasonic = "
    // + Hardware.ultraSonic2.getRefinedDistanceValue());
    // System.out.println("Ultrasonic refined: "
    // + Hardware.ultraSonic.getRefinedDistanceValue());

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
    // System.out.println("USB Cam Brightness: "
    // + "Hardware.camForward.getBrightness()");
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
    // {
    // System.out.println("Angle to target 1: "
    // + Hardware.imageProcessor.getYawAngleToTarget(
    // Hardware.imageProcessor.getLargestBlob()));
    // System.out.println("Angle to target 2: "
    // + Hardware.imageProcessor.getYawAngleToTarget(
    // Hardware.imageProcessor.getNthSizeBlob(1)));
    // System.out.println("Distance from peg: "
    // + Hardware.imageProcessor.getZDistanceToGearTarget(
    // Hardware.imageProcessor.getLargestBlob(),
    // Hardware.imageProcessor.getNthSizeBlob(1)));
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

//// The dead zone for the aligning TODO
private final static double CAMERA_ALIGN_DEADBAND = 10.0 // +/- Pixels
        / Hardware.axisCamera.getHorizontalResolution();

private static boolean tunePIDLoop = false;
// TODO find actual value

private final static double HIGHER_CAMERASERVO_POSITIONY = 90;// TODO find
                                                              // actual value

private final static double HIGHER_CAMERASERVO_POSITIONX = 90;// TODO find
                                                              // actual value

public static boolean changeCameraServoPosition = false;

public static boolean changeGearServoPosition = false;

public static boolean cameraPositionHasChanged = false;

public static boolean cancelAgitator = false;

public static boolean hasCanceledAgitator = false;

public static double testingSpeed;

public static double climbingSpeed = -1;

public static double reverseClimbingSpeed = .5;

// temporary variable to test the ability to send information from a
// separate thread to teleop
public static int valueFromThread;

} // end class

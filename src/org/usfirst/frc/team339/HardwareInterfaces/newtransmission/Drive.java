package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyGyro;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.newtransmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.newtransmission.TransmissionBase.TransmissionType;
import org.usfirst.frc.team339.vision.opencv.VisionProcessor;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The class that controls autonomous driving functions or
 * driver-assisting functions based on sensors.
 * 
 * @author Ryan McGee
 * @written 7/26/2017
 */
public class Drive
{
// The transmission objects. Only one is used based on the transmission
// object that is input.

// The reason this is not one "TransmissionBase" object is that the drive
// functions of each type require a different number of joysticks/input
// values. Thus, inheritance is hard.
private TankTransmission tankTransmission = null;

private TractionTransmission tractionTransmission = null;

private MecanumTransmission mecanumTransmission = null;

private Encoder leftFrontEncoder = null, rightFrontEncoder = null,
        leftRearEncoder = null, rightRearEncoder = null;

private UltraSonic ultrasonic = null;

private KilroyGyro gyro = null;

private VisionProcessor visionProcessor = null;

private final TransmissionType transmissionType;

/**
 * Creates the Drive object. If a sensor listed is not used (except for
 * encoders), set it to null.
 * 
 * 
 * @param transmission
 *            The robot's transmission object
 * @param leftFrontEncoder
 *            The left-front corner encoder
 * @param rightFrontEncoder
 *            The right-front corner encoder
 * @param leftRearEncoder
 *            The left-rear corner encoder
 * @param rightRearEncoder
 *            The right-rear corner encoder
 * @param ultrasonic
 *            The sensor that finds distance using sound
 * @param gyro
 *            A sensor that uses a spinning disk to measure rotation.
 * @param visionProcessor
 *            The camera's vision processing code, as a sensor.
 */
public Drive (TransmissionBase transmission, Encoder leftFrontEncoder,
        Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        UltraSonic ultrasonic, KilroyGyro gyro,
        VisionProcessor visionProcessor)
{
    this.transmissionType = transmission.getType();
    this.leftFrontEncoder = leftFrontEncoder;
    this.rightFrontEncoder = rightFrontEncoder;
    this.leftRearEncoder = leftRearEncoder;
    this.rightRearEncoder = rightRearEncoder;

    this.ultrasonic = ultrasonic;
    this.gyro = gyro;
    this.visionProcessor = visionProcessor;

    init(transmission);
}

/**
 * Creates the Drive object. If a sensor listed is not used (except for
 * encoders), set it to null.
 * Setup for Traction drive (only 2 motors/encoders)
 * 
 * @param transmission
 *            The robot's transmission object
 * @param leftEncoder
 *            The left-side encoder
 * @param rightEncoder
 *            The right-side encoder
 * @param ultrasonic
 *            The sensor that finds distance using sound
 * @param gyro
 *            A sensor that uses a spinning disk to measure rotation.
 */
public Drive (TransmissionBase transmission, Encoder leftEncoder,
        Encoder rightEncoder, UltraSonic ultrasonic,
        KilroyGyro gyro)
{
    this.transmissionType = transmission.getType();
    this.leftRearEncoder = leftEncoder;
    this.rightRearEncoder = rightEncoder;

    this.ultrasonic = ultrasonic;
    this.gyro = gyro;

    init(transmission);
}

private void init (TransmissionBase transmission)
{

    // McGee your code needs more comments
    // Only sets the transmission if it is the same type. Other transmission
    // objects get set to null.
    switch (transmissionType)
        {
        case MECANUM:
            this.mecanumTransmission = (MecanumTransmission) transmission;
            break;
        case TANK:
            this.tankTransmission = (TankTransmission) transmission;
            break;
        case TRACTION:
            this.tractionTransmission = (TractionTransmission) transmission;
            break;
        default:
            System.out.println(
                    "There was an error setting up the DRIVE class. Please check the declaration for a valid transmission object.");
            break;

        }
}

/**
 * Gets the transmission object stored. ONLY use it for transmission.stop()
 * and transmission.driveRaw()
 * 
 * @return The current transmission object used in the Drive class
 */
public TransmissionBase getTransmission ()
{
    switch (transmissionType)
        {
        case MECANUM:
            return mecanumTransmission;
        case TANK:
            return tankTransmission;
        case TRACTION:
            return tractionTransmission;
        default:
            return null;
        }
}

// ================ENCODER METHODS================
/**
 * Different groups of wheels for use in encoder data collection.
 * 
 * @author Ryan McGee
 */
public enum WheelGroups
    {
/**
 * All wheels combined
 */
ALL,
/**
 * Both front and back on the left side
 */
LEFT_SIDE,
/**
 * Both front and back on the right side
 */
RIGHT_SIDE,
/**
 * Both left and right rear wheels
 */
REAR
    }

/**
 * Calculates the appropriate Distance Per Pulse for the encoders, using:
 * dpp = (Encoder Teeth / Output Teeth) * (wheel Diameter * PI) / (pulses per
 * revolution).
 * The default pulses per revolution on Kilroy's normal optical encoders is
 * 1,440.
 * 
 * @param pulsesPerRotation
 *            How many times the encoder pulses in one rotation of the ENCODER.
 * @param gearRatio
 *            The gear ratio of the Encoder:Wheel (Encoder Teeth / Output Teeth)
 * @param wheelDiameter
 *            The size of the drive wheels.
 */
public void setEncoderDistancePerPulse (double pulsesPerRotation,
        double gearRatio, double wheelDiameter)
{
    this.setEncoderDistancePerPulse(
            (gearRatio * wheelDiameter * Math.PI) / pulsesPerRotation);
}

/**
 * Sets how far the robot has driven per pulse the encoder reads.
 * This value should be much lower than one, as there are usually
 * hundreds of pulses per rotation.
 * 
 * To calculate, reset the encoders and
 * push the robot forwards, say, five feet. Then count the number of pulses
 * and do: (5x12)/pulses to get this in inches.
 * 
 * @param value
 *            The encoder distance per pulse.
 */
public void setEncoderDistancePerPulse (double value)
{
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        {
        leftFrontEncoder.setDistancePerPulse(value);
        rightFrontEncoder.setDistancePerPulse(value);
        leftRearEncoder.setDistancePerPulse(value);
        rightRearEncoder.setDistancePerPulse(value);
        }
    else if (transmissionType == TransmissionType.TRACTION)
        {
        leftRearEncoder.setDistancePerPulse(value);
        rightRearEncoder.setDistancePerPulse(value);
        }
}

/**
 * Sets the encoder's stored pulses back to zero.
 */
public void resetEncoders ()
{
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        {
        leftFrontEncoder.reset();
        rightFrontEncoder.reset();
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
    else if (transmissionType == TransmissionType.TRACTION)
        {
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
}

/**
 * Gets the averages of certain wheel groups. All values are the absolute value
 * to stop
 * negative numbers from affecting the average.
 * 
 * @param encoderGroup
 * @return
 */
public double getEncoderDistanceAverage (WheelGroups encoderGroup)
{
    switch (encoderGroup)
        {
        case ALL:
            return (Math.abs(leftFrontEncoder.getDistance())
                    + Math.abs(rightFrontEncoder.getDistance())
                    + Math.abs(leftRearEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 4.0;
        case LEFT_SIDE:
            return (Math.abs(leftFrontEncoder.getDistance())
                    + Math.abs(leftRearEncoder.getDistance())) / 2.0;
        case RIGHT_SIDE:
            return (Math.abs(rightFrontEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 2.0;
        case REAR:
            return (Math.abs(leftRearEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 2.0;
        default:
            return 0.0;
        }
}

/**
 * Tests whether any encoder reads larger than the input length. Useful for
 * knowing
 * when to stop the robot.
 * 
 * @param length
 *            The desired length
 * @return True when any encoder is past length
 */
private boolean isAnyEncoderLargerThan (double length)
{
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        return (Math.abs(leftFrontEncoder.getDistance()) > length
                || Math.abs(rightFrontEncoder.getDistance()) > length
                || Math.abs(leftRearEncoder.getDistance()) > length
                || Math.abs(rightRearEncoder.getDistance()) > length);
    return (Math.abs(leftRearEncoder.getDistance()) > length
            || Math.abs(rightRearEncoder.getDistance()) > length);

}

// ================ DRIVE METHODS ================

/**
 * Resets the Drive class's functions, in case they were cut short.
 */
public void reset ()
{
    this.brakeInit = true;
    this.driveInchesInit = true;
    this.driveStraightInchesInit = true;
    this.turnDegreesInit = true;
}

/**
 * Stops the robot suddenly, to prevent drifting during autonomous functions,
 * and increase the precision.
 * 
 * @return
 *         Whether or not the robot has stopped moving.
 */
public boolean brake ()
{
    if (this.brakeInit)
        {
        // Get the initial powers sent to the motors, to reverse them while
        // braking.
        this.brakeMotorVals[0] = this.getTransmission()
                .getSpeedController(MotorPosition.LEFT_FRONT).get();
        this.brakeMotorVals[1] = this.getTransmission()
                .getSpeedController(MotorPosition.LEFT_REAR).get();
        this.brakeMotorVals[2] = this.getTransmission()
                .getSpeedController(MotorPosition.RIGHT_FRONT).get();
        this.brakeMotorVals[3] = this.getTransmission()
                .getSpeedController(MotorPosition.RIGHT_REAR).get();

        System.out.println(brakeMotorVals);

        this.brakeInit = false;
        }

    // Only test the encoders every (COLLECTION_TIME) milliseconds.
    if (System.currentTimeMillis()
            - this.previousBrakeTime > COLLECTION_TIME)
        {
        // Compare the previous encoder values to the current ones, and stop
        // each wheel separately based on if it is below the deadband.
        if (this.leftFrontEncoder.get()
                - this.prevBrakeVals[0] < BRAKE_DEADBAND)
            brakeMotorVals[0] = 0;
        if (this.leftRearEncoder.get()
                - this.prevBrakeVals[1] < BRAKE_DEADBAND)
            brakeMotorVals[1] = 0;
        if (this.rightFrontEncoder.get()
                - this.prevBrakeVals[2] < BRAKE_DEADBAND)
            brakeMotorVals[2] = 0;
        if (this.rightRearEncoder.get()
                - this.prevBrakeVals[3] < BRAKE_DEADBAND)
            brakeMotorVals[3] = 0;

        // By the time that all motors are set to zero, we have finished
        // braking.
        if (brakeMotorVals[0] == 0 && brakeMotorVals[1] == 0
                && brakeMotorVals[2] == 0 && brakeMotorVals[3] == 0)
            {
            this.getTransmission().stop();
            brakeInit = true;
            return true;
            }

        // Store the values of the encoders for next time.
        this.prevBrakeVals[0] = this.leftFrontEncoder.get();
        this.prevBrakeVals[1] = this.leftRearEncoder.get();
        this.prevBrakeVals[2] = this.rightFrontEncoder.get();
        this.prevBrakeVals[3] = this.rightRearEncoder.get();

        // Reset the "Timer"
        this.previousBrakeTime = System.currentTimeMillis();
        }

    // If we have not finished braking, then set the motors to their
    // opposites.
    this.getTransmission().getSpeedController(MotorPosition.LEFT_FRONT)
            .set(-brakeMotorVals[0]);
    this.getTransmission().getSpeedController(MotorPosition.LEFT_REAR)
            .set(-brakeMotorVals[1]);
    this.getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT)
            .set(-brakeMotorVals[2]);
    this.getTransmission().getSpeedController(MotorPosition.RIGHT_REAR)
            .set(-brakeMotorVals[3]);

    return false;
}

private boolean brakeInit = true;

private long previousBrakeTime = 1;

private double[] brakeMotorVals =
    {0, 0, 0, 0};

private int[] prevBrakeVals =
    {1, 1, 1, 1};

/**
 * Drives the robot a certain distance without encoder correction.
 * Not using correction increases reliability but decreases precision.
 * If one encoder fails, it will instead look for other encoders for input.
 * 
 * @param distance
 *            how far the robot should travel. Should always remain positive!
 * @param speed
 *            how fast the robot should go while traveling. Negative for
 *            backwards.
 * @return whether or not the robot has reached "distance".
 */
public boolean driveInches (int distance, double speed)
{
    // Reset encoders on initialization.
    if (this.driveInchesInit == true)
        {
        this.resetEncoders();
        this.driveInchesInit = false;
        }

    // Test if ANY encoder is past the distance.
    if (this.isAnyEncoderLargerThan(distance) == true)
        {
        this.driveInchesInit = true;
        return true;
        }

    this.getTransmission().driveRaw(speed, speed);
    return false;
}

private boolean driveInchesInit = true;

/**
 * TODO test this!
 * Drives the robot a certain distance based on the encoder values.
 * If the robot should go backwards, set speed to be negative instead of
 * distance.
 * 
 * @param distance
 *            How far the robot should go (should be greater than 0)
 * @param speed
 *            How fast the robot should travel
 * @return Whether or not the robot has finished traveling that given distance.
 */
public boolean driveStraightInches (int distance, double speed,
        boolean willBrake)
{
    // Runs once when the method runs the first time, and does not run again
    // until after the method returns true.
    if (driveStraightInchesInit == true)
        {
        this.resetEncoders();
        driveStraightInchesInit = false;
        }

    // Check encoders to see if the distance has been driven
    if (this.transmissionType == TransmissionType.MECANUM
            || this.transmissionType == TransmissionType.TANK)
        {
        // Check all encoders if it is a four wheel drive system.
        if (this.getEncoderDistanceAverage(WheelGroups.ALL) > distance)
            {
            if (!willBrake)
                this.getTransmission().stop();
            driveStraightInchesInit = true;
            return true;
            }
        }
    else
        {
        // Only check the rear encoders if it is a two wheel drive system.
        if (this.getEncoderDistanceAverage(WheelGroups.REAR) > distance)
            {
            this.getTransmission().stop();
            driveStraightInchesInit = true;
            return true;
            }
        }

    // Drive straight if we have not reached the distance
    this.driveStraight(speed);

    return false;
}

private boolean driveStraightInchesInit = true;

/**
 * TODO Test this!
 * Drives the robot in a straight line based on encoders.
 * 
 * This works by polling the encoders every (COLLECTION_TIME) milliseconds
 * and then taking the difference from the last collection and using it as a
 * ratio to multiply times the speed.
 * 
 * This approach allows a more dynamic correction, as it only corrects as much
 * as
 * it needs to.
 * 
 * Remember: reset the encoders before running this method.
 * 
 * @param speed
 *            How fast the robot will be moving. Correction will be better with
 *            lower percentages.
 */
public void driveStraight (double speed)
{
    // Only check encoders if the right amount of time has elapsed
    // (collectionTime).
    if (System.currentTimeMillis() > driveStraightOldTime
            + COLLECTION_TIME)
        {
        // Reset the "timer"
        driveStraightOldTime = System.currentTimeMillis();
        // Only use the four encoders if the robot uses a four-wheel system
        if (transmissionType == TransmissionType.MECANUM
                || transmissionType == TransmissionType.TANK)
            {
            // Calculate how much has changed between the last collection
            // time and now
            leftChange = (leftFrontEncoder.get()
                    + leftRearEncoder.get()) - prevEncoderValues[0];
            rightChange = (rightFrontEncoder.get()
                    + rightRearEncoder.get()) - prevEncoderValues[1];
            // Setup the previous values for the next collection run
            prevEncoderValues[0] = leftFrontEncoder.get()
                    + leftRearEncoder.get();
            prevEncoderValues[1] = rightFrontEncoder.get()
                    + rightRearEncoder.get();
            }
        else
            {
            // Calculate how much has changed between the last collection
            // time and now
            leftChange = leftRearEncoder.get() - prevEncoderValues[0];
            rightChange = rightRearEncoder.get() - prevEncoderValues[1];
            // Setup the previous values for the next collection run
            prevEncoderValues[0] = leftRearEncoder.get();
            prevEncoderValues[1] = rightRearEncoder.get();
            }
        }
    // Changes how much the robot corrects by how off course it is. The
    // more off course, the more it will attempt to correct.
    this.getTransmission().driveRaw(
            speed * ((double) rightChange / leftChange),
            speed * ((double) leftChange / rightChange));

}

private int leftChange = 1, rightChange = 1;

private int[] prevEncoderValues =
    {1, 1};
// Preset to 1 to avoid divide by zero errors.

// Used for calculating how much time has passed for driveStraight
private long driveStraightOldTime = 0;

/**
 * TODO Test this!
 * Turns the robot to a certain angle using the robot's turning circle to find
 * the arc-length.
 * 
 * @param angle
 *            How far the robot should turn. Negative angle turns left, positive
 *            turns right. (In Degrees)
 * @param speed
 *            How fast the robot should turn (0 to 1.0)
 * @return Whether or not the robot has finished turning
 */
public boolean turnDegrees (int angle, double speed)
{
    // Only reset the encoders on the method's first start.
    if (turnDegreesInit == true)
        {
        this.resetEncoders();
        turnDegreesInit = false;
        }

    // Tests whether any encoder has driven the arc-length of the angle
    // (angle x radius)// took out +15 on Nov 4
    if (this.transmissionType == TransmissionType.MECANUM
            || this.transmissionType == TransmissionType.TANK)
        {
        // Only check 4 encoders if we have a four wheel drive system
        if (this.getEncoderDistanceAverage(
                WheelGroups.ALL) > Math.toRadians(Math.abs(angle))
                        * TURNING_RADIUS)
            {
            // We have finished turning!
            this.getTransmission().stop();
            turnDegreesInit = true;
            return true;
            }
        }
    else
        {
        // Only check 2 encoders if we have a two wheel drive system
        if (this.getEncoderDistanceAverage(
                WheelGroups.REAR) > Math.toRadians(Math.abs(angle))
                        * TURNING_RADIUS)
            {
            // We have finished turning!
            this.getTransmission().stop();
            turnDegreesInit = true;
            return true;
            }
        }

    // Change which way the robot turns based on whether the angle is
    // positive or negative
    if (angle < 0)
        {
        this.getTransmission().driveRaw(-speed, speed);
        }
    else
        {
        this.getTransmission().driveRaw(speed, -speed);
        }

    return false;
}

// variable to determine if it is the first time running a method
private boolean turnDegreesInit = true;

public boolean turnDegreesGyro (int degrees, double speed)
{
    if (turnDegreesGyroInit)
        {
        this.gyro.reset();
        turnDegreesGyroInit = false;
        }

    if (Math.abs(this.gyro.getAngle()) > Math.abs(degrees))
        {

        }

    return false;
}

private boolean turnDegreesGyroInit = true;

// TODO ANE create function to set ticks per encoder for all motors at
// once and one for individual motors, use an enum

// ================GAME SPECIFIC FUNCTIONS================
/*
 * Driving functions that change from game to game, such as using the camera
 * to score, etc.
 */
/**
 * Autonomously drives to the gear using the camera, and when we are too close
 * to see the targets, then switch to using the ultrasonic.
 * 
 * @param speed
 *            how fast the robot will move
 * @return whether or not the robot has finished the program.
 */
public boolean driveToGear (double speed)
{
    // If the ultrasonic reads that we are close enough to the wall, then
    // shut
    // off the motors and we are done.
    if (this.ultrasonic
            .getDistanceFromNearestBumper() < GEAR_AUTO_FINISH_DISTANCE)
        {
        this.driveToGearStatus = DriveToGearStatus.DRIVE_WITH_CAMERA;
        this.getTransmission().driveRaw(0.0, 0.0);
        // We are in place
        return true;
        }
    else if (this.ultrasonic
            .getDistanceFromNearestBumper() < GEAR_AUTO_CUTOFF_DISTANCE)
        {
        this.driveToGearStatus = DriveToGearStatus.DRIVE_STRAIGHT;
        }

    // Only drive with the camera if it's in the right state, and there are
    // at least 2 blobs available.
    this.visionProcessor.processImage();

    if (driveToGearStatus == DriveToGearStatus.DRIVE_WITH_CAMERA
            && visionProcessor.getParticleReports().length > 1)
        {
        // The average of the two blob's x values
        double averageCenter = (visionProcessor
                .getNthSizeBlob(0).center.x
                + visionProcessor.getNthSizeBlob(1).center.x) / 2.0;

        if (averageCenter > GEAR_CAMERA_CENTER - DRIVE_TO_GEAR_DEADBAND
                && averageCenter < GEAR_CAMERA_CENTER
                        + DRIVE_TO_GEAR_DEADBAND)
            {
            this.getTransmission().driveRaw(speed, speed);
            }
        else if (averageCenter > GEAR_CAMERA_CENTER)// Too far right?
            {
            this.getTransmission().driveRaw(
                    speed + DRIVE_CORRECTION_VALUE,
                    speed - DRIVE_CORRECTION_VALUE);
            }
        else // Too far left?
            {
            this.getTransmission().driveRaw(
                    speed - DRIVE_CORRECTION_VALUE,
                    speed + DRIVE_CORRECTION_VALUE);
            }
        }
    // Drives straight to the gear after the camera is done.
    else if (driveToGearStatus == DriveToGearStatus.DRIVE_STRAIGHT)
        {
        driveStraight(speed);
        }

    // Keep driving, we are not there yet.
    return false;
}

private DriveToGearStatus driveToGearStatus = DriveToGearStatus.DRIVE_WITH_CAMERA;

private enum DriveToGearStatus
    {
DRIVE_WITH_CAMERA, DRIVE_STRAIGHT
    }

// ================TUNABLES================

// Number of milliseconds that will pass before collecting data on encoders
// for driveStraight and brake
private static final int COLLECTION_TIME = 10;

// The change in encoder ticks where the robot is considered "stopped".
private static final int BRAKE_DEADBAND = 10;

// The distance from the left side wheel to the right-side wheel divided by
// 2, in inches. Used in turnDegrees.
// TODO find this value.
// Nov 4 changed from 16 to 17
private static final int TURNING_RADIUS = 16;

// Average between the two gear tape blobs
private static final int GEAR_CAMERA_CENTER = 80;

// The distance the camera will cutoff and switch to driving straight
private static final int GEAR_AUTO_CUTOFF_DISTANCE = 40;

private static final double GEAR_AUTO_FINISH_DISTANCE = 14;

private static final double DRIVE_CORRECTION_VALUE = .1;

private static final double DRIVE_TO_GEAR_DEADBAND = 8;// in pixels

}

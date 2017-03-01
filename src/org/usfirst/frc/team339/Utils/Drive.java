package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionFourWheel;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionMecanum;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class allows us to drive semi-autonomously, and is to be implemented
 * with a transmission using Tank or Mecanum drive, as well as a camera
 * and an image processor.
 * 
 * @author Ryan McGee
 *
 */
public class Drive
{
/*
 * We have both TransmissionMecanum and TransmissionFourWheel so we're able to
 * drive with either. The constructor detects which and then sets the
 * transmissionType enum so we know which we're using throughout the class, and
 * can use the appropriate code to drive our wheels.
 * 
 */
private TransmissionMecanum transmissionMecanum = null;

private TransmissionFourWheel transmissionFourWheel = null;

private ImageProcessor imageProcessor = null;

private Encoder rightFrontEncoder = null;

private Encoder rightRearEncoder = null;

private Encoder leftFrontEncoder = null;

private Encoder leftRearEncoder = null;

private boolean isUsingEncoders = false;

private boolean isUsingUltrasonics = false;

private UltraSonic ultrasonic = null;

private Timer timer = new Timer();

private double correction = 0.0;// TODO find out what this does.

// The amount the encoders are allowed to be off and considered "equal"
private double encoderSlack = 0.0;

// If true, we print out debugging info, else we do not.
private boolean debugStatus = false;

/**
 * Creates an instance of the Drive class, with a mecanum drive system.
 * If this is called, the mecanum versions of each method are used.
 * 
 * @param transmissionMecanum
 *            The transmission to be input
 * @param camera
 *            The camera we want to use for image saving
 * @param imageProcessor
 *            The processor we want to use for aiming and aligning
 */
public Drive (TransmissionMecanum transmissionMecanum,
        ImageProcessor imageProcessor)
{
    this.transmissionMecanum = transmissionMecanum;
    // We called the mecanum constructor, so set the drive class to use mecanum
    this.transmissionType = TransmissionType.MECANUM;
    this.imageProcessor = imageProcessor;
}

/**
 * Creates an instance of the Drive class, with a mecanum drive system.
 * If this is called, the mecanum versions of each method are used.
 * 
 * @param transmissionMecanum
 *            The transmission to be input
 * @param camera
 *            The camera we want to use for image saving
 * @param imageProcessor
 *            The processor we want to use for aiming and aligning
 * @param rightFrontEncoder
 *            The front right encoder
 * @param rightRearEncoder
 *            The back right encoder
 * @param leftFrontEncoder
 *            The front left encoder
 * @param leftRearEncoder
 *            The back left encoder
 * @param leftUlt
 *            The ultrasonic on the left side of the robot
 * @param rightUlt
 *            The ultrasonic on the right side of the robot
 */
public Drive (TransmissionMecanum transmissionMecanum,
        ImageProcessor imageProcessor,
        Encoder rightFrontEncoder, Encoder rightRearEncoder,
        Encoder leftFrontEncoder, Encoder leftRearEncoder,
        UltraSonic rightUlt)
{
    // set up the transmission and vision processor objects
    this(transmissionMecanum, imageProcessor);
    // Set up the encoders.
    this.initEncoders(leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder);
    // Save the ultrasonic object
    this.ultrasonic = rightUlt;
    // We have the ultrasonic, so tell the class that.
    isUsingUltrasonics = true;
}

/**
 * Creates an instance of the Drive class, with a tank drive system.
 * If this is called, the tank versions of each method are used.
 * 
 * @param transmissionFourWheel
 *            The transmission to be input
 * @param camera
 *            The camera we want to use for image saving
 * @param imageProcessor
 *            The processor we want to use for aiming and aligning
 */
public Drive (TransmissionFourWheel transmissionFourWheel,
        ImageProcessor imageProcessor)
{
    this.transmissionFourWheel = transmissionFourWheel;
    // We're calling the four wheel version, so tell the class.
    this.transmissionType = TransmissionType.TANK;
    this.imageProcessor = imageProcessor;
}

/**
 * Creates an instance of the Drive class, with a Tank drive system.
 * If this is called, the Tank versions of each method are used.
 * 
 * @param transmissionFourWheel
 *            The transmission to be input
 * @param camera
 *            The camera we want to use for image saving
 * @param imageProcessor
 *            The processor we want to use for aiming and aligning
 * @param leftFrontEncoder
 *            The front left encoder
 * @param rightFrontEncoder
 *            The front right encoder
 * @param leftRearEncoder
 *            The back left encoder
 * @param rightRearEncoder
 *            The back right encoder
 * @param rightUlt
 *            The ultrasonic on the right side of the robot
 * @param driveTimer
 *            TODO
 * @param leftUlt
 *            The ultrasonic on the left side of the robot
 */
public Drive (TransmissionFourWheel transmissionFourWheel,
        ImageProcessor imageProcessor,
        Encoder leftFrontEncoder, Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        UltraSonic rightUlt)
{
    // Init the four wheel transmission and the image processor.
    this(transmissionFourWheel, imageProcessor);
    // Set up the encoders
    this.initEncoders(leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder);
    // Set up the ultrasonic
    this.ultrasonic = rightUlt;
    // tell the class we're using the ultrasonics.
    isUsingUltrasonics = true;
}


/**
 * Initializes all the encoders. Is called in the constructor and can
 * be called outside the class.
 *
 * @param _leftFrontEncoder
 *            The front left encoder
 * @param _rightFrontEncoder
 *            The front right encoder
 * @param _leftRearEncoder
 *            The back left encoder
 * @param _rightRearEncoder
 *            The back right encoder
 */
public void initEncoders (Encoder _leftFrontEncoder,
        Encoder _rightFrontEncoder, Encoder _leftRearEncoder,
        Encoder _rightRearEncoder)
{
    // tell the class we have the encoders ready to go!
    isUsingEncoders = true;
    // Actually set up the local encoder objects
    this.leftFrontEncoder = _leftFrontEncoder;
    this.rightFrontEncoder = _rightFrontEncoder;
    this.leftRearEncoder = _leftRearEncoder;
    this.rightRearEncoder = _rightRearEncoder;
    // Set the encoder distance per pulse to a default value so we have it.
    // TODO call with correct value in robot init, possibly remove here
    this.setEncoderDistancePerPulse(DEFAULT_DISTANCE_PER_PULSE);
}

/**
 * Gets the averaged distance out of the four encoders
 * 
 * Takes the absolute value of the encoder values, so it's really the average of
 * the absolute values, and will always be positive.
 * 
 * @return averaged current distance of the encoders
 */
public double getAveragedEncoderValues ()
{
    return (Math.abs(this.getLeftFrontEncoderDistance())
            + Math.abs(this.getRightFrontEncoderDistance())
            + Math.abs(this.getLeftRearEncoderDistance())
            + Math.abs(this.getRightRearEncoderDistance())) / 4.0;
}

// TODO Test this
/**
 * Drives a distance given. To drive backwards, give negative speed, not
 * negative distance.
 * 
 * @param inches
 *            How far we want to go
 * @param speed
 *            How fast we want to go there
 * @return Whether or not we have finished driving yet
 */
public boolean driveInches (double inches, double speed)
{
    /*
     * If it's our first time running the method, reset everything and tell it
     * not to run this block again.
     */
    if (firstTimeDriveInches)
        {
        this.resetEncoders();
        firstTimeDriveInches = false;
        }

    // If we've traveled beyond our target value.
    if (Math.abs(this.getAveragedEncoderValues()) >= Math.abs(inches))
        {
        // Stop
        this.drive(0.0, 0.0);
        // Prepare for next run
        firstTimeDriveInches = true;
        // Tell the caller we're done
        return true;
        }
    // Drive forward.
    this.drive(speed, 0.0);
    // tell the caller we're not done.
    return false;
}

private boolean firstTimeDriveInches = true;

/**
 * Method takes deltas of each side, and if they aren't equal, compensate by
 * making the other side go faster
 * 
 * @param inches
 *            How far we want to go, in inches
 * @param speed
 *            How fast we want to go
 * @return True if we've reached our target distance, false otherwise.
 * @author Becky Button
 * 
 */
public boolean driveStraightInches (final double inches,
        final double speed)
{
    // If it's the first time we're running
    if (firstTimeDriveInches == true)
        {
        // Reset the encoders (no duh)
        this.resetEncoders();
        // Tell the method not to run again
        firstTimeDriveInches = false;
        }
    // If we've gone beyond our target distance.
    if (this.getAveragedEncoderValues() >= inches)
        {
        // if we want debug info out.
        if (this.getDebugStatus() == true)
            {
            // Tell the programmer we're done
            System.out.println("We are finished driving straight");
            }
        // Stop
        this.driveNoDeadband(0.0, 0.0);
        // Prepare for setup again
        this.firstTimeDriveInches = true;
        // Tell the caller we're done.
        return true;
        }
    // Calculate the average value of the left and right sides of the drive
    // train.
    double averageLeft = (this.getLeftFrontEncoderDistance()
            + this.getLeftRearEncoderDistance()) / 2;
    double averageRight = (this.getRightFrontEncoderDistance()
            + this.getRightRearEncoderDistance()) / 2;
    // If we're printing debug info
    if (this.getDebugStatus() == true)
        {
        System.out.println(
                "Average Encoder Values"
                        + this.getAveragedEncoderValues());
        System.out.println("Average Left: " + averageLeft);
        System.out.println("Average Right: " + averageRight);
        System.out.println("we correcting");
        }
    // If we're within our error range
    if (Math.abs(averageRight - averageLeft) <= this
            .getEncoderSlack())
        {
        // drive straight
        this.driveNoDeadband(speed, 0);
        }
    // if we're outside our error range and the left is ahead of the right.
    else if (averageLeft > averageRight)// TODO this is wrong, fix.
        {
        // correct to the right (TODO I think this is also wrong)
        this.driveNoDeadband(speed, -this.getDriveCorrection());
        }
    // if we're outside our error range and the right is ahead of the left.
    else if (averageLeft < averageRight)
        {
        // correct to the left (TODO I think this is also wrong)
        this.driveNoDeadband(speed, this.getDriveCorrection());
        }
    // Tell the caller we're not done.
    return false;
}


/**
 * Aligns to the low dual targets for the gear peg. This finds the
 * average of the two targets, divides by the resolution of the camera
 * (to make it relative coordinates) and compares it to the given
 * value defined as relativeCenter.
 * 
 * @author Ryan McGee
 * 
 * @param relativeCenter
 *            The "center" of the camera, the value we want to align to.
 * @param movementSpeed
 *            The speed we want the motors to run at
 * @param deadband
 *            The "happy" value; the method will say "I am aligned!" when we are
 *            in this range.
 * @return Whether or not we are aligned to the center yet.
 */
public AlignReturnType alignToGear (final double relativeCenter,
        final double movementSpeed,
        final double deadband)
{
    if (isTurning == false)
        {
        // Process the an image from the camera so we know where we are.
        this.imageProcessor.processImage();
        // If we don't have two blobs...
        if (this.imageProcessor.getNthSizeBlob(1) == null)
            {
            // Stop
            this.drive(0.0, 0.0);
            // Tell the caller we lost at least one of our blobs.
            return AlignReturnType.NO_BLOBS;
            }
        // Find the distance from the center of the image of a combination of
        // the blobs.
        double distanceToCenter = imageProcessor
                .getPositionOfRobotToGear(
                        imageProcessor
                                .getNthSizeBlob(0),
                        imageProcessor
                                .getNthSizeBlob(1),
                        relativeCenter);
        // If the distance to center is the default value, we've lost our blobs.
        if (distanceToCenter == Double.MAX_VALUE)
            {
            // Stop
            this.drive(0.0, 0.0);
            // Tell the caller we don't blobs
            return AlignReturnType.NO_BLOBS;
            }

        // If we're were we want to be
        if (Math.abs(distanceToCenter) <= deadband)
            {
            // Stop
            this.drive(0.0, 0.0);
            // Tell the caller we're aligned
            return AlignReturnType.ALIGNED;
            }
        }
    // If we've turned to the goal, we're done, set up the next call as such.
    this.isTurning = this
            .turnDegrees(-Math.toDegrees(this.imageProcessor
                    .getYawAngleToTarget(this.imageProcessor
                            .getNthSizeBlob(0))
                    + this.imageProcessor
                            .getYawAngleToTarget(this.imageProcessor
                                    .getNthSizeBlob(1)))
                    / 2.0);
    // We're not aligned, tell the caller as such.
    return AlignReturnType.MISALIGNED;
}

private boolean isTurning = false;

/**
 * Aligns to the gear peg WHILE driving towards it. If the transmission type
 * is Mecanum, it will STRAFE while moving forwards. If it is tank drive
 * mode, it will adjust each side's speed accordingly.
 * 
 * @param driveSpeed
 *            The speed we will drive forwards
 * @param alignVar
 *            If it is tank drive, this will be the speed it aligns.
 *            If it is in mecanum drive, this is the angle added on.
 * @param deadband
 *            If the camera's center is in this deadband, we are 'aligned'.
 * @param relativeCenter
 *            Where we want the center of the robot to be (in RELATIVE
 *            coordinates)
 * @param distanceToTarget
 *            What we want the distance to the wall from the bumper
 *            to be when we stop aligning, in inches
 * @return Whether or not we are aligned, close enough, misaligned, or see no
 *         blobs.
 */
// Alex should comment his code. This isn't mine
public AlignReturnType strafeToGear (double driveSpeed,
        double alignVar, double deadband, double relativeCenter,
        int distanceToTarget)
{
    // If this is our first call.
    if (this.firstStrafe)
        {
        this.timer.reset();
        this.timer.start();
        this.firstStrafe = false;
        }
    /*
     * if (this.purgingUltrasonic)
     * {
     * this.rightUlt.getDistanceFromNearestBumper();
     * if (this.timer.get() >= .25)
     * {
     * this.timer.stop();
     * this.purgingUltrasonic = false;
     * }
     * else
     * {
     * this.drive(0.0, 0.0);
     * return AlignReturnType.WAITING;
     * }
     * }
     */
    this.imageProcessor.processImage();

    // If we have no blobs, return so.
    if (this.imageProcessor.getNthSizeBlob(1) == null)
        {
        this.driveNoDeadband(0.0, 0.0);
        return AlignReturnType.NO_BLOBS;
        }
    // If we don't have any ultrasonics in the constructor, stop aligning.
    if (this.isUsingUltrasonics == false)
        {
        this.driveNoDeadband(0.0, 0.0);
        return AlignReturnType.ALIGNED;
        }
    double distanceToCenter = this.imageProcessor
            .getPositionOfRobotToGear(
                    this.imageProcessor.getNthSizeBlob(0),
                    this.imageProcessor.getNthSizeBlob(1),
                    relativeCenter);
    System.out.println("DistanceToCenter: " + distanceToCenter);
    System.out.println("Distance from wall: "
            + this.ultrasonic.getDistanceFromNearestBumper());
    if (distanceToCenter == Double.MAX_VALUE)
        {
        this.driveNoDeadband(0.0, 0.0);
        return AlignReturnType.NO_BLOBS;
        }

    if (this.ultrasonic
            .getDistanceFromNearestBumper() <= distanceToTarget)
        {
        System.out.println("distance from nearest bumper: "
                + this.ultrasonic.getDistanceFromNearestBumper());
        System.out.println("distance to target: " + distanceToTarget);
        this.purgingUltrasonic = true;
        this.firstStrafe = true;
        this.driveNoDeadband(0.0, 0.0);
        return AlignReturnType.CLOSE_ENOUGH;
        }
    if (Math.abs(distanceToCenter) < deadband)
        {
        this.driveNoDeadband(driveSpeed, 0);
        return AlignReturnType.ALIGNED;
        }
    System.out.println("distanceToCenter: " + distanceToCenter);
    System.out.println("relative center: " + relativeCenter);
    if (distanceToCenter < 0)
        {
        System.out.println("trying to adjust left");
        // TODO Magic Numbers
        this.driveNoDeadband(driveSpeed + .3, -alignVar);// TODO nasty hack
        }
    else if (distanceToCenter > 0)
        {
        System.out.println("trying to adjust right");
        this.driveNoDeadband(driveSpeed + .3, alignVar);
        }
    return AlignReturnType.MISALIGNED;
}

private boolean purgingUltrasonic = true;

private boolean firstStrafe = true;


/**
 * Tells us how any aligning method returned.
 * 
 * @author Ryan McGee
 *
 */
public static enum AlignReturnType
    {
    /**
     * No blobs are present
     */
    NO_BLOBS,
    /**
     * We are now aligned with the target
     */
    ALIGNED,
    /**
     * We are not aligned with the target, keep aligning
     */
    MISALIGNED,
    /**
     * Only used if we are using an ultrasonic
     */
    CLOSE_ENOUGH,
    /**
     * We are waiting for the ultrasonic to purge bad values
     * before starting
     */
    WAITING


    }

/**
 * Positive number will make robot strafe right
 * 
 * @param inches
 */
/**
 * Drives WITH rotation. If we are using tank drive, it only turns based
 * on correction.
 * 
 * @param speed
 * @param correction
 * @param rotation
 */
public void drive (double speed, double correction, double rotation)
{
    switch (this.transmissionType)
        {
        case MECANUM:
            this.transmissionMecanum.drive(speed, correction, rotation);
            break;
        case TANK:
            this.transmissionFourWheel.driveWithoutCorrection(
                    speed + correction, speed - correction);
        default:
            break;
        }
}

/**
 * Drives based on correction speed and driving speed
 * 
 * @param speed
 *            how fast we drive and which direction (+forwards, -backwards)
 * @param correction
 *            how fast we correct, and which direction (+turn right, -turn left)
 */
public void drive (double speed, double correction)
{
    switch (this.transmissionType)
        {
        case MECANUM:
            this.transmissionMecanum.drive(speed, correction, 0.0);
            break;
        case TANK:
            this.transmissionFourWheel.driveWithoutCorrection(
                    speed + correction, speed - correction);
        default:
            break;
        }
}

// TODO comment
public void driveNoDeadband (double speed, double correction,
        double rotation)
{
    switch (this.transmissionType)
        {
        case MECANUM:
            this.transmissionMecanum.driveNoDeadband(speed, correction,
                    rotation);
            break;
        case TANK:
            this.transmissionFourWheel.driveWithoutCorrection(
                    speed + correction, speed - correction);
        default:
            break;
        }
}

public void driveNoDeadband (double speed, double correction)
{
    switch (this.transmissionType)
        {
        case MECANUM:
            System.out.println("using mecanum");
            this.transmissionMecanum.driveNoDeadband(speed, correction,
                    0.0);

            break;
        case TANK:
            System.out.println("using 4 wheel");
            this.transmissionFourWheel.driveWithoutCorrection(
                    speed + correction, speed - correction);
        default:
            break;
        }
}

/**
 * Linearly and continuously accelerate to <em> targetSpeed</em> over
 * <em>timeInWhichToAccelerate</em> seconds.
 * 
 * @param targetSpeed
 *            The final speed of the acceleration.
 * @param timeInWhichToAccelerate
 *            The number of seconds to spend accelerating.
 * @return
 *         True if we're done accelerating, false otherwise.
 */
public boolean accelerate (double targetSpeed,
        double timeInWhichToAccelerate)
{
    if (firstTimeAccelerateRun)
        {
        this.timer.stop();
        this.timer.reset();
        this.timer.start();
        firstTimeAccelerateRun = false;
        }
    this.driveNoDeadband(
            Math.max(targetSpeed
                    * (this.timer.get() / timeInWhichToAccelerate),
                    Max),
            0, 0);
    if (this.timer.get() > timeInWhichToAccelerate)
        {
        firstTimeAccelerateRun = true;
        return true;
        }
    return false;
}

private boolean firstTimeAccelerateRun = true;

private double savedDeadband = 0.0;

/**
 * @return the distance the front left encoder has driven based on the
 *         distance per pulse set earlier.
 */
public double getLeftFrontEncoderDistance ()
{
    return this.leftFrontEncoder.getDistance();
}

/**
 * @return the distance the front right encoder has driven based on the
 *         distance per pulse set earlier.
 */
public double getRightFrontEncoderDistance ()
{
    return this.rightFrontEncoder.getDistance();
}

/**
 * @return the distance the back left encoder has driven based on the
 *         distance per pulse set earlier.
 */
public double getLeftRearEncoderDistance ()
{
    return this.leftRearEncoder.getDistance();
}

/**
 * @return the distance the back rear encoder has driven based on the
 *         distance per pulse set earlier.
 */
public double getRightRearEncoderDistance ()
{
    return this.rightRearEncoder.getDistance();
}

/**
 * Sets the value multiplied by to get an accurate distance we have driven
 * 
 * @param value
 *            Distance per pulse
 */
public void setEncoderDistancePerPulse (double value)
{
    this.leftFrontEncoder.setDistancePerPulse(value);
    this.rightFrontEncoder.setDistancePerPulse(value);
    this.leftRearEncoder.setDistancePerPulse(value);
    this.rightRearEncoder.setDistancePerPulse(value);
}

/**
 * method reads distance per pulse
 * 
 * @param encoder
 * @return Distance per pulse
 */
public double getEncoderDistancePulse (Encoder encoder)
{
    return encoder.getRate();
}

/**
 * Resets all encoders back to 0 'distance units' driven
 */
public void resetEncoders ()
{
    this.leftFrontEncoder.reset();
    this.rightFrontEncoder.reset();
    this.leftRearEncoder.reset();
    this.rightRearEncoder.reset();
}

/**
 * Brakes for the given iterations
 * 
 * @param iterations
 *            the number of times we loop through this brake function
 * @param speed
 *            how fast we should brake. Negative if we want to brake after going
 *            forwards
 * 
 * @return true only if we have finished braking, false otherwise
 */
public boolean brake (int iterations, double speed)
{
    if (brakeIterations < iterations)
        {
        this.driveNoDeadband(speed, 0.0);
        brakeIterations++;
        return false;
        }
    this.drive(0.0, 0.0);
    brakeIterations = 0;
    return true;

}

private int brakeIterations = 0;

/**
 * brakes until we have stopped, then sets motors to zero
 * 
 * @param speed
 *            how fast we should brake, negate for forward
 * @return true if we have stopped completely
 */
public boolean timeBrake (double speed, double time)// COMMENT THIS! | I got chu
                                                    // fam.
{
    double alteredSpeed = Math.abs(speed);

    System.out.println("brake timer:" + movementTimer.get());
    if (firstTimeTimeBrake)
        {
        // For each speed controller, if they're going backwards (is less than
        // 0), tell them to go forwards, otherwise, tell them to go backwards
        // when we do the actual breaking.

        // Left Front speed controller sign
        if (this.transmissionMecanum.leftSpeedController
                .get() < 0)
            {
            this.motorSigns[0] = 1;
            }
        else
            {
            this.motorSigns[0] = -1;
            }

        // Left Rear speed controller sign
        if (this.transmissionMecanum.leftRearSpeedController
                .get() < 0)
            {
            this.motorSigns[1] = 1;
            }
        else
            {
            this.motorSigns[1] = -1;
            }

        // Right Front Speed Controller sign
        if (this.transmissionMecanum.rightSpeedController
                .get() < 0)
            {
            this.motorSigns[2] = 1;
            }
        else
            {
            this.motorSigns[2] = -1;
            }

        // Right Rear Speed Controller Sign
        if (this.transmissionMecanum.rightRearSpeedController
                .get() < 0)
            {
            this.motorSigns[3] = 1;
            }
        else
            {
            this.motorSigns[3] = -1;
            }

        // Setup the timer
        movementTimer.stop();
        movementTimer.reset();
        movementTimer.start();
        // Don't set up again this run
        firstTimeTimeBrake = false;
        }
    // if We're done breaking
    if (movementTimer.get() >= time)
        {
        // stop
        this.driveNoDeadband(0.0, 0.0);
        // tell the user
        System.out.println("We are at zero");
        // stop the timer
        movementTimer.stop();
        // be ready to set up again
        firstTimeTimeBrake = true;
        // Tell the caller we're done
        return true;
        }
    // otherwise...
    /*
     * Drive the motors in the opposite direction they were running when the
     * function was first called. Uses the motorSigns array to find out which
     * way that was.
     */
    this.transmissionMecanum
            .driveLeftMotor(alteredSpeed * this.motorSigns[0]);

    this.transmissionMecanum
            .driveLeftRearMotor(alteredSpeed * this.motorSigns[1]);

    this.transmissionMecanum
            .driveRightMotor(alteredSpeed * this.motorSigns[2]);

    this.transmissionMecanum
            .driveRightRearMotor(alteredSpeed * this.motorSigns[3]);
    // Tell the user we're not done
    System.out.println("We are not at zero");
    // Tell the caller we're not done
    return false;

}

/**
 * left front, left rear, right front, right rear
 */
private int[] motorSigns =
    {1, 1, 1, 1};

private boolean firstTimeTimeBrake = true;

// public double MAX_TIME = .1;// TODO final?

Timer movementTimer = new Timer();

/**
 * Brakes based on the encoders, and whether or not they are reading within the
 * deadband.
 * 
 * @author Ryan McGee
 * 
 * @param voltage
 *            How fast we want to stop. Based on how fast we are going now.
 * @return Whether or not we are finished braking.
 */
public boolean brakeToZero (double voltage)
{
    if (firstBrakeToZero)
        {
        this.resetEncoders();
        firstBrakeToZero = false;
        }

    if (!this.brakeEachWheel[0] && !this.brakeEachWheel[1]
            && !this.brakeEachWheel[2] && !this.brakeEachWheel[3])
        {
        this.firstBrakeToZero = true;
        this.brakeEachWheel = new boolean[]
            {true, true, true, true};
        return true;
        }

    // FOR ALL OF THIS NONSENSE BELOW:
    // we check if the change in encoder distances FOR EACH MOTOR
    // is less than the deadband. FOR EACH MOTOR if it is bigger than the
    // deadband, it runs the motor the reverse of the sign of the delta,
    // at the absolute value of the voltage input.

    if (Math.abs(this.getLeftFrontEncoderDistance()
            - this.lastBrakeValues[0]) < this.BRAKE_DEADBAND)
        this.brakeEachWheel[0] = false;

    if (Math.abs(this.getLeftRearEncoderDistance()
            - this.lastBrakeValues[1]) < this.BRAKE_DEADBAND)
        this.brakeEachWheel[1] = false;

    if (Math.abs(this.getRightFrontEncoderDistance()
            - this.lastBrakeValues[2]) < this.BRAKE_DEADBAND)
        this.brakeEachWheel[2] = false;

    if (Math.abs(this.getRightRearEncoderDistance()
            - this.lastBrakeValues[3]) < this.BRAKE_DEADBAND)
        this.brakeEachWheel[3] = false;

    // Braking code

    // Left Front brake
    if (this.brakeEachWheel[0] && this.getLeftFrontEncoderDistance()
            - this.lastBrakeValues[0] < 0)
        {
        this.transmissionMecanum.driveLeftMotor(
                Math.abs(voltage));
        }
    else
        {
        this.transmissionMecanum.driveLeftMotor(
                Math.abs(-voltage));
        }

    // Left Rear brake
    if (this.brakeEachWheel[1] && this.getLeftRearEncoderDistance()
            - this.lastBrakeValues[1] < 0)
        {
        this.transmissionMecanum.driveLeftRearMotor(
                Math.abs(voltage));
        }
    else
        {
        this.transmissionMecanum.driveLeftRearMotor(
                Math.abs(-voltage));
        }

    // Right Front brake
    if (this.brakeEachWheel[2] && this.getRightFrontEncoderDistance()
            - this.lastBrakeValues[2] < 0)
        {
        this.transmissionMecanum.driveRightMotor(Math.abs(voltage));
        }
    else
        {
        this.transmissionMecanum.driveRightMotor(Math.abs(-voltage));
        }

    // Right Rear brake
    if (this.brakeEachWheel[3] && this.getRightRearEncoderDistance()
            - this.lastBrakeValues[3] < 0)
        {
        this.transmissionMecanum
                .driveRightRearMotor(Math.abs(voltage));
        }
    else
        {
        this.transmissionMecanum
                .driveRightRearMotor(Math.abs(-voltage));
        }

    this.lastBrakeValues[0] = this.getLeftFrontEncoderDistance();
    this.lastBrakeValues[1] = this.getLeftRearEncoderDistance();
    this.lastBrakeValues[2] = this.getRightFrontEncoderDistance();
    this.lastBrakeValues[3] = this.getRightRearEncoderDistance();

    return false;
}

private boolean firstBrakeToZero = true;

private double[] lastBrakeValues =
    {0, 0, 0, 0};

private boolean[] brakeEachWheel =
    {true, true, true, true};

private boolean checkBrakeAgain = false;


/**
 * @method isStopped
 * @return Returns false if chosen encoders are not stopped
 * @author Eli Neagle
 * @author ASHLEY ESPELAND FRICKIN COMMENTED ALL THIS CODE I DESERVE
 *         SOME CREDIT
 * @param leftEncoder
 * @param rightEncoder
 * @written Sep 1, 2016
 */

// TODO stop, stop while turning, stop on a hill, get out of stop function past
// a certain time
public boolean isStopped (Encoder leftEncoder, Encoder rightEncoder)
{

    // if the difference between the current encoder value and the encoder
    // value from the last time we called this function is equal to 0
    if (Math.abs(leftEncoder.getDistance())
            - this.brakePreviousDistanceL == 0
            && Math.abs(rightEncoder.getDistance())
                    - this.brakePreviousDistanceR == 0)
        {
        System.out.println(
                "Left Delta: " + (Math.abs(leftEncoder.getDistance())
                        - this.brakePreviousDistanceL));
        System.out.println(
                "Right Delta: " + (Math.abs(rightEncoder.getDistance())
                        - this.brakePreviousDistanceR));
        // then set the brakePreviousDistance to 0, and return true
        this.brakePreviousDistanceL = 0;
        this.brakePreviousDistanceR = 0;
        return true;
        }
    // else set brakePreviousDistance to the current encoder value,
    // and return false
    brakePreviousDistanceL = Math.abs(leftEncoder.getDistance());
    brakePreviousDistanceR = Math.abs(rightEncoder.getDistance());

    return false;
}

public boolean directionChanged (Encoder leftEncoder,
        Encoder rightEncoder)
{
    // if /Encoder/ - distance from previous call is less than 0
    //
    if (Math.abs(leftEncoder.getDistance())
            - this.brakePreviousDistanceL < 0
            && Math.abs(rightEncoder.getDistance())
                    - this.brakePreviousDistanceR < 0)
        {
        // set brakePreviousDistance to 0 and return true
        this.brakePreviousDistanceL = 0;
        this.brakePreviousDistanceR = 0;
        return true;
        }
    else
        {
        // set brakePreviousDistance to the encoder value
        brakePreviousDistanceL = Math.abs(leftEncoder.getDistance());
        brakePreviousDistanceR = Math.abs(rightEncoder.getDistance());
        return false;
        }
}

private double brakePreviousDistanceL = 0.0;

private double brakePreviousPreviousDistanceL = 0.0;

private double brakePreviousDistanceR = 0.0;

private double brakePreviousPreviousDistanceR = 0.0;

// distance we've moved in two reads where we consider ourselves braked in
// the brake() method.
private final double AUTO_ENCODER_THRESHOLD_INCHES = 0.25;


/**
 * set the dead-band range for the joystick(s)
 *
 * @method setAllGearLightsOff
 * @param percentage
 *            the percentage of the range that we want to make
 *            into a dead-band The range is 0.0 - 1.0
 * @return boolean - denotes whether or not the dead-band range was set
 * @author Bob Brown
 * @written Sep 20, 2009
 *          -------------------------------------------------------
 */
public boolean setJoystickDeadbandRange (double percentage)
{
    if ((percentage >= 0) && (percentage <= 1.0))
        {
        this.deadbandPercentageZone = percentage;
        return (true);
        } // end if
    return (false);
} // setJoystickDeadbandRange

private double deadbandPercentageZone = 0.0;
/// **
// *
// * @param lBrakeSpeed
// * @param rBrakeSpeed
// * @return have we finished true = yes
// */
// public boolean brake (final double lBrakeSpeed,
// final double rBrakeSpeed)
// {
// if (((Math.abs(
// leftFrontEncoder.getDistance()) >= (this.prevBrakeDistanceLF
// * this.encoderFactorUpperRange)
// - this.ENCODER_THRESHOLD
// && Math.abs(leftFrontEncoder
// .getDistance()) <= (this.prevBrakeDistanceLF
// * this.encoderFactorLowerRange)
// + ENCODER_THRESHOLD)))
// {
// withinLeftFrontRange = true;
// }
// if (((Math.abs(
// leftRearEncoder.getDistance()) >= (this.prevBrakeDistanceLR
// * this.encoderFactorUpperRange)
// - this.ENCODER_THRESHOLD
// && Math.abs(leftRearEncoder
// .getDistance()) <= (this.prevBrakeDistanceLR
// * this.encoderFactorLowerRange)
// + ENCODER_THRESHOLD)))
// {
// withinLeftRearRange = true;
// }
// if (((Math.abs(
// rightFrontEncoder
// .getDistance()) >= (this.prevBrakeDistanceRF
// * this.encoderFactorUpperRange)
// - this.ENCODER_THRESHOLD
// && Math.abs(rightFrontEncoder
// .getDistance()) <= (this.prevBrakeDistanceRF
// * this.encoderFactorLowerRange)
// + ENCODER_THRESHOLD)))
// {
// withinRightFrontRange = true;
// }
// if (((Math.abs(
// rightRearEncoder.getDistance()) >= (this.prevBrakeDistanceRR
// * this.encoderFactorUpperRange)
// - this.ENCODER_THRESHOLD
// && Math.abs(rightRearEncoder
// .getDistance()) <= (this.prevBrakeDistanceRR
// * this.encoderFactorLowerRange)
// + ENCODER_THRESHOLD)))
// {
// withinRightRearRange = true;
// }
//
//
// if (transmissionType == TransmissionType.MECANUM)
// {
// this.transmissionMecanum.drive(lBrakeSpeed, 0, 0);
// }
// if (transmissionType == TransmissionType.TANK)
// {
// this.transmissionFourWheel.drive(rBrakeSpeed, lBrakeSpeed);
// }
//
//
// this.prevBrakeDistanceLF = leftFrontEncoder.getDistance();
// this.prevBrakeDistanceLR = leftRearEncoder.getDistance();
// this.prevBrakeDistanceRF = rightFrontEncoder.getDistance();
// this.prevBrakeDistanceRR = rightRearEncoder.getDistance();
//
// if (withinRightRearRange && withinRightFrontRange
// && withinLeftFrontRange && withinLeftRearRange)
// {
// this.transmissionFourWheel.drive(0, 0);
// return true;
// }
//
// return false;
//
// }
//
// private double prevBrakeDistanceRR = 0;
// private double prevBrakeDistanceRF = 0;
// private double prevBrakeDistanceLR = 0;
// private double prevBrakeDistanceLF = 0;
//
//// private double prevPrevBrakeDistanceRR = 0;
//// private double prevPrevBrakeDistanceRF = 0;
//// private double prevPrevBrakeDistanceLR = 0;
//// private double prevPrevBrakeDistanceLF = 0;
//
// private boolean withinRightFrontRange = false;
// private boolean withinRightRearRange = false;
// private boolean withinLeftFrontRange = false;
// private boolean withinLeftRearRange = false;
//
// final private double encoderFactorUpperRange = .5;
// final private double encoderFactorLowerRange = .3;
//
// private double ENCODER_THRESHOLD = .25;

/**
 * Rotates the robot by degrees.
 * 
 * @param degrees
 *            Number of degrees to rotate by. Negative if left, positive if
 *            right.
 * @param speed
 *            How fast we want the robot to turn
 * @return Whether or not we have finished turning yet.
 */
public boolean turnDegrees (double degrees, double speed)
{
    if (firstAlign)
        {
        this.resetEncoders();
        this.firstAlign = false;
        }
    // We do not know why, but the robot by default turns opposite what we want.
    double adjustedDegrees = -degrees;

    if (!isUsingEncoders)
        {
        this.firstAlign = true;
        return true;
        }
    double angleInRadians = Math.toRadians(Math.abs(degrees));

    double leftSideAverage = Math
            .abs(this.getLeftFrontEncoderDistance()
                    + this.getLeftRearEncoderDistance())
            / 2.0;

    double rightSideAverage = Math
            .abs(this.getRightFrontEncoderDistance()
                    + this.getRightRearEncoderDistance())
            / 2.0;

    // If the arc length is equal to the amount driven, we finish
    if (rightSideAverage >= angleInRadians * turningCircleRadius
            || leftSideAverage >= angleInRadians * turningCircleRadius)
        {
        this.firstAlign = true;
        return true;
        }

    // Rotate if not
    if (adjustedDegrees < 0)
        {
        if (transmissionType == TransmissionType.TANK)
            transmissionFourWheel.drive(speed, -speed);
        else
            transmissionMecanum.drive(0.0, 0.0, -speed);
        }
    else if (adjustedDegrees > 0)
        {
        if (transmissionType == TransmissionType.TANK)
            transmissionFourWheel.drive(-speed, speed);
        else
            transmissionMecanum.drive(0.0, 0.0, speed);
        }
    return false;
}

/**
 * Method to turn degrees using the default or 'set' speed in this class
 * 
 * @param degrees
 *            How far we want to turn
 * @return
 */
public boolean turnDegrees (double degrees)
{
    return this.turnDegrees(degrees, rotateSpeed);
}

private double rotateSpeed = .6;

/**
 * Gets how fast we are rotating in turnDegrees
 * 
 * @return rotation speed
 */
public double getRotateSpeed ()
{
    return this.rotateSpeed;
}

/**
 * Sets how fast we should rotate in turnDegrees
 * 
 * @param speed
 *            rotation speed
 */
public void setRotateSpeed (double speed)
{
    this.rotateSpeed = speed;
}


private double turningCircleRadius = 11;


/**
 * Gets the radius of the circle that the robot rotates around
 * 
 * @return Radius in inches
 */
public double getTurningCircleRadius ()
{
    return turningCircleRadius;
}

/**
 * Sets the radius of the circle that the robot rotates around
 * 
 * @param radius
 *            Radius in inches
 */
public void setTurningCircleRadius (double radius)
{
    this.turningCircleRadius = radius;
}

private boolean firstAlign = true;

public void setEncoderSlack (double slack)
{
    this.encoderSlack = slack;
}

public double getEncoderSlack ()
{
    return this.encoderSlack;
}

public void setDriveCorrection (double correction)
{
    this.correction = correction;
}

public double getDriveCorrection ()
{
    return this.correction;
}

public void setDebugStatus (boolean debug)
{
    this.debugStatus = debug;
}

public boolean getDebugStatus ()
{
    return this.debugStatus;
}


/**
 * The type of transmission we are using. Is set in the constructor.
 * 
 * @author Ryan McGee
 *
 */
public static enum TransmissionType
    {
    /**
     * Tank drive
     */
    TANK,
    /**
     * Mecanum drive
     */
    MECANUM
    }

// =====================================================================
// Constants
// =====================================================================
private TransmissionType transmissionType = null;

private final double BRAKE_DEADBAND = 0.1;

/**
 * The value that the getDistance is multiplied by to get an accurate
 * distance.
 */
private static final double DEFAULT_DISTANCE_PER_PULSE = .069;// 1.0 / 12.9375;

private static final double Max = 0.3;


}

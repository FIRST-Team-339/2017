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

private TransmissionMecanum transmissionMecanum = null;

private TransmissionFourWheel transmissionFourWheel = null;

private ImageProcessor imageProcessor = null;

private Encoder rightFrontEncoder = null;

private Encoder rightRearEncoder = null;

private Encoder leftFrontEncoder = null;

private Encoder leftRearEncoder = null;

private boolean isUsingEncoders = false;

private boolean isUsingUltrasonics = false;

private UltraSonic rightUlt = null;

private Timer timer = new Timer();

public double correction = 0.0;

public double encoderSlack = 0.0;

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
    this(transmissionMecanum, imageProcessor);

    this.initEncoders(leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder);

    this.rightUlt = rightUlt;
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
    this(transmissionFourWheel, imageProcessor);
    this.initEncoders(leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder);
    this.rightUlt = rightUlt;

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
    isUsingEncoders = true;

    this.leftFrontEncoder = _leftFrontEncoder;
    this.rightFrontEncoder = _rightFrontEncoder;
    this.leftRearEncoder = _leftRearEncoder;
    this.rightRearEncoder = _rightRearEncoder;

    this.setEncoderDistancePerPulse(DEFAULT_DISTANCE_PER_PULSE);
}

// public double encoderRate (Encoder _leftFrontEncoder,
// Encoder _rightFrontEncoder, Encoder _leftRearEncoder,
// Encoder _rightRearEncoder)
// {
//
// return leftFrontRate = this.leftFrontEncoder.getRate();
// return rightFrontRate = this.rightFrontEncoder.getRate();
// return leftRearRate = this.leftRearEncoder.getRate();
// return rightRearRate = this.rightRearEncoder.getRate();
//
// }
//
// private double leftFrontRate = 0.0;
// private double rightFrontRate = 0.0;
// private double leftRearRate = 0.0;
// private double rightRearRate = 0.0;

/**
 * Gets the averaged distance out of the four encoders
 * 
 * @return averaged distance after driving forwards
 */
public double getAveragedEncoderValues ()
{
    return (this.getLeftFrontEncoderDistance()
            + this.getRightFrontEncoderDistance()
            + this.getLeftRearEncoderDistance()
            + this.getRightRearEncoderDistance()) / 4.0;
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

    if (firstTimeDriveInches)
        {
        this.resetEncoders();
        firstTimeDriveInches = false;
        }
    System.out.println(this.getAveragedEncoderValues());
    if (Math.abs(this.getAveragedEncoderValues()) >= Math.abs(inches))
        {
        this.drive(0.0, 0.0);
        firstTimeDriveInches = true;
        return true;
        }
    this.drive(speed, 0.0);

    return false;
}

private boolean firstTimeDriveInches = true;


/**
 * @param inches
 *            How far we want to go
 * @param speed
 *            How fast we want to go
 * @return Whether or not we have finished driving yet
 * 
 */
public boolean driveStraightInches (double inches, double speed)
{
    // Again, we don't know why it's going backwards...

    double averageLeft = (this.getLeftFrontEncoderDistance()
            + this.getLeftRearEncoderDistance()) / 2;
    double averageRight = (this.getRightFrontEncoderDistance()
            + this.getRightRearEncoderDistance()) / 2;

    if (firstTimeDriveInches)
        {
        this.resetEncoders();
        firstTimeDriveInches = false;
        }

    // if (this.transmissionType == TransmissionType.MECANUM)
    // this.drive(speed, 0.0);
    if (averageRight >= averageLeft + getEncoderSlack())
        bottomValue = true;
    if (averageRight <= averageLeft - getEncoderSlack())//
        topValue = true;
    if (bottomValue == true && topValue == true)
        this.drive(-speed, -speed);
    if (averageLeft > averageRight - getEncoderSlack())//
        this.transmissionFourWheel.drive(
                -speed - getDriveCorrection(),
                -speed); // negate this because how drive takes into account of
                         // negative joystick
    if (averageLeft < averageRight + getEncoderSlack())//
        this.transmissionFourWheel.drive(-speed,
                -speed - getDriveCorrection());
    if (Math.abs(this.getAveragedEncoderValues()) >= Math.abs(inches))
        {
        this.drive(0.0, 0.0);
        firstTimeDriveInches = true;
        return true;
        }

    return false;


}

private boolean bottomValue = false;

private boolean topValue = false;

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
public AlignReturnType alignToGear (double relativeCenter,
        double movementSpeed,
        double deadband)
{
    if (!isTurning)
        {
        this.imageProcessor.processImage();
        if (this.imageProcessor.getNthSizeBlob(1) == null)
            {
            this.drive(0.0, 0.0);
            return AlignReturnType.NO_BLOBS;
            }
        double distanceToCenter = imageProcessor
                .getPositionOfRobotToGear(
                        imageProcessor
                                .getNthSizeBlob(0),
                        imageProcessor
                                .getNthSizeBlob(1),
                        relativeCenter);

        if (distanceToCenter == Double.MAX_VALUE)
            {
            this.drive(0.0, 0.0);
            return AlignReturnType.NO_BLOBS;
            }
        if (Math.abs(distanceToCenter) <= deadband)
            {
            this.drive(0.0, 0.0);
            return AlignReturnType.ALIGNED;
            }
        }
    // else if (distanceToCenter > 0)
    // this.drive(0, movementSpeed);
    // else if (distanceToCenter < 0)
    // this.drive(0.0, -movementSpeed);
    this.isTurning = this
            .turnDegrees(-Math.toDegrees(this.imageProcessor
                    .getYawAngleToTarget(this.imageProcessor
                            .getNthSizeBlob(0))
                    + this.imageProcessor
                            .getYawAngleToTarget(this.imageProcessor
                                    .getNthSizeBlob(1)))
                    / 2.0);



    return AlignReturnType.MISALIGNED;
}

private boolean isTurning = false;



// TODO we need to test this!!
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
 *            to be when we stop aligning
 * @return Whether or not we are aligned, close enough, misaligned, or see no
 *         blobs.
 */
public AlignReturnType strafeToGear (double driveSpeed,
        double alignVar, double deadband, double relativeCenter,
        int distanceToTarget)
{
    if (this.firstStrafe)
        {
        this.timer.reset();
        this.timer.start();
        this.firstStrafe = false;
        }

    if (this.purgingUltrasonic)
        {
        this.rightUlt.getDistanceFromNearestBumper();
        if (this.timer.get() >= .25)
            {
            this.timer.stop();
            this.purgingUltrasonic = false;
            }
        else
            {
            this.drive(0.0, 0.0);
            return AlignReturnType.WAITING;
            }
        }

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
            + this.rightUlt.getDistanceFromNearestBumper());
    if (distanceToCenter == Double.MAX_VALUE)
        {
        this.driveNoDeadband(0.0, 0.0);
        return AlignReturnType.NO_BLOBS;
        }

    if (this.rightUlt
            .getDistanceFromNearestBumper() <= distanceToTarget)
        {
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

    if (distanceToCenter > 0)
        {
        this.driveNoDeadband(driveSpeed, alignVar);
        }
    else if (distanceToCenter < 0)
        {
        this.driveNoDeadband(driveSpeed, -alignVar);
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
            this.transmissionMecanum.driveNoDeadband(speed, correction,
                    0.0);
            break;
        case TANK:
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
            targetSpeed * (this.timer.get() / timeInWhichToAccelerate),
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
 * Sets the radius of the circlie that the robot rotates around
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
    return encoderSlack;
}

public void setDriveCorrection (double correction)
{
    this.correction = correction;
}

public double getDriveCorrection ()
{
    return correction;
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

/**
 * The value that the getDistance is multiplied by to get an accurate
 * distance.
 */
private static final double DEFAULT_DISTANCE_PER_PULSE = 1.0 / 12.9375;


}

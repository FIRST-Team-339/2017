package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyCamera;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionFourWheel;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionMecanum;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.Encoder;

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

private KilroyCamera camera = null;

private Encoder rightFrontEncoder = null;

private Encoder rightRearEncoder = null;

private Encoder leftFrontEncoder = null;

private Encoder leftRearEncoder = null;

private boolean isUsingEncoders = false;

private boolean isUsingUltrasonics = false;

private UltraSonic rightUlt = null;

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
        KilroyCamera camera, ImageProcessor imageProcessor)
{
    this.transmissionMecanum = transmissionMecanum;
    this.transmissionType = TransmissionType.MECANUM;
    this.camera = camera;
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
        KilroyCamera camera, ImageProcessor imageProcessor,
        Encoder rightFrontEncoder, Encoder rightRearEncoder,
        Encoder leftFrontEncoder, Encoder leftRearEncoder,
        UltraSonic rightUlt)
{
    this(transmissionMecanum, camera, imageProcessor);

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
        KilroyCamera camera, ImageProcessor imageProcessor)
{
    this.transmissionFourWheel = transmissionFourWheel;
    this.transmissionType = TransmissionType.TANK;
    this.camera = camera;
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
public Drive (TransmissionFourWheel transmissionFourWheel,
        KilroyCamera camera, ImageProcessor imageProcessor,
        Encoder leftFrontEncoder, Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        UltraSonic rightUlt)
{
    this(transmissionFourWheel, camera, imageProcessor);
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
    // Again, we don't know why it's going backwards...

    if (firstTimeDriveInches)
        {
        this.resetEncoders();
        firstTimeDriveInches = false;
        }

    if (Math.abs(this.getAveragedEncoderValues()) >= Math.abs(inches))
        {
        if (this.transmissionType == TransmissionType.MECANUM)
            this.transmissionMecanum.drive(0.0, 0.0, 0.0, 0, 0);
        else
            this.transmissionFourWheel.drive(0.0, 0.0);
        firstTimeDriveInches = true;
        return true;
        }
    if (this.transmissionType == TransmissionType.MECANUM)
        this.transmissionMecanum.drive(speed, 0.0, 0.0, 0, 0);
    else
        this.transmissionFourWheel.drive(-speed, -speed);

    return false;
}

private boolean firstTimeDriveInches = true;

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
    switch (this.transmissionType)
        {
        case TANK:
            if (isTurning == false)
                {
                this.imageProcessor.processImage();
                if (this.imageProcessor.getNthSizeBlob(1) != null)
                    {
                    double distanceToCenter = imageProcessor
                            .getPositionOfRobotToGear(
                                    imageProcessor
                                            .getNthSizeBlob(0),
                                    imageProcessor
                                            .getNthSizeBlob(1),
                                    relativeCenter);
                    if (distanceToCenter > 0)
                        {
                        System.out.println("We are Left of target");
                        }
                    else if (distanceToCenter < 0)
                        System.out
                                .println("We are RIGHT of target");
                    System.out
                            .println("Distance to center: "
                                    + distanceToCenter);

                    System.out.println("Deadband: " + (10.0
                            / this.camera
                                    .getHorizontalResolution()));
                    if (distanceToCenter == Double.MAX_VALUE)
                        {
                        transmissionFourWheel.drive(0.0, 0.0);
                        return AlignReturnType.NO_BLOBS;
                        }
                    if (Math.abs(distanceToCenter) <= deadband)
                        {
                        transmissionFourWheel.drive(0.0, 0.0);
                        return AlignReturnType.ALIGNED;
                        }
                    else if (distanceToCenter > 0)
                        {
                        this.transmissionFourWheel.drive(movementSpeed,
                                -movementSpeed);
                        }
                    else if (distanceToCenter < 0)
                        {
                        this.transmissionFourWheel.drive(-movementSpeed,
                                movementSpeed);
                        }
                    }

                }
            // Turns based on the average of the yaw angles of the
            // two blobs, to account for a delayed axis camera.
            // double yawAngle = (this.imageProcessor
            // .getYawAngleToTarget(
            // this.imageProcessor.getNthSizeBlob(1))
            // + this.imageProcessor.getYawAngleToTarget(
            // this.imageProcessor.getNthSizeBlob(0))
            // / 2.0);
            // System.out.println("Average yaw angle: " + yawAngle);
            //
            //
            //
            // isTurning = this.turnDegrees((this.imageProcessor
            // .getYawAngleToTarget(this.imageProcessor
            // .getNthSizeBlob(0))
            // + this.imageProcessor.getYawAngleToTarget(
            // this.imageProcessor
            // .getNthSizeBlob(1)))
            // / 2.0, .5);
            break;
        case MECANUM:

            break;
        default:
            break;
        }
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
 *            Where we want the center of the robot to be
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
    // If we have no blobs, return so.
    if (this.imageProcessor.getNthSizeBlob(1) == null)
        return AlignReturnType.NO_BLOBS;

    // If we don't have any ultrasonics in the constructor, stop aligning.
    if (this.isUsingUltrasonics == false)
        return AlignReturnType.ALIGNED;

    double distanceToCenter = this.imageProcessor
            .getPositionOfRobotToGear(
                    this.imageProcessor.getNthSizeBlob(0),
                    this.imageProcessor.getNthSizeBlob(1),
                    relativeCenter);

    if (this.rightUlt
            .getOffsetDistanceFromNearestBummper() <= distanceToTarget)
        return AlignReturnType.CLOSE_ENOUGH;

    if (Math.abs(distanceToCenter) < deadband)
        {
        if (this.transmissionType == TransmissionType.MECANUM)
            transmissionMecanum.drive(driveSpeed, 0.0, 0.0, 0, 0);
        else if (this.transmissionType == TransmissionType.TANK)
            transmissionFourWheel.drive(driveSpeed, driveSpeed);
        return AlignReturnType.ALIGNED;
        }

    if (distanceToCenter > 0)
        {
        if (this.transmissionType == TransmissionType.MECANUM)
            this.transmissionMecanum.drive(driveSpeed, alignVar, 0, 0,
                    0);
        else if (this.transmissionType == TransmissionType.TANK)
            this.transmissionFourWheel.drive(driveSpeed - alignVar,
                    driveSpeed + alignVar);
        }
    else if (distanceToCenter < 0)
        {
        if (this.transmissionType == TransmissionType.MECANUM)
            this.transmissionMecanum.drive(driveSpeed, -alignVar, 0.0,
                    0, 0);
        else if (this.transmissionType == TransmissionType.TANK)
            this.transmissionFourWheel.drive(driveSpeed + alignVar,
                    driveSpeed - alignVar);
        }


    return AlignReturnType.MISALIGNED;
}



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
    CLOSE_ENOUGH
    }

public void strafeStraight (double inches)
{
    resetEncoders();
    double rightFrontSpeed = inches;
    double rightRearSpeed = inches;
    double leftFrontSpeed = inches;
    double leftRearSpeed = inches;

}

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
            transmissionMecanum.drive(0.0, 0.0, -speed, 0, 0);
        }
    else if (adjustedDegrees > 0)
        {
        if (transmissionType == TransmissionType.TANK)
            transmissionFourWheel.drive(-speed, speed);
        else
            transmissionMecanum.drive(0.0, 0.0, speed, 0, 0);
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

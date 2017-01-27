package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyCamera;
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

public Drive (TransmissionMecanum transmissionMecanum,
        KilroyCamera camera, ImageProcessor imageProcessor,
        Encoder rightFrontEncoder, Encoder rightRearEncoder,
        Encoder leftFrontEncoder, Encoder leftRearEncoder)
{
    this.transmissionMecanum = transmissionMecanum;
    this.transmissionType = TransmissionType.MECANUM;
    this.camera = camera;
    this.imageProcessor = imageProcessor;

    this.rightFrontEncoder = rightFrontEncoder;
    this.rightRearEncoder = rightRearEncoder;
    this.leftFrontEncoder = leftFrontEncoder;
    this.leftRearEncoder = rightRearEncoder;
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
    imageProcessor.processImage();
    if (imageProcessor.getParticleAnalysisReports() != null)
        switch (this.transmissionType)
            {
            case TANK:
                double position = imageProcessor
                        .getPositionOfRobotToGear(
                                imageProcessor.getNthSizeBlob(0),
                                imageProcessor.getNthSizeBlob(1),
                                relativeCenter);
                double distanceToCenter = position
                        / camera.getHorizontalResolution();
                System.out
                        .println("Distance to center: "
                                + distanceToCenter);

                System.out.println("Deadband: " + (10.0
                        / this.camera.getHorizontalResolution()));
                if (position == Double.MAX_VALUE)
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
                    transmissionFourWheel.drive(movementSpeed,
                            -movementSpeed);
                    }
                else if (distanceToCenter < 0)
                    {
                    transmissionFourWheel.drive(-movementSpeed,
                            movementSpeed);
                    }
                break;
            case MECANUM:

                break;
            default:
                break;
            }
    return AlignReturnType.MISALIGNED;
}



public static enum AlignReturnType
    {
    NO_BLOBS, ALIGNED, MISALIGNED
    }




public static enum TurnDirection
    {
    RIGHT, LEFT
    }




public static enum TransmissionType
    {
    TANK, MECANUM
    }

private TransmissionType transmissionType = null;

}

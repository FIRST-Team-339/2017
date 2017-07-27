package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import org.usfirst.frc.team339.HardwareInterfaces.newtransmission.TransmissionBase.TransmissionType;

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

	private Encoder leftFrontEncoder = null, rightFrontEncoder = null, leftRearEncoder = null, rightRearEncoder = null;

	private final TransmissionType transmissionType;

	/**
	 * Creates the Drive object.
	 * 
	 * @param transmission The robot's transmission object
	 */
	public Drive(TransmissionBase transmission)
	{
		this.transmissionType = transmission.getType();

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

		}
	}

	/**
	 * Gets the transmission object stored. You will NOT be able to 
	 * drive the transmission using this function, with or without the
	 * joysticks. You will only be able to stop it.
	 * 
	 * @return The current transmission object used in the Drive class
	 */
	public TransmissionBase getTransmission()
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
		RIGHT_SIDE
	}

	/**
	 * Sets how far the robot has driven per pulse the encoder reads.
	 * This value should be much lower than one, as there are usually 
	 * hundreds of pulses per rotation.
	 * 
	 *  To calculate, reset the encoders and
	 * push the robot forwards, say, five feet. Then count the number of pulses
	 * and do: (5x12)/pulses to get this in inches.
	 * 
	 * @param value The encoder distance per pulse.
	 */
	public void setEncoderDistancePerPulse(double value)
	{
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
		{
			leftFrontEncoder.setDistancePerPulse(value);
			rightFrontEncoder.setDistancePerPulse(value);
			leftRearEncoder.setDistancePerPulse(value);
			rightRearEncoder.setDistancePerPulse(value);
		} else if (transmissionType == TransmissionType.TRACTION)
		{
			leftRearEncoder.setDistancePerPulse(value);
			rightRearEncoder.setDistancePerPulse(value);
		}
	}

	/**
	 * Sets the encoder's stored pulses back to zero.
	 */
	public void resetEncoders()
	{
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
		{
			leftFrontEncoder.reset();
			rightFrontEncoder.reset();
			leftRearEncoder.reset();
			rightRearEncoder.reset();
		} else if (transmissionType == TransmissionType.TRACTION)
		{
			leftRearEncoder.reset();
			rightRearEncoder.reset();
		}
	}

	/**
	 * Gets the averages of certain wheel groups. All values are the absolute value to stop 
	 * negative numbers from affecting the average.
	 * @param encoderGroup
	 * @return
	 */
	public double getEncoderDistanceAverage(WheelGroups encoderGroup)
	{
		switch (encoderGroup)
		{
		case ALL:
			return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(rightFrontEncoder.getDistance())
					+ Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 4.0;
		case LEFT_SIDE:
			return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(leftRearEncoder.getDistance())) / 2.0;
		case RIGHT_SIDE:
			return (Math.abs(rightFrontEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
		default:
			return 0.0;
		}
	}

	/**
	 * Tests whether any encoder reads larger than the input length. Useful for knowing
	 * when to stop the robot.
	 * @param length The desired length
	 * @return True when any encoder is past length
	 */
	private boolean isAnyEncoderLargerThan(double length)
	{
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			return (Math.abs(leftFrontEncoder.getDistance()) > length
					|| Math.abs(rightFrontEncoder.getDistance()) > length
					|| Math.abs(leftRearEncoder.getDistance()) > length
					|| Math.abs(rightRearEncoder.getDistance()) > length);
		return (Math.abs(leftRearEncoder.getDistance()) > length || Math.abs(rightRearEncoder.getDistance()) > length);

	}

	// ================DRIVING FUNCTIONS================

	private boolean driveInchesInit = true;

	/**
	 * Drives the robot a certain distance based on the encoder values. 
	 * If the robot should go backwards, set speed to be negative instead of distance.
	 * 
	 * @param distance How far the robot should go (should be greater than 0)
	 * @param speed How fast the robot should travel
	 * @return Whether or not the robot has finished traveling that given distance.
	 */
	public boolean driveInches(int distance, double speed)
	{
		// Runs once when the method runs the first time, and does not run again
		// until after the method returns true.
		if (driveInchesInit == true)
		{
			this.resetEncoders();
			driveInchesInit = false;
		}

		if (isAnyEncoderLargerThan(distance))
		{
			this.getTransmission().stop();
			driveInchesInit = true;
			return true;
		}

		this.getTransmission().driveRaw(speed, speed);

		return false;
	}

	/**
	 * Drives the robot in a straight line based on encoders.
	 * 
	 * This works by polling the encoders every (COLLECTION_TIME) milliseconds
	 * and then taking the difference from the last collection and using it as a 
	 * ratio to multiply times the speed.
	 * 
	 * This approach allows a more dynamic correction, as it only corrects as much as
	 * it needs to.
	 * 
	 * Remember: reset the encoders before running this method.
	 * 
	 * @param speed How fast the robot will be moving. Correction will be better with lower percentages.
	 */
	public void driveStraight(double speed)
	{
		// Only check encoders if the right amount of time has elapsed
		// (collectionTime).
		if (System.currentTimeMillis() > driveStraightOldTime + COLLECTION_TIME)
		{
			// Reset the "timer"
			driveStraightOldTime = System.currentTimeMillis();
			int leftChange, rightChange;
			// Only use the four encoders if the robot uses a four-wheel system
			if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			{
				// Calculate how much has changed between the last collection
				// time and now
				leftChange = (leftFrontEncoder.get() + leftRearEncoder.get()) - prevEncoderValues[0];
				rightChange = (rightFrontEncoder.get() + rightRearEncoder.get()) - prevEncoderValues[1];
				// Setup the previous values for the next collection run
				prevEncoderValues[0] = leftFrontEncoder.get() + leftRearEncoder.get();
				prevEncoderValues[1] = rightFrontEncoder.get() + rightRearEncoder.get();
			} else
			{
				// Calculate how much has changed between the last collection
				// time and now
				leftChange = leftRearEncoder.get() - prevEncoderValues[0];
				rightChange = rightRearEncoder.get() - prevEncoderValues[1];
				// Setup the previous values for the next collection run
				prevEncoderValues[0] = leftRearEncoder.get();
				prevEncoderValues[1] = rightRearEncoder.get();
			}

			// Changes how much the robot corrects by how off course it is. The
			// more off course, the more it will attempt to correct.
			this.getTransmission().driveRaw(speed * ((double) rightChange / leftChange),
					speed * ((double) leftChange / rightChange));

		}
	}

	private int[] prevEncoderValues =
	{ 1, 1 };
	// Preset to 1 to avoid divide by zero errors.

	// Used for calculating how much time has passed for driveStraight
	private long driveStraightOldTime = 0;

	// ================TUNABLES================

	// Number of milliseconds that will pass before collecting data on encoders
	// for driveStraight
	private static final int COLLECTION_TIME = 20;
}

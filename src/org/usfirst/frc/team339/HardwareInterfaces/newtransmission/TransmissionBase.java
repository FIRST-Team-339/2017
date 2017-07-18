package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

/**
 * Contains necessary functions that must be included in
 * each transmission type class created.
 * 
 * @author Ryan McGee
 * @written 7/17/2017
 */
public abstract class TransmissionBase
{
	protected final double[] gearRatios =
	{ .5, .7, 1 };

	// Will default to the highest gear available
	protected int currentGear = gearRatios.length;

	// The motors will start turning only once the joystick is past this
	// deadband.
	protected double joystickDeadband = .2;

	/**
	 * The current types of transmissions available.
	 * 
	 * @author Ryan McGee
	 *
	 */
	public enum TransmissionType
	{
		/**
		 * Tank-style drive system with 2 driven traction wheels
		 * and two omniwheels for smooth steering.
		 */
		TRACTION,
		/**
		 * Tank-style drive system with four omniwheels each driven
		 * by a separate motor.
		 */
		TANK,
		/**
		 * Four driven mecanum wheels that allows strafing as well as 
		 * linear movement and rotation.
		 */
		MECANUM
	}

	protected TransmissionType type = null;

	/**
	 * @return The type of transmission of a class extending TransmissionBase.
	 */
	public TransmissionType getType()
	{
		return type;
	}

	/**
	 * Sets the current gear for the robot. This will change the maximum
	 * speed of the robot for precise aiming/driving.
	 * @param gear The requested gear number. If outside the range, it will do nothing.
	 */
	public void setGear(int gear)
	{
		if (gear >= 0 && gear < gearRatios.length)
			this.currentGear = gear;
	}

	/**
	 * Adds one to the current gear of the robot, allowing the user to drive faster.
	 */
	public void upShift()
	{
		if (currentGear < gearRatios.length - 1)
			currentGear++;
	}

	/**
	 * Removes one from the current gear of the robot, allowing the user to drive slower.
	 */
	public void downShift()
	{
		if (currentGear > 0)
			currentGear--;
	}

	/**
	 * Sets the minimum value the joysticks must output in order for the robot to start moving.
	 * @param deadband Percentage value, ranging from 0.0 to 1.0, in decimals.
	 */
	public void setJoystickDeadband(double deadband)
	{
		this.joystickDeadband = deadband;
	}

	/**
	 *  Uses the formula for mapping one set of values to the other:
	 * y = mx + b
	 * 
	 * m = 1 / (1 - deadband)
	 * b = (1 - m)
	 * x = joystick input
	 * y = motor output
	 * 
	 * Therefore,
	 * motor output = (1 / (1 - deadband)) * joystick input
	 * 					+ (1 - (1 / (1 - deadband)))
	 * 
	 * If this equation does not make much sense, try graphing it first
	 * as the original x = y, and then the scaled output starting at the
	 * deadband, and use the slope formula.
	 * 
	 * @param input
	 * @return
	 */
	public double scaleJoystickForDeadband(double input)
	{
		double deadbandSlope = 1.0 / (1.0 - joystickDeadband);
		double constant = 1 - deadbandSlope;

		return (deadbandSlope * input) + constant;
	}

}

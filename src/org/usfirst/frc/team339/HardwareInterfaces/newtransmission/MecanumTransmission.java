package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * One of the more complex drive systems: uses 4 mecanum wheels to allow
 * for lateral movement as well as normal tank controls.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class MecanumTransmission extends TransmissionBase
{

	private final SpeedController leftFrontMotor;

	private final SpeedController rightFrontMotor;

	private final SpeedController leftRearMotor;

	private final SpeedController rightRearMotor;

	private double strafeCushion = 0;// Disabled by default.

	private double directionalDeadband = Math.toRadians(15);

	/**
	 * Creates the MecanumTransmission object.
	 * 
	 * @param leftFrontMotor
	 *            The left front motor controller
	 * @param rightFrontMotor
	 *            The right front motor controller
	 * @param leftRearMotor
	 *            The left rear motor controller
	 * @param rightRearMotor
	 *            The right rear motor controller
	 */
	public MecanumTransmission(SpeedController leftFrontMotor, SpeedController rightFrontMotor,
			SpeedController leftRearMotor, SpeedController rightRearMotor)
	{
		this.leftFrontMotor = leftFrontMotor;
		this.rightFrontMotor = rightFrontMotor;
		this.leftRearMotor = leftRearMotor;
		this.rightRearMotor = rightRearMotor;

		super.type = TransmissionType.MECANUM;
	}

	/**
	 * Drives the robot with the aid of a joystick deadband, directional deadbands,
	 * and software gear ratios.
	 * 
	 * @param joystick
	 *            A 3-axis joystick to control 2 axis movement plus turning.
	 */
	public void drive(Joystick joystick)
	{
		this.drive(joystick.getMagnitude(), joystick.getDirectionRadians(), joystick.getZ());
	}

	/**
	 * Drives the robot with mecanum with raw values, taking into account
	 * joystick deadbands and gear ratios.
	 * 
	 * 
	 * @param magnitude The magnitude of the joystick, (0.0 to 1.0)
	 * @param direction The direction of the joystick in radians (-PI to PI)
	 * @param rotation The rotation of the joystick, (-1.0 to 1.0)
	 */
	public void drive(double magnitude, double direction, double rotation)
	{
		double altMagnitude, altDirection, altRotation;

		altMagnitude = super.scaleJoystickForDeadband(magnitude) * super.gearRatios[super.currentGear];
		altDirection = direction;
		altRotation = super.scaleJoystickForDeadband(rotation) * super.gearRatios[super.currentGear];

		// Check between the deadbands for the strafing cushion and and 90
		// degree "snap".
		if (direction > (-Math.PI / 2.0) - (directionalDeadband / 2.0)
				&& direction < (-Math.PI / 2.0) + (directionalDeadband / 2.0))
		{
			altDirection = -Math.PI / 2.0;
			altMagnitude += this.strafeCushion;
		} else if (direction > (Math.PI / 2.0) - (directionalDeadband / 2.0)
				&& direction < (Math.PI / 2.0) + (directionalDeadband / 2.0))
		{
			altDirection = Math.PI / 2.0;
			altMagnitude += this.strafeCushion;
		}

		this.driveRaw(altMagnitude, altDirection, altRotation);

	}

	/**
	 * Drives the robot without the use of deadbands and gear ratios. The value
	 * input is the value that is passed to the motors (after going through the
	 * formula for mecanum drive)
	 * 
	 * The equation for mecanum drive is:
	 * y = m * (cos(d + (PI/4)) + z) where d is the direction, m is the magnitude,
	 * and z is the rotation,
	 * for the front left and back right wheels.
	 * 
	 * y = m * (cos(d - (PI/4)) + z)
	 * for the front right and back left wheels.
	 * 
	 * values must be cut off at one, so a higher or lower z may push it over 1.0 or
	 * under -1.0.
	 * In this case, the function inRange makes those cutoffs.
	 * 
	 * @param magnitude
	 *            How fast the robot moves (0 to 1.0)
	 * @param direction
	 *            which direction the robot will travel in radians. (0 is front, PI or -PI
	 *            is back)
	 * @param rotation
	 *            How fast the robot should turn( -1.0 to 1.0, zero is straight)
	 */
	public void driveRaw(double magnitude, double direction, double rotation)
	{

		double leftFrontVal = Math.cos(direction + (Math.PI / 4.0));
		double rightFrontVal = Math.cos(direction - (Math.PI / 4.0));
		double rightRearVal = leftFrontVal;// The corner's equations are
		double leftRearVal = rightFrontVal;// equal to each other.

		leftFrontMotor.set(magnitude * inRange(leftFrontVal + rotation));
		leftRearMotor.set(magnitude * inRange(leftRearVal + rotation));
		rightFrontMotor.set(magnitude * inRange(rightFrontVal - rotation));
		rightRearMotor.set(magnitude * inRange(rightRearVal - rotation));

	}

	/**
	 * Sets the angle deadband where the direction snaps to a 90 degree angle.
	 * 
	 * @param value
	 *            deadband, in degrees
	 */
	public void setDirectionalDeadband(double value)
	{
		this.directionalDeadband = Math.toRadians(value);
	}

	/**
	 * Sets the percentage added to the magnitude while strafing.
	 * This is only used inside the directional deadband, so if it is set to 0,
	 * then it will not take affect.
	 * 
	 * @param value
	 *            The value that strafeCusion will be set to.
	 */
	public void setStrafeCusion(double value)
	{
		this.strafeCushion = value;
	}

	/**
	 * Checks if the value input is in between -1 and 1 to keep it in range for
	 * motor inputs.
	 * 
	 * @param val
	 *            The input value
	 * @return The correctly ranged value
	 */
	private double inRange(double val)
	{
		if (val > 1)
			return 1;
		else if (val < -1)
			return -1;

		return val;
	}

	@Override
	public void driveRaw(double leftVal, double rightVal)
	{
		this.leftFrontMotor.set(leftVal);
		this.rightFrontMotor.set(rightVal);
		this.leftRearMotor.set(leftVal);
		this.rightRearMotor.set(rightVal);
	}

	@Override
	public void stop()
	{
		this.leftFrontMotor.set(0.0);
		this.rightFrontMotor.set(0.0);
		this.leftRearMotor.set(0.0);
		this.rightRearMotor.set(0.0);
	}
}

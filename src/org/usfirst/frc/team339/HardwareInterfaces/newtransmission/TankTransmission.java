package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Like the traction drive system, except with four motors, usually all as omni-wheels.
 * Or, tank drive with paired motors. Each joystick controls one side of the robot.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class TankTransmission extends TransmissionBase
{

	private final SpeedController leftFrontMotor;
	private final SpeedController rightFrontMotor;
	private final SpeedController leftRearMotor;
	private final SpeedController rightRearMotor;

	/**
	 * Creates the Transmission object.
	 * 
	 * @param leftFrontMotor The left-front motor controller
	 * @param rightFrontMotor The right-front motor controller
	 * @param leftRearMotor The left-rear motor controller
	 * @param rightRearMotor The right-rear motor controller
	 */
	public TankTransmission(SpeedController leftFrontMotor, SpeedController rightFrontMotor,
			SpeedController leftRearMotor, SpeedController rightRearMotor)
	{
		this.leftFrontMotor = leftFrontMotor;
		this.rightFrontMotor = rightFrontMotor;
		this.leftRearMotor = leftRearMotor;
		this.rightRearMotor = rightRearMotor;

		super.type = TransmissionType.TANK;
	}

	/**
	 * Controls the robot with the aid of deadbands and software gear ratios.
	 * 
	 * @param leftJoystick The joystick that will control the left side of the robot
	 * @param rightJoystick The joystick that will control the right side of the robot
	 */
	public void drive(Joystick leftJoystick, Joystick rightJoystick)
	{
		double leftY = leftJoystick.getY();
		double rightY = rightJoystick.getY();

		// Setting the left motors only if the left joystick is passed the
		// deadband, and scale the output based on the current gear.
		if (leftY > joystickDeadband)
		{
			leftFrontMotor.set(gearRatios[currentGear] * super.scaleJoystickForDeadband(leftY));
			leftRearMotor.set(gearRatios[currentGear] * super.scaleJoystickForDeadband(leftY));
		} else if (leftY < -joystickDeadband)
		{
			// -leftY will make it positive again, acting like an absolute value
			// for the formula to work.
			leftFrontMotor.set(gearRatios[currentGear] * -super.scaleJoystickForDeadband(-leftY));
			leftRearMotor.set(gearRatios[currentGear] * -super.scaleJoystickForDeadband(-leftY));
		} else
		{
			leftFrontMotor.set(0.0);
			leftRearMotor.set(0.0);
		}

		// Setting the right motor only if the left joystick is passed the
		// deadband, and scale the output based on the current gear.
		if (rightY > joystickDeadband)
		{
			rightFrontMotor.set(gearRatios[currentGear] * super.scaleJoystickForDeadband(rightY));
			rightRearMotor.set(gearRatios[currentGear] * super.scaleJoystickForDeadband(rightY));
		} else if (rightY < -joystickDeadband)
		{
			// -rightY will make it positive again, acting like an absolute
			// value for the formula to work.
			rightFrontMotor.set(gearRatios[currentGear] * -super.scaleJoystickForDeadband(-rightY));
			rightRearMotor.set(gearRatios[currentGear] * -super.scaleJoystickForDeadband(-rightY));
		} else
		{
			rightFrontMotor.set(0.0);
			rightRearMotor.set(0.0);
		}
	}

	@Override
	public void driveRaw(double leftVal, double rightVal)
	{
		leftFrontMotor.set(leftVal);
		leftRearMotor.set(leftVal);

		rightFrontMotor.set(rightVal);
		rightRearMotor.set(rightVal);
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

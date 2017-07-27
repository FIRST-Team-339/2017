package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSpeedController;

/**
 * The most basic of drive systems: contains 2 motor controllers, 
 * one on the left side of the robot, and one on the right. 
 * 
 * @author Ryan McGee
 * @written 7/17/17
 */
public class TractionTransmission extends TransmissionBase
{
	private final PWMSpeedController leftMotor;
	private final PWMSpeedController rightMotor;

	/**
	 * Constructs the transmission object with 2 motors.
	 * 
	 * @param leftMotor The controller connected to the left motor
	 * @param rightMotor The controller connected to the right motor
	 */
	public TractionTransmission(PWMSpeedController leftMotor, PWMSpeedController rightMotor)
	{
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		super.type = TransmissionType.TRACTION;
	}

	/**
	 * Controls the robot with the help of gear ratios and deadbands.
	 * 
	 * @param leftJoystick The joystick that will control the left side of the robot
	 * @param rightJoystick The joystick that will control the right side of the robot
	 */
	public void drive(Joystick leftJoystick, Joystick rightJoystick)
	{
		double leftY = leftJoystick.getY();
		double rightY = rightJoystick.getY();

		// Setting the left motor only if the left joystick is passed the
		// deadband, and scale the output based on the current gear.
		if (leftY > joystickDeadband)
		{
			leftMotor.set(gearRatios[currentGear] * scaleJoystickForDeadband(leftY));
		} else if (leftY < -joystickDeadband)
		{
			// -leftY will make it positive again, acting like an absolute value
			// for the formula to work.
			leftMotor.set(gearRatios[currentGear] * -scaleJoystickForDeadband(-leftY));
		} else
		{
			leftMotor.set(0.0);
		}

		// Setting the right motor only if the left joystick is passed the
		// deadband, and scale the output based on the current gear.
		if (rightY > joystickDeadband)
		{
			rightMotor.set(gearRatios[currentGear] * scaleJoystickForDeadband(rightY));
		} else if (rightY < -joystickDeadband)
		{
			// -rightY will make it positive again, acting like an absolute
			// value for the formula to work.
			rightMotor.set(gearRatios[currentGear] * -scaleJoystickForDeadband(-rightY));
		} else
		{
			rightMotor.set(0.0);
		}
	}

	@Override
	public void driveRaw(double leftVal, double rightVal)
	{
		leftMotor.set(leftVal);
		rightMotor.set(rightVal);
	}

	@Override
	public void stop()
	{
		this.leftMotor.set(0.0);
		this.rightMotor.set(0.0);
	}

}

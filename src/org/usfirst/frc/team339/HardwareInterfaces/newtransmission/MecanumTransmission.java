package org.usfirst.frc.team339.HardwareInterfaces.newtransmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * TODO Test this on Buzzbot!
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
public MecanumTransmission (SpeedController leftFrontMotor,
        SpeedController rightFrontMotor,
        SpeedController leftRearMotor, SpeedController rightRearMotor)
{
    this.leftFrontMotor = leftFrontMotor;
    this.rightFrontMotor = rightFrontMotor;
    this.leftRearMotor = leftRearMotor;
    this.rightRearMotor = rightRearMotor;

    super.type = TransmissionType.MECANUM;
}

/**
 * Drives the robot with the aid of a joystick deadband and software gear
 * ratios.
 * 
 * The equation for mecanum drive is:
 * y = m * (cos(d + (PI/4)) + z) where d is the direction, m is the magnitude,
 * and z is the rotation,
 * for-the front left and back right wheels.
 * 
 * y = m * (cos(d - (PI/4) + z)
 * for the front right and back left wheels.
 * 
 * values must be cut off at one, so a higher or lower z may push it over 1.0 or
 * under -1.0.
 * In this case, the function inRange makes those cutoffs.
 * 
 * @param joystick
 *            A 3-axis joystick to control 2 axis movement plus turning.
 */
public void drive (Joystick joystick)
{
    double magnitude = joystick.getMagnitude();
    double direction = joystick.getDirectionRadians();
    double twistVal = 0;

    double leftFrontVal = Math.cos(direction + (Math.PI / 4.0));
    double rightFrontVal = Math.cos(direction - (Math.PI / 4.0));
    double rightRearVal = leftFrontVal;// The corner's equations are
    double leftRearVal = rightFrontVal;// equal to each other.

    double gearMultiplier = magnitude
            * super.gearRatios[super.currentGear];

    // Check for the trigger first. Only add the Z value if it is pressed
    // and above the deadband.
    if (magnitude > super.joystickDeadband
            || joystick.getTrigger() == true)
        {
        // Only use the joystick's z axis if the trigger is pressed and its
        // within the deadband.
        if (joystick.getTrigger() == true
                && Math.abs(joystick.getZ()) > super.joystickDeadband)
            {
            twistVal = super.gearRatios[super.currentGear]
                    * joystick.getZ();
            }

        // Checks each value to either one or zero to make sure all values
        // are between -1 and 1.

        // Because the z axis increases as you twist clockwise, it is added
        // to the left motors and subtracted from the right.

        leftFrontMotor
                .set(inRange(
                        (gearMultiplier * leftFrontVal) + twistVal));
        leftRearMotor
                .set(inRange(
                        (gearMultiplier * leftRearVal) + twistVal));
        rightFrontMotor.set(
                inRange((gearMultiplier * rightFrontVal) - twistVal));
        rightRearMotor
                .set(inRange(
                        (gearMultiplier * rightRearVal) - twistVal));

        }
    else
        {
        this.driveRaw(0.0, 0.0, 0.0);
        }
}

/**
 * Drives the robot without the use of deadbands and gear ratios. The value
 * input is the value that is passed to the motors (after going through the
 * formula for mecanum drive)
 * 
 * @param magnitude
 *            How fast the robot moves (0 to 1.0)
 * @param direction
 *            which direction the robot will travel in degrees. (0 is front, 180
 *            is back)
 * @param rotation
 *            How fast the robot should turn( -1.0 to 1.0, zero is straight)
 */
public void driveRaw (double magnitude, double direction,
        double rotation)
{

    double leftFrontVal = Math
            .cos(Math.toRadians(direction) + (Math.PI / 4.0));
    double rightFrontVal = Math
            .cos(Math.toRadians(direction) - (Math.PI / 4.0));
    double rightRearVal = leftFrontVal;// The corner's equations are
    double leftRearVal = rightFrontVal;// equal to each other.

    leftFrontMotor.set(magnitude * inRange(leftFrontVal + rotation));
    leftRearMotor.set(magnitude * inRange(leftRearVal + rotation));
    rightFrontMotor.set(magnitude * inRange(rightFrontVal - rotation));
    rightRearMotor.set(magnitude * inRange(rightRearVal - rotation));

}

/**
 * Checks if the value input is in between -1 and 1 to keep it in range for
 * motor inputs.
 * 
 * @param val
 *            The input value
 * @return The correctly ranged value
 */
private double inRange (double val)
{
    if (val > 1)
        return 1;
    else if (val < -1)
        return -1;

    return val;
}

@Override
public void driveRaw (double leftVal, double rightVal)
{
    this.leftFrontMotor.set(leftVal);
    this.rightFrontMotor.set(rightVal);
    this.leftRearMotor.set(leftVal);
    this.rightRearMotor.set(rightVal);
}

@Override
public void stop ()
{
    this.leftFrontMotor.set(0.0);
    this.rightFrontMotor.set(0.0);
    this.leftRearMotor.set(0.0);
    this.rightRearMotor.set(0.0);
}
}

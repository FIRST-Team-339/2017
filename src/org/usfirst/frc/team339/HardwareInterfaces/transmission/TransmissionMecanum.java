package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

public class TransmissionMecanum extends TransmissionFourWheel
{

/**
 * @description If we are within x many degrees of being purely up, down,
 *              left, or right, then we send that "pure" degree value to
 *              account for human error in joystick control.
 *
 * @author Noah Golmant
 * @written 23 July 2015
 */
private double directionalDeadzone = 0.0;

private double directionalXOrthogonalZone = 0.0;

private double directionalYOrthogonalZone = 0.0;

private double firstGearPercentage = 0.0;

/** Sets whether or not the mecanum control joystick is reversed */
private boolean mecanumJoystickReversed = false;

/**
 * Transmission class to control a four-wheel mecanum drive robot.
 *
 * @param rightFrontSpeedController
 * @param rightRearSpeedController
 * @param leftFrontSpeedController
 * @param leftRearSpeedController
 *
 * @author Noah Golmant #written 23 July 2015
 */
public TransmissionMecanum (SpeedController rightFrontSpeedController,
        SpeedController rightRearSpeedController,
        SpeedController leftFrontSpeedController,
        SpeedController leftRearSpeedController)
{
    super(rightFrontSpeedController, rightRearSpeedController,
            leftFrontSpeedController, leftRearSpeedController);

}

/**
 * Drives without taking into account the current deadband set into this
 * class.
 * 
 * @param magnitude
 *            The magnitude of the drive vector we're using
 * @param direction
 *            The direction of the drive vector we're using
 * @param rotation
 *            The rotation we'll drive with.
 */
public void driveNoDeadband (double magnitude, double direction,
        double rotation)
{
    // Save the current deadband
    this.tempSavedDeadband = this.getDeadbandPercentageZone();
    // set the current deadband to zero so we can drive slowly
    this.setDeadbandPercentageZone(0.0);
    // drive without the deadband
    this.drive(magnitude, direction, rotation);
    // restore the deadband
    this.setDeadbandPercentageZone(tempSavedDeadband);
}

private double tempSavedDeadband = 0.0;

/**
 * Drives the transmission in mecanum drive with 0 rotation. NOTE: this
 * should not be used; it is preferable to use the 3-argument one for
 * consistency.
 *
 * @param magnitude
 *            the magnitude of the vector we want to travel in
 * @param direction
 *            the direction of the vector we want to travel in
 *
 * @author Noah Golmant
 * @date 23 July 2015
 */
@Override
@Deprecated
public void drive (double magnitude, double direction)
{
    this.drive(magnitude, direction, 0.0);
}

/**
 * Drives the transmission in a four wheel drive . rightJoystickVal controls
 * both right motors, and vice versa for the left. It scales it according to
 * our deadband and the current gear, then makes sure we're not out of our
 * allowed motor value ranges.
 *
 * @param magnitude
 *            the magnitude of the current joystick vector
 * @see Joystick.getMagnitude()
 * @param direction
 *            the direction of the current joystick vector
 * @see Joystick.getDirection
 * @param rotation
 *            the amount of rotation we want to apply to the current vecot
 * @see Joystick.getRotation()
 * 
 * @param yValue
 *            the y threshold we want to pass to drive
 * @see Joystick.getY()
 * 
 * @param xValue
 *            the x threshold we want to pass to drive
 * @see Joystick.getX()
 *
 * @author Noah Golmant
 * @param xValue
 * @written 23 July 2015 Edited By Becky Button
 */
public void drive (double magnitude, double direction, double rotation)
{

    double tempRotation = rotation, tempMagnitude = magnitude,
            tempDirection = direction;

    if ((Math.abs(magnitude) >= super.getDeadbandPercentageZone())
            || Math.abs(
                    tempRotation) >= super.getDeadbandPercentageZone())
        {

        if (tempDirection < this.getDirectionalDeadzone()
                && tempDirection > -this.getDirectionalDeadzone())
            tempDirection = 0;
        // else if (tempDirection < 45 + this.getDirectionalDeadzone()
        // && tempDirection > 45 - this.getDirectionalDeadzone())
        // tempDirection = 45;
        else if (tempDirection < 90 + this.getDirectionalDeadzone()
                && tempDirection > 90 - this.getDirectionalDeadzone())
            tempDirection = 90;
        // else if (tempDirection < 135 + this.getDirectionalDeadzone()
        // && tempDirection > 135 - this.getDirectionalDeadzone())
        // tempDirection = 135;
        else if (tempDirection < -180 + this.getDirectionalDeadzone()
                && tempDirection > 180 - this.getDirectionalDeadzone())
            tempDirection = 180;
        // else if (tempDirection < -135 + this.getDirectionalDeadzone()
        // && tempDirection > -135 - this.getDirectionalDeadzone())
        // tempDirection = -135;
        else if (tempDirection < -90 + this.getDirectionalDeadzone()
                && tempDirection > -90 - this.getDirectionalDeadzone())
            tempDirection = -90;
        // else if (tempDirection < -45 + this.getDirectionalDeadzone()
        // && tempDirection > -45 - this.getDirectionalDeadzone())
        // tempDirection = -45;
        //
        // System.out.println("Direction: " + tempDirection);

        // Magnitude and rotation deadzones

        // Deadzone for Rotation
        // if (Math.abs(rotation) > this.getDeadbandPercentageZone() &&
        // tempRotation > 0)
        // {
        // tempRotation = (tempRotation + (tempRotation *
        // this.getDeadbandPercentageZone())) -
        // this.getDeadbandPercentageZone();
        // }else if (Math.abs(rotation) > this.getDeadbandPercentageZone()
        // && tempRotation < 0)
        // {
        // tempRotation = (tempRotation + (tempRotation *
        // this.getDeadbandPercentageZone())) +
        // this.getDeadbandPercentageZone();
        // }else
        // {
        // tempRotation = 0.0;
        // }

        // Deadzone for Magnitude
        // if (Math.abs(magnitude) > this.getDeadbandPercentageZone() &&
        // tempMagnitude > 0)
        // {
        // tempMagnitude = (tempMagnitude + (tempMagnitude *
        // this.getDeadbandPercentageZone())) -
        // this.getCurrentGearPercentage();
        // }else
        // {
        // tempMagnitude = 0.0;
        // }

        if ((this.getDebugState() == DebugState.DEBUG_MOTOR_DATA)
                || (this.getDebugState() == DebugState.DEBUG_ALL))
            {
            System.out
                    .println("MECANUM INPUT:\n" + "Original direction: "
                            + direction + "\tReal direction: "
                            + tempDirection + "\n" + "Magnitude: "
                            + tempMagnitude + "\n" + "Rotation: "
                            + rotation);
            }

        // limit the rotation value between -1 and +1
        tempRotation = this.limit(rotation);

        // check if the joystick is reversed
        if (this.isMecanumJoystickReversed() == true)
            {
            tempRotation *= -1.0;
            tempMagnitude *= -1.0;
            }

        /**
         * MECANUM CONTROLS EXPLANATION
         *
         * First, we apply all of our deadzones and limits. What we start
         * with is a goal vector to travel along, with a direction and a
         * magnitude, i.e. how fast we move along it.
         *
         * We break this vector into its X and Y components, and send these
         * components to their respective motors.
         *
         * The mecanum motors form an X pattern with the angled rollers.
         *
         * LEFT FRONT: \\ RIGHT FRONT: // LEFT REAR: // RIGHT REAR: \\
         *
         * As the "\\" wheels move forward, they go 45 degrees to the right.
         * As the "//" wheels move forward, they go 45 degrees to the left.
         *
         * As an example of how the mecanum can move in any input vector,
         * consider a goal of moving right, with a degree input of +90.0
         * degrees.
         *
         * The correct input degree value, accounting for 45 degree rollers,
         * is 135. cos(135 degrees) = -.707 sin(135 degrees) = +.707
         *
         * If the left front and right rear motors (\\) receive a positive
         * value, they will move forward and to the right.
         *
         * If the right front and left rear motors (//) receive a negative
         * value, they will move backwards and to the right.
         *
         * Since the forward and backwards movements (theoretically) cancel
         * out, the net movement will be completely towards the right.
         *
         * We can also apply a rotation amount to twist the robot as it
         * moves along the goal vector.
         *
         * @author Noah Golmant
         * @written 23 July 2015
         */

        // Add 45 to account for the angle of the rollers
        // on the mecanum wheels.
        final double dirInRad = ((tempDirection + 45.0) * 3.14159)
                / 180.0;
        final double cosD = Math.cos(dirInRad);
        final double sinD = Math.sin(dirInRad);

        // Calculate the speed to send to each motor.
        double leftFrontSpeed = (sinD * tempMagnitude) + tempRotation;
        double rightFrontSpeed = (cosD * tempMagnitude) - tempRotation;
        double leftRearSpeed = (cosD * tempMagnitude) + tempRotation;
        double rightRearSpeed = (sinD * tempMagnitude) - tempRotation;

        if ((this.getDebugState() == DebugState.DEBUG_MOTOR_DATA)
                || (this.getDebugState() == DebugState.DEBUG_ALL))
            {
            System.out.println("MECANUM OUTPUT:\n" + "LF: "
                    + leftFrontSpeed + "\tRF: " + rightFrontSpeed + "\n"
                    + "LR: " + leftRearSpeed + "\tRR: "
                    + rightRearSpeed);
            }

        final double gearPercentage = getFirstGearPercentage();
        // limit the values to our motor range of -1..1
        leftFrontSpeed = this.limit(leftFrontSpeed * gearPercentage);
        leftRearSpeed = this.limit(leftRearSpeed * gearPercentage);
        rightFrontSpeed = this.limit(rightFrontSpeed * gearPercentage);
        rightRearSpeed = this.limit(rightRearSpeed * gearPercentage);

        // scale all of the motor "send" values by our current gear and
        // deadzone.
        leftFrontSpeed = this.scaleJoystickValue(leftFrontSpeed);
        leftRearSpeed = this.scaleJoystickValue(leftRearSpeed);
        rightFrontSpeed = this.scaleJoystickValue(rightFrontSpeed);
        rightRearSpeed = this.scaleJoystickValue(rightRearSpeed);

        // finally, send the scaled values to our motors.
        this.driveLeftMotor(leftFrontSpeed);
        this.driveLeftRearMotor(leftRearSpeed);
        this.driveRightMotor(rightFrontSpeed);
        this.driveRightRearMotor(rightRearSpeed);
        // }
        }
    else
        {
        this.driveLeftMotor(0.0);
        this.driveRightMotor(0.0);
        this.driveLeftRearMotor(0.0);
        this.driveRightRearMotor(0.0);
        }
}

/**
 * Gets the current directional deadzone for the joystick angle.
 *
 * @return current directional deadzone value
 *
 * @author Noah Golmant
 * @written 23 July 2015
 */
public double getDirectionalDeadzone ()
{
    return this.directionalDeadzone;
}

/**
 * Gets whether or not the mecanum joystick is reversed
 *
 * @return true if the joystick is reversed
 */
public boolean isMecanumJoystickReversed ()
{
    return this.mecanumJoystickReversed;
}

/**
 * Sets the deadzone for the direction, in degrees. It will snap to the angle
 * (given in this.drive) if it is +/- the degrees given
 *
 *
 * @author Ryan McGee
 * @param deadzoneDegrees
 *            the degreees that we want the joystick angle to snap to.
 * @written 29 January 2017
 */
public void setDirectionalDeadzone (double deadzoneDegrees)
{
    this.directionalDeadzone = deadzoneDegrees;
}

/**
 * Sets whether or not the mecanum joystick is reversed
 *
 * @param isReversed
 *            true if the joystick is reversed
 */
public void setMecanumJoystickReversed (boolean isReversed)
{
    this.mecanumJoystickReversed = isReversed;
}

/**
 * set drive coefficient to change speed
 * 
 * @param firstGearPercentage
 *            drive coefficient
 * @author Becky Button
 */
public void setFirstGearPercentage (double firstGearPercentage)
{
    this.firstGearPercentage = firstGearPercentage;
}

/**
 * get first gear percentage
 * 
 * @return percentage
 * @author Becky Button
 */
public double getFirstGearPercentage ()
{
    return this.firstGearPercentage;
}

}

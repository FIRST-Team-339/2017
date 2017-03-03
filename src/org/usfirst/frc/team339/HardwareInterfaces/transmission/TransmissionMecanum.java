package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

public class TransmissionMecanum extends TransmissionFourWheel
{

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
public void drive (final double magnitude, final double direction)
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
public void drive (final double magnitude, final double direction,
        final double rotation)
{
    // nasty hack is used to set the temporarily set the magnitude value higher
    // than it usually would be in order to make strafing certain directions
    // work; note: the "nasty nasty hack" should be fixed later
    double tempRotation = rotation, tempMagnitude = magnitude,
            tempDirection = direction, nastyHack = 2.5;

    // checks if the deadbands for magnitude or twist have been exceeded so we
    // know if the robot should move
    if ((Math.abs(magnitude) >= super.getDeadbandPercentageZone())
            || Math.abs(
                    tempRotation) >= super.getDeadbandPercentageZone())
        {


        // "snaps" to a direction of 0 (straight forward) if the input value is
        // within the Directional Deadzone of 0; helps the driver go directly
        // forwards
        if (tempDirection < this.getDirectionalDeadzone()
                && tempDirection > -this.getDirectionalDeadzone())
            tempDirection = 0;

        // else if (tempDirection < 45 + this.getDirectionalDeadzone()
        // && tempDirection > 45 - this.getDirectionalDeadzone())
        // tempDirection = 45;

        // "snaps" to a direction of 90 (straight right) if the input value is
        // within the Directional Deadzone of 90; helps the driver strafe
        // directly right
        else if (tempDirection < 90 + this.getDirectionalDeadzone()
                && tempDirection > 90 - this.getDirectionalDeadzone())
            {
            // TODO nasty nasty hack
            tempMagnitude = nastyHack;
            tempDirection = 90;
            }
        // else if (tempDirection < 135 + this.getDirectionalDeadzone()
        // && tempDirection > 135 - this.getDirectionalDeadzone())
        // tempDirection = 135;

        // "snaps" to a direction of 180 (straight back) if the input value is
        // within the Directional Deadzone of 180; helps the driver back
        // directly back
        else if (tempDirection < -180 + this.getDirectionalDeadzone()
                && tempDirection > 180 - this.getDirectionalDeadzone())
            tempDirection = 180;
        // else if (tempDirection < -135 + this.getDirectionalDeadzone()
        // && tempDirection > -135 - this.getDirectionalDeadzone())
        // tempDirection = -135;

        // "snaps" to a direction of -90 (straight left) if the input value is
        // within the Directional Deadzone of -90; helps the driver strafe
        // directly left
        else if (tempDirection < -90 + this.getDirectionalDeadzone()
                && tempDirection > -90 - this.getDirectionalDeadzone())
            {
            tempDirection = -90;
            // TODO nasty nasty hack
            tempMagnitude = nastyHack;
            }
        // else if (tempDirection < -45 + this.getDirectionalDeadzone()
        // && tempDirection > -45 - this.getDirectionalDeadzone())
        // tempDirection = -45;

        // // Magnitude and rotation deadzones

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


        // prints out select values if we are in a relevant debug state
        if ((this.getDebugState() == DebugState.DEBUG_MOTOR_DATA)
                || (this.getDebugState() == DebugState.DEBUG_ALL))
            {
            System.out
                    .println("MECANUM INPUT:\n" + "Original direction: "
                            + direction + "\tReal direction: "
                            + tempDirection + "\n" + "Magnitude: "
                            + tempMagnitude + "\n" + "Rotation: "
                            + rotation);
            } // if

        // limit the rotation value between -1 and +1
        tempRotation = this.limit(rotation);

        // check if the joystick is reversed
        if (this.isMecanumJoystickReversed() == true)
            {
            tempRotation *= -1.0;
            tempMagnitude *= -1.0;
            } // if

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
        // on the mecanum wheels; defines several function exclusive variables
        final double dirInRad = ((tempDirection + 45.0) * 3.14159)
                / 180.0; // TODO
        final double cosD = Math.cos(dirInRad);
        final double sinD = Math.sin(dirInRad);

        // Calculates the speed to send to each motor.
        // uses sine for the \\ wheels (for clarification of this notation,
        // see Noah's javadoc above); uses cosine for // wheels, then scales
        // them according to the magnitude
        // Also add/ subtracts the rotation value so we can turn while
        // driving in a direction
        // Other notes: because of the 45 degrees added earlier and the
        // nature of trig functions, going straight forward, even at a magnitude
        // of 1, the motors be given about .707. A motor value of 1.0 is only
        // theoretically achievable at a joystick angle of -135, -45, 45, and
        // 135 (assuming a magnitude of 1.0)
        double leftFrontSpeed = (sinD * tempMagnitude) + tempRotation;
        double rightFrontSpeed = (cosD * tempMagnitude) - tempRotation;
        double leftRearSpeed = (cosD * tempMagnitude) + tempRotation;
        double rightRearSpeed = (sinD * tempMagnitude) - tempRotation;


        // prints out select values if we are debugging this part of the code
        if ((this.getDebugState() == DebugState.DEBUG_MOTOR_DATA)
                || (this.getDebugState() == DebugState.DEBUG_ALL))
            {
            System.out.println("MECANUM OUTPUT:\n" + "LF: "
                    + leftFrontSpeed + "\tRF: " + rightFrontSpeed + "\n"
                    + "LR: " + leftRearSpeed + "\tRR: "
                    + rightRearSpeed);
            } // if

        final double gearPercentage = getCurrentGearPercentage();
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
        // if we aren't doing anything else, tell the motors not to move
        this.driveLeftMotor(0.0);
        this.driveRightMotor(0.0);
        this.driveLeftRearMotor(0.0);
        this.driveRightRearMotor(0.0);
        } // else
} // end drive()

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
public void driveNoDeadband (final double magnitude,
        final double direction,
        final double rotation)
{
    double tempSavedDeadband = 0.0;
    double tempSavedAngularDeadband = 0.0;

    // Save the current deadbands
    tempSavedDeadband = this.getDeadbandPercentageZone();
    tempSavedAngularDeadband = this.getDirectionalDeadzone();
    // set the current deadbands to zero so we can drive slowly
    this.setDeadbandPercentageZone(0.0);
    this.setDirectionalDeadzone(0.0);
    // drive without the deadbands
    this.drive(magnitude, direction, rotation);
    // restore the deadbands
    this.setDeadbandPercentageZone(tempSavedDeadband);
    this.setDirectionalDeadzone(tempSavedAngularDeadband);
} // end driveNoDeadBand()

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
    return (this.directionalDeadzone);
} // end getDirectionalDeadzone()

/**
 * get first gear percentage (the coefficient that scales input values (such as
 * joystick) given to the drive function)
 * 
 * @return first gear percentage
 * @author Becky Button
 */
public double getFirstGearPercentage ()
{
    return (this.getGearPercentage(1));
} // end getFirstGearPercentage()

/**
 * Gets whether or not the mecanum joystick is reversed
 *
 * @return true if the joystick is reversed
 */
public boolean isMecanumJoystickReversed ()
{
    return (this.mecanumJoystickReversed);
} // end isMecanumJoystickReversed()

/**
 * Sets the deadzone for the direction, in degrees. It will snap to the angle
 * (given in this.drive) if it is within +/- the degrees given
 *
 *
 * @author Ryan McGee
 * @param deadzoneDegrees
 *            the degreees that we want the joystick angle to snap to.
 * @written 29 January 2017
 */
public void setDirectionalDeadzone (final double deadzoneDegrees)
{
    this.directionalDeadzone = deadzoneDegrees;
} // end setDirectionalDeadzone()

/**
 * sets drive coefficient to change speed
 * 
 * @param firstGearPercentage
 *            drive coefficient
 * @author Becky Button
 */
public void setFirstGearPercentage (final double firstGearPercentage)
{
    this.setGearPercentage(1, firstGearPercentage);
}


/**
 * Sets whether or not the mecanum joystick is reversed
 *
 * @param isReversed
 *            true if the joystick is reversed
 */
public void setMecanumJoystickReversed (final boolean isReversed)
{
    this.mecanumJoystickReversed = isReversed;
} // end setMecanumJoystickReversed()


/**
 * @description If we are within x many degrees of being purely up, down,
 *              left, or right, then we send that "pure" degree value to
 *              account for human error in joystick control.
 *
 * @author Noah Golmant
 * @written 23 July 2015
 */
private double directionalDeadzone = 0.0;

/** Sets whether or not the mecanum control joystick is reversed */
private boolean mecanumJoystickReversed = false;

} // end class TransmissionMechanum

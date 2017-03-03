
// ====================================================================
// FILE NAME: KilroyGyro.java (Team 339 - Kilroy)
//
// CREATED ON: sometime during 2017 build season
// CREATED BY: Alex Kneipp
// MODIFIED ON:2/28/17 and 2/29/17
// MODIFIED BY: Ashley Espeland
// ABSTRACT:
// deals with all of our shooter code


package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import org.usfirst.frc.team339.HardwareInterfaces.IRSensor;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.Spark;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO shooter encoder instead of pot
/**
 * Describes the shooter object for the 2017 game: FIRST Steamworks. It's a
 * flywheel shooter with an elevator loader. Look at the technical packet from
 * 2017 for more information.
 * 
 * @author Alexander H. Kneipp
 *
 */
public class Shooter
{
private CANTalon flywheelController = null;

private IRSensor elevatorSensor = null;

private Victor elevatorController = null;

private double acceptableError = 0;

private ImageProcessor visionTargeter = null;

private double acceptableGimbalError = .5;// in degrees

private CANTalon gimbalMotor = null;

private Spark agitatorMotor = null;

// private Timer shooterTimer = new Timer();

/**
 * Creates a new shooter object for the 2017 season, SteamWorks
 * 
 * @param controller
 *            The motor controller which runs the flywheel.
 * @param ballLoaderSensor
 *            Detects if there's a ball ready to be fired.
 * @param elevator
 *            The motor controller which loads the loader elevator
 * @param acceptableFlywheelSpeedError
 *            The error we can handle on the flywheel without losing
 *            accuracy
 * @param visionTargeting
 *            Our vision processor object, used to target the high boiler.
 * @param gimbalEnc
 *            The potentiometer that reads the bearing of the turret.
 * @param acceptableGimbalError
 *            The acceptable angular angle, in degrees, the gimbal turret is
 *            allowed to be off.
 * @param gimbalMotor
 *            The motor controller the turret is run on
 * @param agitatorMotor
 *            The motor controller the agitator motor is connected to
 */
public Shooter (CANTalon controller, IRSensor ballLoaderSensor,
        Victor elevator, double acceptableFlywheelSpeedError,
        ImageProcessor visionTargeting, double acceptableGimbalError,
        CANTalon gimbalMotor, Spark agitatorMotor)
{
    this.flywheelController = controller;
    this.elevatorSensor = ballLoaderSensor;
    this.elevatorController = elevator;
    this.acceptableError = acceptableFlywheelSpeedError;
    this.visionTargeter = visionTargeting;
    this.gimbalMotor = gimbalMotor;
    this.agitatorMotor = agitatorMotor;
}

/**
 * @param error
 *            The allowed error deadband for our gimbal, in degrees.
 */
public void setAcceptableGimbalError (double error)
{
    this.acceptableGimbalError = error;
}

/**
 * @return The allowed error deadband for our gimbal, in degrees.
 */
public double getAcceptableGimbalError ()
{
    // returns what we consider an acceptable amount of error for the gimbal
    return this.acceptableGimbalError;
}

/**
 * @param error
 *            The acceptable flywheel RPM error at which we will still fire
 *            balls.
 */
public void setAcceptableFlywheelError (double error)
{
    // sets the acceptable error for the flywheel
    this.acceptableError = error;
}

/**
 * @return The acceptable flywheel RPM error at which we will still fire
 *         balls.
 */
public double getAcceptableFlywheelError ()
{
    // returns what we consider an acceptable amount of error for the flywheel
    return this.acceptableError;
}

// TODO write stops for other motors. TODO check that we did that.
/**
 * Stops the flywheel motor.
 */
public void stopFlywheelMotor ()
{
    // stops the flywheel motor
    this.flywheelController.set(0.0);
}

/**
 * Runs the agitator and elevator towards the shooter.
 */
public void loadBalls ()
{
    // load balls by running the elevator and agitator at their assigned speeds
    this.elevatorController.set(ELEVATOR_SPEED);
    this.agitatorMotor.set(AGITATOR_SPEED);
}

/**
 * Stops both the elevator and the agitator.
 */
public void stopLoader ()
{
    // stops loading balls by stopping the elevator and the agitator
    this.elevatorController.set(0.0);
    this.agitatorMotor.set(0.0);
}

/**
 * Runs the elevator backwards and also runs the agitatior.
 */
public void reverseLoader ()
{
    // reverse loads by running the elevator in reverse and the agitator at
    // its normal speed (normal includes its direction)
    this.elevatorController.set(-ELEVATOR_SPEED);
    this.agitatorMotor.set(AGITATOR_SPEED);
}

/**
 * Prepares to fire and fires a ball.
 * 
 * @return True if we've fired, false if we haven't yet.
 * @deprecated Use {@link #fire(double)} instead
 */
public boolean fire ()
{
    // return true if weve fired and false if we havent yet
    return fire(0);
}

/**
 * Prepares to fire and fires a ball.
 * 
 * @param rpmOffset
 *            TODO
 * 
 * @return True if we've fired, false if we haven't yet.
 */
public boolean fire (double rpmOffset)
{
    // System.out.println("RPMOffset in fire: " + rpmOffset);
    readyToFire = prepareToFire(rpmOffset);
    // if readyToFire is equal to false
    if (readyToFire == false)
        {
        // then return false
        return false;
        }
    // if the elevator sensor is on
    if (this.elevatorSensor.isOn() == true)
        {
        // then set the elevator to its assigned speed
        this.elevatorController.set(ELEVATOR_SPEED);
        // return false
        return false;
        }
    // this.elevatorController.set(0);
    // sets readyToFire to false
    readyToFire = false;
    // return true
    return true;

}

private boolean readyToFire = false;

/**
 * Prepares to fire a ball by revving up the flywheel motor and sets up a
 * ball to be fired.
 * 
 * @return true if we're ready to fire, false otherwise.
 * @deprecated Use {@link #prepareToFire(double)} instead
 */
public boolean prepareToFire ()
{
    // return true if were ready to fire fire and false if we arent
    return prepareToFire(0);
}

/**
 * Prepares to fire a ball by revving up the flywheel motor and sets up a
 * ball to be fired.
 * 
 * @param rpmOffset
 * 
 * 
 * @return true if we're ready to fire, false otherwise.
 */
public boolean prepareToFire (double rpmOffset)
{
    // System.out.println("RPMOffset in prepareToFire: " + rpmOffset);
    // dist is the distance to goal
    double dist = 9.25;/*
                        * this.visionTargeter.getZDistanceToFuelTarget(
                        * this.visionTargeter.getLargestBlob());
                        */
    // if the distance to goal is greater than 0
    if ((dist > 0) == true)
        {
        // then set flywheel to half the calculated RPM(to make the goal)
        // plus the rpm offset
        this.flywheelController
                .set(.5 * this.calculateRPMToMakeGoal(dist)
                        + rpmOffset);
        // print to the smartDashboard the flywheel speed
        SmartDashboard.putNumber("Flywheel speed",
                this.flywheelController.getSpeed());
        // calls load balls
        this.loadBalls();
        // divides the absolute value of the flywheel error by four
        // if this value is greater than the acceptable error
        if (Math.abs(this.flywheelController.getError()
                / 4.0) > this.acceptableError)
            {
            // this.stopLoader();
            // returns false
            return false;
            }
        }
    else
        // return false
        return false;
    // if (this.elevatorSensor.isOn())
    // {
    // this.stopLoader();
    // }
    // else
    // {
    // this.loadBalls();
    // return false;
    // }

    // return true
    return true;
}

/**
 * Turns the turret to the new bearing on the robot.
 * 
 * @param newBearing
 *            The new angle, relative to the robot, to turn the turret to.
 *            (- is left, + is to the right)
 * @return SUCCESS if we're at our angle (within our threshold), TOO_FAR if
 *         we can't gimbal that much, or WORKING if we're not done yet.
 */
// TODO slow down as we approach it
public turnReturn turnToBearing (double newBearing)
{
    // if the absolute value of the difference between the new bearing and
    // the current bearing is less than or equal to the acceptable Gimbal error
    if (Math.abs(
            newBearing - this.getBearing()) >= acceptableGimbalError)
        {
        // if the difference between the newBearing and the current bearing is
        // less than 0
        if (newBearing - getBearing() < 0)
            {
            // returns turnGimbal(-MEDIUM_TURN_SPEED)
            // AKA negative medium turn speed
            return this.turnGimbal(-MEDIUM_TURN_SPEED);
            }
        // returns turnGimbal(MEDIUM_TURN_SPEED)
        return this.turnGimbal(MEDIUM_TURN_SPEED);

        }
    // stops gimbal
    this.stopGimbal();
    // returns turnReturn.SUCCESS
    return turnReturn.SUCCESS;
}

/**
 * Sets the gimbal motor to 0. Call after any turnGimbal function (except
 * turnToBearing)
 */
public void stopGimbal ()
{
    // sets the gimbal to 0
    this.turnGimbal(0.0);
}

/**
 * Turns the gimbal at our slow speed.
 * 
 * run stopGimbal afterwards
 * 
 * @param direction
 *            Negative 1 or positive 1, positive for right, negative for
 *            left
 * @return see turnGimbal(double)
 */
public turnReturn turnGimbalSlow (int direction)
{
    // returns turnGimbal(direction * SLOW_TURN_SPEED)
    return this.turnGimbal(direction * SLOW_TURN_SPEED);
}

/**
 * Turns the gimbal at our medium speed.
 * 
 * run stopGimbal afterwards
 * 
 * @param direction
 *            Negative 1 or positive 1, positive for right, negative for
 *            left
 * @return see turnGimbal(double)
 */
public turnReturn turnGimbalMedium (int direction)
{
    // returns turnGimbal(direction * MEDIUM_TURN_SPEED)
    return this.turnGimbal(direction * MEDIUM_TURN_SPEED);
}

/**
 * Turns the turrets as quickly as we can.
 * 
 * run stopGimbal afterwards
 * 
 * @param direction
 *            Negative 1 or positive 1, positive for right, negative for
 *            left
 * @return see turnGimbal(double)
 */
public turnReturn turnGimbalFast (int direction)
{
    // returns turnGimbal(direction * MAX_TURN_SPEED)
    return this.turnGimbal(direction * MAX_TURN_SPEED);
}

/**
 * Turns the gimbal if we're allowed to.
 * 
 * @param speed
 *            The speed and direction we turn the gimbal at (+ right, -
 *            left)s
 * @return WORKING if we're turning, TOO_FAR if we're at or have passed our
 *         limit.
 */
private turnReturn turnGimbal (double speed)
{
    // if the bearing is greater than or equal to the max gimbaling angle
    // and the speed is greater than 0
    // OR if the bearing is less than or equal the max gimbaling angle
    // and the speed is less than 0
    if ((this.getBearing() >= MAX_GIMBALING_ANGLE && speed > 0)
            || (this.getBearing() <= MIN_GIMBALING_ANGLE && speed < 0))
        {
        // set gimbal motor to 0
        this.gimbalMotor.set(0.0);
        // return turnReturn.TOO_FAR
        return turnReturn.TOO_FAR;
        }
    // TODO direction
    /*
     * Make sure we never turn faster than the maximum speed.
     * ALSO motor is reversed so... that's why it's like that.
     */
    // if the speed is less than 0
    if (speed < 0)
        {
        // then set to either the speed or the negative max turn speed,
        // based on which is greater
        this.gimbalMotor.set(
                Math.max(speed, -MAX_TURN_SPEED));
        }
    // else id speed is greater than 0
    else if (speed > 0)
        {
        // set the gimbal motor to the higher value of either the speed
        // or the Max turn speed
        this.gimbalMotor.set(
                Math.min(speed, MAX_TURN_SPEED));
        }
    // else
    else
        {
        // set the gimbal motor to 0 (SSSSTOPPPP)
        this.gimbalMotor.set(0.0);
        }
    // return turnReturn.WORKING
    return turnReturn.WORKING;
}

/**
 * @return The bearing of the shooter relative to the robot, with negative
 *         degrees to the left and positive to the right. Returns a range
 *         0 to MAX_
 */
public double getBearing ()
{
    // return the product of the encoder position and the gimbal encoder factor
    return this.gimbalMotor.getEncPosition()
            * this.GIMBAL_ENCODER_FACTOR;
}

/**
 * Returned from turnToBearing as well as the turn slow, medium, and fast
 * functions. Provides additional information as to the state of the
 * function.
 * 
 * @author Alexander H. Kneipp
 */
public static enum turnReturn
    {
    /**
     * We cannot gimbal as far as the user wants us to. Quit state.
     */
    TOO_FAR,
    /**
     * We're still aligning, be patient! Not a quit state.
     */
    WORKING,
    /**
     * We've successfully turned to the target position, good job robot.
     * Quit state.
     */
    SUCCESS
    }

/**
 * Uses the vision processing to align the gimbal to the high boiler.
 * 
 * @return True if we're aligned, false otherwise.
 */
// TODO Radians and degrees
public turnToGoalReturn turnToGoal ()
{
    // if we are running this for the first time
    if (firstTimeRun == true)
        {
        // then process the image
        this.visionTargeter.processImage();
        // set firstTimeRun to false
        firstTimeRun = false;
        }
    // if the getLargetBlob is not equal to null
    if (this.visionTargeter.getLargestBlob() != null)
        {
        // if the turnToBearing (ehich is the direction to the largest
        // blob) is equalt to what we consider a "success"
        if (this.turnToBearing(
                Math.toDegrees(this.visionTargeter
                        .getYawAngleToTarget(this.visionTargeter
                                .getLargestBlob()))) == turnReturn.SUCCESS)
            {
            // return turnToGoalReturn.SUCCESS
            return turnToGoalReturn.SUCCESS;
            }
        }
    else
        {
        // process image
        this.visionTargeter.processImage();
        // return turnToGoalReturn.NO_BLOBS
        return turnToGoalReturn.NO_BLOBS;
        }

    // return turnToGoalReturn.WORKING
    return turnToGoalReturn.WORKING;



}

public boolean turnToGoalRaw ()
{
    // process image
    this.visionTargeter.processImage();
    // if the getLargestBlob is not equal to null
    if (this.visionTargeter.getLargestBlob() != null)
        {
        // if the absolute value of the center of the largest blob
        // (x value) divided by the horizontal resolution minus .5 is
        // less than turn to goal raw deadband
        if ((Math.abs(this.visionTargeter.getLargestBlob().center_mass_x
                / this.visionTargeter.camera.getHorizontalResolution()
                - centerXLineOfImage) <= TURN_TO_GOAL_RAW_DEADBAND) == true)
            {
            // return stopGimbal
            this.stopGimbal();
            // return true
            return true;
            }
        // if the center of the largest blob(x value) divided by the
        // horizontal resolution is greater than .5
        // (meaning it is too far to the right)
        if (this.visionTargeter.getLargestBlob().center_mass_x
                / this.visionTargeter.camera
                        .getHorizontalResolution() > centerXLineOfImage)
            {
            // turn the gimbal to the left at -1
            this.turnGimbalMedium(-1);
            // return false
            return false;
            }
        // if the center of the largest blob (x value) divided by the
        // horizontal resolution is greater than the center of the image
        if (this.visionTargeter.getLargestBlob().center_mass_x
                / this.visionTargeter.camera
                        .getHorizontalResolution() < centerXLineOfImage)
            {
            // turn gimbal at medium speed to the right
            this.turnGimbalMedium(1);
            // return false
            return false;
            }
        }
    return false;
}

private double centerXLineOfImage = .5;

private boolean isTurningToGoal = false;

private turnReturn turningToGoalVal = turnReturn.SUCCESS;

private boolean firstTimeRun = false;

private double gimbalTarget = Double.MIN_VALUE;

/**
 * 
 * Return values for turnToGoal(), indicates different failure states and
 * working state.
 * 
 * @author Alexander H. Kneipp
 *
 */
// TODO decide about whether we quit on one blob.
public static enum turnToGoalReturn
    {
    /**
     * We don't see any blobs. Quit state.
     */
    NO_BLOBS,
    /**
     * We see 1 blob, not enough to align. Quit state.
     */
    NOT_ENOUGH_BLOBS,
    /**
     * We have successfully aligned. Quit state.
     */
    SUCCESS,
    /**
     * Not currently used, but may be used if this often runs "working" for
     * way to long. Quit state.
     */
    TIMEOUT,
    /**
     * Still aligning to target. NOT a quit state.
     */
    WORKING,
    /**
     * We see the target, but we can't gimbal to it. The robot will need to
     * turn. Quit state.
     */
    OUT_OF_GIMBALING_RANGE
    }

/**
 * Uses physics to figure out how fast we need to spin the flywheel. Uses
 * the constant FLYWHEEL_SPEED_CORRECTION_CONSTANT to account for friction,
 * air resistance, and calculation imprecision. Change if you're shooting
 * too far or too close. Increase to increase shooting distance, decrese to
 * decrease.
 * 
 * @param distance
 *            The distance, in feet, the shooter is away from the goal
 *            across the floor.
 * @return the RPM to run the flywheel wheel at.
 */
public double calculateRPMToMakeGoal (double distance)
{
    // changes the distance into meters by multiplying by a the ratio
    double distanceMeters = distance * 3.28084;// Convert the distance
                                               // parameter
                                               // meters, for easier
                                               // computations.
    // calculates the "perfect" RPM to set the flywheel to, by calculating
    double perfectRPM = (60.0 / (2 * Math.PI) * (Math.sqrt(
            ((4.9 * (Math.pow(distanceMeters, 2)))
                    / ((this.FLYWHEEL_RADIUS_METERS
                            * this.FLYWHEEL_RADIUS_METERS)
                            * (Math.pow(Math.cos(
                                    Math.toRadians(this.MOUNT_ANGLE)),
                                    2))
                            * (distanceMeters * Math.tan(
                                    Math.toRadians(this.MOUNT_ANGLE))
                                    - this.RELATIVE_GOAL_HEIGHT_METERS))))));
    // return perfectRPM plus the flywheel correction speed constant
    // multiplied by the perfect RPM
    return perfectRPM
            + this.FLYWHEEL_SPEED_CORRECTION_CONSTANT * perfectRPM;
}
// ---------------------------------------------------------------------
// variables
// ---------------------------------------------------------------------

private final double MAX_TURN_SPEED = .5;

private final double MEDIUM_TURN_SPEED = .35;

private final double SLOW_TURN_SPEED = .25;

private final double TURN_TO_GOAL_RAW_DEADBAND = .075;

private final double ELEVATOR_SPEED = 1.0;// .8

private final double AGITATOR_SPEED = .5;

private final double GIMBAL_LEFT_OFFSET = .1;// Going left is slower than going
                                             // right for some reason

public final double MAX_GIMBALING_ANGLE = 16;// in degrees

public final double MIN_GIMBALING_ANGLE = -16;// in degrees

private final double MOUNT_ANGLE = 60;// TODO figure out the actual number.

private final double RELATIVE_GOAL_HEIGHT_METERS = 1.93;

private final double FLYWHEEL_RADIUS_METERS = 0.0508;

private final double FLYWHEEL_SPEED_CORRECTION_CONSTANT = -.13578;// TODO tune

/**
 * factor the gimbal encoder must be set to (distance per pulse)
 * for degrees
 */
private final double GIMBAL_ENCODER_FACTOR = .00022448;
}

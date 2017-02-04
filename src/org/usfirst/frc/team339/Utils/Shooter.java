package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import org.usfirst.frc.team339.HardwareInterfaces.IRSensor;
import org.usfirst.frc.team339.HardwareInterfaces.Potentiometer;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Describes the shooter object for the 2017 game: FIRST Steamworks.
 * It's a flywheel shooter with an elevator loader. Look at the technical packet
 * from 2017 for more information.
 * 
 * @author Alexander H. Kneipp
 *
 */
public class Shooter
{
private CANTalon flywheelController = null;

private IRSensor elevatorSensor = null;

private PWMSpeedController elevatorController = null;

private double acceptableError = 0;

private ImageProcessor visionTargeter = null;

private Potentiometer gimbalPot = null;

private double acceptableGimbalError = 0;

private PWMSpeedController gimbalMotor = null;

private Timer shooterTimer = new Timer();

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
 *            The error we can handle on the flywheel without losing accuracy
 * @param visionTargeting
 *            Our vision processor object, used to target the high boiler.
 * @param gimbalPot
 *            The potentiometer that reads the bearing of the turret.
 * @param acceptableGimbalError
 *            The acceptable angular angle, in degrees, the gimbal turret is
 *            allowed to be off.
 * @param gimbalMotor
 *            TODO
 */
public Shooter (CANTalon controller, IRSensor ballLoaderSensor,
        PWMSpeedController elevator,
        double acceptableFlywheelSpeedError,
        ImageProcessor visionTargeting, Potentiometer gimbalPot,
        double acceptableGimbalError, PWMSpeedController gimbalMotor)
{
    this.flywheelController = controller;
    this.elevatorSensor = ballLoaderSensor;
    this.elevatorController = elevator;
    this.acceptableError = acceptableFlywheelSpeedError;
    this.visionTargeter = visionTargeting;
    this.gimbalPot = gimbalPot;
    this.gimbalMotor = gimbalMotor;
}

/**
 * Prepares to fire and fires a ball.
 * 
 * @return
 *         True if we've fired, false if we haven't yet.
 */
public boolean fire ()
{
    if (prepareToFire())
        {
        if (this.elevatorSensor.isOn())
            {
            this.elevatorController.set(ELEVATOR_SPEED);
            return false;
            }
        this.elevatorController.set(0);
        return true;
        }
    return false;
}

/**
 * Prepares to fire a ball by revving up the flywheel motor and sets up a ball
 * to be fired.
 * 
 * @return
 *         true if we're ready to fire, false otherwise.
 */
public boolean prepareToFire ()
{
    boolean retVal;
    this.flywheelController.setSetpoint(2500);// TODO read distance/lookup
                                              // table/whatever.
    if (Math.abs(
            this.flywheelController.getError()) > this.acceptableError)
        {
        retVal = false;
        }
    else
        {
        retVal = true;
        }
    if (this.elevatorSensor.isOn())
        {
        retVal = retVal && true;
        }
    else
        {
        this.elevatorController.set(ELEVATOR_SPEED);// TODO magic number
        retVal = false;
        }
    return retVal;
}

/**
 * Turns the turret to the new bearing on the robot.
 * 
 * @param newBearing
 *            The new angle, relative to the robot, to turn the turret to. (- is
 *            left, + is to the right)
 * @return
 *         True if we're at the target angle, false otherwise.
 */
// TODO slow down as we approach it
public boolean turnToBearing (double newBearing)
{
    if (Math.abs(newBearing - getBearing()) >= acceptableGimbalError)
        {
        /*
         * TODO magic speed number and unsure about direction, but it will
         * attempt to turn towards the error.
         */
        this.gimbalMotor.set(((newBearing - getBearing())
                / Math.abs(newBearing - getBearing()))
                * MAX_TURN_SPEED);
        return false;
        }
    return true;
}

/**
 * Uses the vision processing to align the gimbal to the high boiler.
 * 
 * @return
 *         True if we're aligned, false otherwise.
 */
public turnToGoalReturn turnToGoal ()
{
    if (this.visionTargeter.getNthSizeBlob(0) != null)
        {
        if (this.visionTargeter.getNthSizeBlob(1) != null)
            {
            if (gimbalTarget == Double.MIN_VALUE)
                {
                gimbalTarget = this.visionTargeter.getYawAngleToTarget(
                        this.visionTargeter.getNthSizeBlob(0));
                }
            else
                {
                if (turnToBearing(gimbalTarget + this.getBearing()))
                    {
                    gimbalTarget = Double.MIN_VALUE;
                    return turnToGoalReturn.SUCCESS;
                    }
                }
            return turnToGoalReturn.WORKING;
            }
        return turnToGoalReturn.NOT_ENOUGH_BLOBS;
        }
    return turnToGoalReturn.NO_BLOBS;

}


private boolean firstTimeRun = false;

private double gimbalTarget = Double.MIN_VALUE;

public static enum turnToGoalReturn
    {
    NO_BLOBS, NOT_ENOUGH_BLOBS, SUCCESS, TIMEOUT, WORKING
    }

/**
 * @return
 *         The bearing of the shooter relative to the robot, with negative
 *         degrees to the left and positive to the right. Returns a range from
 *         -gimbalPot maxDegrees/2 to +gimbalPot maxDegrees/2
 */
public double getBearing ()
{
    // normalizes the bearing from -gimbalPot.getMaxDegrees()/2 to
    // gimbalPot.getMaxDegrees()/2
    return this.gimbalPot.get() - (this.gimbalPot.getMaxDegrees() / 2);
}

private final double MAX_TURN_SPEED = .5;

private final double ELEVATOR_SPEED = .4;// TODO tune
}

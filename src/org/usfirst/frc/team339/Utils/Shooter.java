package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import org.usfirst.frc.team339.HardwareInterfaces.IRSensor;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.PWMSpeedController;

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

// TODO generalize to the PWM class or whatever
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
 */
public Shooter (CANTalon controller, IRSensor ballLoaderSensor,
        PWMSpeedController elevator,
        double acceptableFlywheelSpeedError,
        ImageProcessor visionTargeting)
{
    this.flywheelController = controller;
    this.elevatorSensor = ballLoaderSensor;
    this.elevatorController = elevator;
    this.acceptableError = acceptableFlywheelSpeedError;
    this.visionTargeter = visionTargeting;
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
            this.elevatorController.set(.4);
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
        this.elevatorController.set(.4);// TODO magic number
        retVal = false;
        }
    return retVal;
}

/**
 * 
 * @param newBearing
 * @return
 */
public boolean turnToBearing (double newBearing)
{
    return false;
}

/**
 * Uses the vision processing to align the gimbal to the high boiler.
 * 
 * @return
 *         True if we're aligned, false otherwise.
 */
public boolean turnToGoal ()
{
    return false;
}
}

package org.usfirst.frc.team339.Utils;

import java.util.Timer;
import java.util.TimerTask;
import org.usfirst.frc.team339.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMSpeedController;

/**
 * A class made to simplify the actions of the Gear Pickup Mechanism.
 * 
 * @author Ryan McGee
 *
 */
public class GearIntake
{
/**
 * The amount of time, in milliseconds, to stop reversing the wheels
 * after ejecting the gear.
 */
private final long GEAR_EJECT_WHEEL_CUTOFF_DELAY = 500;// .5 seconds

private final double GEAR_INTAKE_WHEEL_SPEED = .3;

private PWMSpeedController intakeMotor = null;

private DoubleSolenoid intakeArm = null;

private Timer ejectTimer = new Timer();


/**
 * Creates the Gear Pickup Mechanism object.
 * 
 * @param intakeMotor
 * @param intakeArm
 */
public GearIntake (PWMSpeedController intakeMotor,
        DoubleSolenoid intakeArm)
{
    this.intakeMotor = intakeMotor;
    this.intakeArm = intakeArm;
}

/**
 * Brings the arm down, so it can pick up a gear off of the floor.
 */
public void lowerArm ()
{
    this.intakeArm.setReverse(false);
}

/**
 * Picks up the arm (hopefully with a gear inside) in order to drop it off at
 * the peg.
 * 
 * @param button
 *            The button that enables this function.
 * @return
 *         Whether or not the specified button is pressed, for logic purposes.
 */
public void raiseArm ()
{
    this.intakeArm.setReverse(true);
}

/**
 * Spins the intake wheels so that we can pick up a gear.
 * 
 * @param button
 *            The button that enables this function.
 * @return
 *         Whether or not the specified button is pressed, for logic purposes.
 */
public boolean runIntakeWheels (boolean button)
{
    if (button)
        this.intakeMotor.set(-GEAR_INTAKE_WHEEL_SPEED);

    isRunningWheels = button;
    return button;
}

private boolean isRunningWheels = false;

/**
 * Spins the intake wheels in the opposite direction, in order to spit out a
 * gear.
 * 
 * @param button
 *            The button that enables this function.
 * @return
 *         Whether or not the specified button is pressed, for logic purposes.
 */
public boolean reverseIntakeWheels (boolean button)
{
    if (button)
        this.intakeMotor.set(GEAR_INTAKE_WHEEL_SPEED);

    isReversingWheels = button;
    return button;
}

private boolean isReversingWheels = false;

/**
 * Stops all movement in the intake wheels.
 * 
 * @param button
 *            The button that enables this function.
 * @return
 *         Whether or not the specified button is pressed, for logic purposes.
 */
public void stopIntakeWheels ()
{
    this.intakeMotor.set(0.0);
}

/**
 * Brings down the arm and reverses the intake wheels for half a second,
 * in order to properly place the gear on the peg.
 * 
 * @param button
 *            The button that will eject the gear.
 *            If used autonomously, set to true.
 * @param hasGear
 *            The output from the gear sensor
 * @return
 *         Whether or not the specified button is pressed, for logic purposes.
 */
public boolean activate (boolean button, boolean hasGear)
{
    this.isActivating = button;

    if (timerHasEllapsed == false)
        {
        this.lowerArm();
        return button;
        }

    // Start the intake wheels and schedule the wheel cutoff
    // ONLY on the first start, and we have a gear.
    if (hasGear == true && button == true
            && ejectGearLastAction == false)
        {
        this.reverseIntakeWheels(true);
        this.timerHasEllapsed = false;
        // Schedule the wheels to stop after 500 milliseconds, or .5
        // seconds, in a separate thread.
        this.ejectTimer.schedule(new TimerTask()
        {
        @Override
        public void run ()
        {
            timerHasEllapsed = true;
        }

        }, this.GEAR_EJECT_WHEEL_CUTOFF_DELAY);
        }
    else if (button == true && hasGear == false)
        {
        this.runIntakeWheels(true);
        }
    else if (button == true)// If we do have a gear, then stop the wheels
        {
        isActivating = false;
        }
    // MUST RUN OUTSIDE IF STATEMENT! ...to correctly reset whether it is
    // the first-time run.
    ejectGearLastAction = button;


    if (button)
        this.lowerArm();

    return button;
}

private boolean ejectGearLastAction = false;

private boolean timerHasEllapsed = true;

private boolean isActivating = false;

/**
 * Looks at all the methods being called and only sets the intake
 * wheels to 0 if they need to be.
 */
public void controlStopping ()
{
    if (!isActivating && !isReversingWheels && !isRunningWheels)
        this.stopIntakeWheels();
}


}

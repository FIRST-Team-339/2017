package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Timer;

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

private Timer intakeTimer = new Timer();

public enum LowerArmState
    {
INIT, LOWER_AND_REVERSE, LOWER_AND_FORWARD
    }

private LowerArmState lowerArmState = LowerArmState.LOWER_AND_REVERSE;

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
 */
public void raiseArm ()
{
    this.intakeArm.setReverse(true);
}

/**
 * Spins the intake wheels so that we can pick up a gear.
 */
public void runIntakeWheels ()
{
    this.intakeMotor.set(-GEAR_INTAKE_WHEEL_SPEED);
}

private boolean isRunningWheels = false;

/**
 * Spins the intake wheels in the opposite direction, in order to spit out a
 * gear.
 * 
 */
public void reverseIntakeWheels ()
{
    this.intakeMotor.set(GEAR_INTAKE_WHEEL_SPEED);
}

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
 * Lowers the arm and ejects gear for a certain amount of time, and then
 * starts running the wheels forward.
 * If the sensor reads there is a gear, stop the intake. Else, run them
 * forwards.
 * 
 * @param sensor
 *            Decides whether there is a gear in the robot (true) or not (false)
 */
private void activateArm (boolean sensor)
{
    // Resets the state machine after the person lets go of the button.
    if (!actArmRunOnce)
        lowerArmState = LowerArmState.INIT;

    // Main state machine for lowering the arm.
    switch (lowerArmState)
        {
        // Starts and resets the timer.
        case INIT:
            intakeTimer.reset();
            intakeTimer.start();

            lowerArmState = LowerArmState.LOWER_AND_REVERSE;
            break;
        // The state that lowers the arm, and reverses the intake (for x
        // seconds)
        case LOWER_AND_REVERSE:
            // Go to next state if time has elapsed.
            if (this.intakeTimer.get() > GEAR_EJECT_WHEEL_CUTOFF_DELAY)
                {
                lowerArmState = LowerArmState.LOWER_AND_FORWARD;
                break;
                }

            this.lowerArm();
            this.reverseIntakeWheels();

            break;

        default:
            // The state that lowers the arm and runs the intake "in"
        case LOWER_AND_FORWARD:
            // If the sensor does not see a gear, intake and lower arm.
            if (!sensor)
                {
                this.runIntakeWheels();
                this.lowerArm();
                }
            // If it does, stop wheels and lower arm.
            else
                {
                this.stopIntakeWheels();
                this.lowerArm();
                }
        }

}

private boolean actArmRunOnce = false;

/**
 * Runs the Gear Intake Mechanism based on the buttons input.
 * 
 * @param lowerArmButton
 *            Will lower the arm, spit out the gear for GEAR_EJECT_WHEEL_CUTOFF
 *            seconds, and begin running the wheels based on the gear sensor.
 * @param spinWheelsButton
 *            Runs the intake wheels to intake a gear
 * @param reverseWheelsButton
 *            Reverses the intake wheels to spit out the gear.
 * @param sensor
 *            The sensor that determines if we have a gear.
 */
public void runGearIntake (boolean lowerArmButton,
        boolean spinWheelsButton, boolean reverseWheelsButton,
        boolean sensor)
{
    // Run the "lower arm and wheels" function
    if (lowerArmButton)
        {
        this.activateArm(sensor);
        }
    // Intake the gear
    else if (spinWheelsButton)
        {
        this.runIntakeWheels();
        }
    // Eject the gear
    else if (reverseWheelsButton)
        {
        this.reverseIntakeWheels();
        }
    // Raise the arm and stop the wheels.
    else
        {
        this.raiseArm();
        this.stopIntakeWheels();
        }

    // Makes sure that the "activate arm" only initializes on the first
    // time the button is pushed.
    actArmRunOnce = lowerArmButton;

}



}

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
private final double GEAR_EJECT_WHEEL_CUTOFF_DELAY = .8;// .5 seconds

private final double GEAR_INTAKE_WHEEL_SPEED = .5;

private final double GEAR_OUTPUT_WHEEL_SPEED = .25;

private PWMSpeedController intakeMotor = null;

private DoubleSolenoid intakeArm = null;

private Timer intakeTimer = new Timer();

public enum ArmState
    {
LOWER_AND_IN, LOWER_AND_OUT, LOWER_AND_OFF, LOWER_OVERRIDE_OUT, LOWER_OVERRIDE_IN, UP_AND_OFF, LOWER_AND_AUTO_INIT
    }

private ArmState mainArmState = ArmState.UP_AND_OFF;

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
    this.intakeMotor.set(GEAR_OUTPUT_WHEEL_SPEED);
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


private boolean armRunOnce = false;

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
    // override pull in
    if (lowerArmButton && spinWheelsButton)
        {
        this.mainArmState = ArmState.LOWER_OVERRIDE_IN;
        }
    // override push out
    else if (lowerArmButton && reverseWheelsButton)
        {
        this.mainArmState = ArmState.LOWER_OVERRIDE_OUT;
        }
    // Eject the gear then run intake
    else if (lowerArmButton && armRunOnce == false)
        {
        this.mainArmState = ArmState.LOWER_AND_AUTO_INIT;
        }
    // Raise the arm and stop the wheels.
    else if (lowerArmButton && mainArmState != ArmState.LOWER_AND_IN
            && mainArmState != ArmState.LOWER_AND_OUT
            && mainArmState != ArmState.LOWER_AND_AUTO_INIT)
        {
        this.mainArmState = ArmState.LOWER_AND_OFF;
        }
    else if (!lowerArmButton)
        {
        this.mainArmState = ArmState.UP_AND_OFF;
        }

    switch (mainArmState)
        {
        // Lowers the arm and pulls in the gear, regardless of the sensor.
        case LOWER_OVERRIDE_IN:
            this.lowerArm();
            this.runIntakeWheels();
            break;
        // Lowers the arm and pushes out the gear
        case LOWER_OVERRIDE_OUT:
            this.lowerArm();
            this.reverseIntakeWheels();
            break;
        // Start the timer for the "push out, then pull in" function
        case LOWER_AND_AUTO_INIT:
            this.intakeTimer.reset();
            this.intakeTimer.start();
            this.mainArmState = ArmState.LOWER_AND_OUT;
            // CONTINUE TO NEXT STATE
            // Push out until the timer trips
        case LOWER_AND_OUT:
            this.lowerArm();
            this.reverseIntakeWheels();
            if (this.intakeTimer.get() > GEAR_EJECT_WHEEL_CUTOFF_DELAY)
                {
                this.intakeTimer.stop();
                this.mainArmState = ArmState.LOWER_AND_IN;
                }
            break;
        // Pulls in the intake wheels based on the sensor.
        case LOWER_AND_IN:
            this.lowerArm();
            if (sensor)
                {
                this.stopIntakeWheels();
                }
            else
                {
                this.runIntakeWheels();
                }
            break;
        // Lowers the arm and turns off the intake wheels.
        case LOWER_AND_OFF:
            this.lowerArm();
            this.stopIntakeWheels();
            break;
        // Brings up the gear, and turns off the intake wheels
        default:
        case UP_AND_OFF:
            this.raiseArm();
            this.stopIntakeWheels();
            break;
        }

    // Makes sure that the "activate arm" only initializes on the first
    // time the button is pushed.
    armRunOnce = lowerArmButton;

}



}

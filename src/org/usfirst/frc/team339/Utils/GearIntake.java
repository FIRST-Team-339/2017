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
	public GearIntake(PWMSpeedController intakeMotor, DoubleSolenoid intakeArm)
	{
		this.intakeMotor = intakeMotor;
		this.intakeArm = intakeArm;
	}

	/**
	 * Brings the arm down, so it can pick up a gear off of the floor.
	 * @param button The button that enables this function.
	 * @return
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean lowerArm(boolean button)
	{
		if (button)
			this.intakeArm.setReverse(false);
		return button;
	}

	/**
	 * Picks up the arm (hopefully with a gear inside) in order to drop it off at the peg.
	 * @param button The button that enables this function.
	 * @return
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean raiseArm(boolean button)
	{
		if (button)
			this.intakeArm.setReverse(true);
		return button;
	}

	/**
	 * Spins the intake wheels so that we can pick up a gear.
	 * @param button The button that enables this function.
	 * @return
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean runIntakeWheels(boolean button)
	{
		if (button)
			this.intakeMotor.set(-GEAR_INTAKE_WHEEL_SPEED);
		return button;
	}

	/**
	 * Spins the intake wheels in the opposite direction, in order to spit out a gear.
	 * @param button The button that enables this function.
	 * @return
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean reverseIntakeWheels(boolean button)
	{
		if (button)
			this.intakeMotor.set(GEAR_INTAKE_WHEEL_SPEED);
		return button;
	}

	/**
	 * Stops all movement in the intake wheels.
	 * @param button The button that enables this function.
	 * @return
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean stopIntakeWheels(boolean button)
	{
		if (button)
			this.intakeMotor.set(0.0);
		return button;
	}

	/**
	 * Brings down the arm and reverses the intake wheels for half a second,
	 *  in order to properly place the gear on the peg.
	 * @param button 
	 * 			The button that will eject the gear.
	 * 			If used autonomously, set to true.
	 * @return 
	 * 			Whether or not the specified button is pressed, for logic purposes.
	 */
	public boolean ejectGear(boolean button)
	{
		// Start the intake wheels and schedule the wheel cutoff
		// ONLY on the first start.
		if (button == true && ejectGearLastAction == false)
		{
			this.reverseIntakeWheels(true);
			// Schedule the wheels to stop after 500 milliseconds, or .5
			// seconds, in a separate thread.
			this.ejectTimer.schedule(new TimerTask()
			{
				@Override
				public void run()
				{
					stopIntakeWheels(true);
				}

			}, this.GEAR_EJECT_WHEEL_CUTOFF_DELAY);
		}
		// MUST RUN OUTSIDE IF STATEMENT! ...to correctly reset whether it is
		// the first-time run.
		ejectGearLastAction = button;

		if (button)
			this.lowerArm(true);
		return button;
	}

	private boolean ejectGearLastAction = false;

}

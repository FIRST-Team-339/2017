package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import org.usfirst.frc.team339.HardwareInterfaces.IRSensor;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

// TODO shooter encoder instead of pot
/**
 * Describes the shooter object for the 2017 game: FIRST Steamworks. It's a
 * flywheel shooter with an elevator loader. Look at the technical packet from
 * 2017 for more information.
 * 
 * @author Alexander H. Kneipp
 *
 */
public class Shooter {
	private CANTalon flywheelController = null;

	private IRSensor elevatorSensor = null;

	private Victor elevatorController = null;

	private double acceptableError = 0;

	private ImageProcessor visionTargeter = null;

	private double acceptableGimbalError = 3;//in degrees

	private CANTalon gimbalMotor = null;

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
	 *            TODO
	 */
	public Shooter(CANTalon controller, IRSensor ballLoaderSensor, Victor elevator, double acceptableFlywheelSpeedError,
			ImageProcessor visionTargeting, double acceptableGimbalError, CANTalon gimbalMotor) {
		this.flywheelController = controller;
		this.elevatorSensor = ballLoaderSensor;
		this.elevatorController = elevator;
		this.acceptableError = acceptableFlywheelSpeedError;
		this.visionTargeter = visionTargeting;
		this.gimbalMotor = gimbalMotor;
		
		
	}

	/**
	 * @param error
	 *            The allowed error deadband for our gimbal, in degrees.
	 */
	public void setAcceptableGimbalError(double error) {
		this.acceptableGimbalError = error;
	}

	/**
	 * @return The allowed error deadband for our gimbal, in degrees.
	 */
	public double getAcceptableGimbalError() {
		return this.acceptableGimbalError;
	}

	/**
	 * @param error
	 *            The acceptable flywheel RPM error at which we will still fire
	 *            balls.
	 */
	public void setAcceptableFlywheelError(double error) {
		this.acceptableError = error;
	}

	/**
	 * @return The acceptable flywheel RPM error at which we will still fire
	 *         balls.
	 */
	public double getAcceptableFlywheelError() {
		return this.acceptableError;
	}

	// TODO write stops for other motors.
	public void stopFlywheelMotor() {
		this.flywheelController.set(0.0);
	}

	public void loadBalls() {
		this.elevatorController.set(ELEVATOR_SPEED);
	}

	public void stopLoader() {
		this.elevatorController.set(0.0);
	}

	public void reverseLoader() {
		this.elevatorController.set(-ELEVATOR_SPEED);
	}

	/**
	 * Prepares to fire and fires a ball.
	 * 
	 * @return True if we've fired, false if we haven't yet.
	 */
	public boolean fire() {
		if (!readyToFire) {
			readyToFire = prepareToFire();
			return false;
		}

		if (this.elevatorSensor.isOn()) {
			this.elevatorController.set(ELEVATOR_SPEED);
			return false;
		}
		// this.elevatorController.set(0);
		readyToFire = false;
		return true;

	}

	private boolean readyToFire = false;

	/**
	 * Prepares to fire a ball by revving up the flywheel motor and sets up a
	 * ball to be fired.
	 * 
	 * @return true if we're ready to fire, false otherwise.
	 */
	public boolean prepareToFire() {
		double dist = this.visionTargeter.getZDistanceToFuelTarget(this.visionTargeter.getLargestBlob());
		if (dist > 0) {
			this.flywheelController.setSetpoint(-2 * this.calculateRPMToMakeGoal(dist));
			// multiplied by 2 for gear ratio.
			this.loadBalls();
			if (Math.abs(this.flywheelController.getError() / 4.0) > this.acceptableError) {
				// this.stopLoader();
				return false;
			}
		} else
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
	public turnReturn turnToBearing(double newBearing) {
		if (Math.abs(newBearing - this.getBearing()) >= acceptableGimbalError) {
			return this.turnGimbal(MEDIUM_TURN_SPEED * (newBearing - getBearing() < 0 ? -1 : 1));
		}
		return turnReturn.SUCCESS;
	}

	/**
	 * Sets the gimbal motor to 0. Call after any turnGimbal function (except
	 * turnToBearing)
	 */
	public void stopGimbal() {
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
	public turnReturn turnGimbalSlow(int direction) {
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
	public turnReturn turnGimbalMedium(int direction) {
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
	public turnReturn turnGimbalFast(int direction) {
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
	private turnReturn turnGimbal(double speed) {
		if (this.getBearing() >= MAX_GIMBALING_ANGLE || this.getBearing() <= MIN_GIMBALING_ANGLE)
		{
			this.stopGimbal();
			return turnReturn.TOO_FAR;
		}
		// TODO direction
		/*
		 * Make sure we never turn faster than the maximum speed.
		 *
		 * ALEX! your TERNARY OPERATOR was CRAP! -McGee
		 * 
		 * ALSO motor is reversed so... that's why it's like that.
		 */
		if (speed < 0)
			this.gimbalMotor.set(Math.min(speed, MAX_TURN_SPEED));
		else if (speed > 0)
			this.gimbalMotor.set(Math.max(speed, -MAX_TURN_SPEED));
		else
			this.gimbalMotor.set(0.0);

		return turnReturn.WORKING;
	}
	
	/**
	 * @return The bearing of the shooter relative to the robot, with negative
	 *         degrees to the left and positive to the right. Returns a range
	 *         0 to MAX_
	 */
	public double getBearing()
	{
		return this.gimbalMotor.getEncPosition() * this.GIMBAL_ENCODER_FACTOR;
	}

	/**
	 * Returned from turnToBearing as well as the turn slow, medium, and fast
	 * functions. Provides additional information as to the state of the
	 * function.
	 * 
	 * @author Alexander H. Kneipp
	 */
	public static enum turnReturn {
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
	public turnToGoalReturn turnToGoal() {
		// if we have at least one blob
		this.visionTargeter.processImage();
		if (this.visionTargeter.getNthSizeBlob(0) != null) {
			// If we haven't yet calculated our setpoint yet.
			if (gimbalTarget == Double.MIN_VALUE) {
				// calculate our setpoint
				gimbalTarget = this.visionTargeter.getYawAngleToTarget(this.visionTargeter.getNthSizeBlob(0));
			} else {
				// turn to our new bearing
				if (turnToBearing(gimbalTarget + this.getBearing()) == turnReturn.SUCCESS) {
					// we're done!
					gimbalTarget = Double.MIN_VALUE;
					return turnToGoalReturn.SUCCESS;
				} else if (turnToBearing(gimbalTarget + this.getBearing()) == turnReturn.TOO_FAR) {
					gimbalTarget = Double.MIN_VALUE;
					return turnToGoalReturn.OUT_OF_GIMBALING_RANGE;
				}
			}
			// still working...l
			return turnToGoalReturn.WORKING;
		}
		// We don't see anything.
		return turnToGoalReturn.NO_BLOBS;
	}

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
	public static enum turnToGoalReturn {
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
	public double calculateRPMToMakeGoal(double distance) {
		double distanceMeters = distance * 3.28084;// Convert the distance
													// parameter
													// meters, for easier
													// computations.
		double perfectRPM = (60.0 / (2 * Math.PI) * (Math.sqrt(
				((4.9 * (Math.pow(distanceMeters, 2))) / ((this.FLYWHEEL_RADIUS_METERS * this.FLYWHEEL_RADIUS_METERS)
						* (Math.pow(Math.cos(Math.toRadians(this.MOUNT_ANGLE)), 2))
						* (distanceMeters * Math.tan(Math.toRadians(this.MOUNT_ANGLE))
								- this.RELATIVE_GOAL_HEIGHT_METERS))))));
		return perfectRPM + this.FLYWHEEL_SPEED_CORRECTION_CONSTANT * perfectRPM;
	}

	private final double MAX_TURN_SPEED = .5;

	private final double MEDIUM_TURN_SPEED = .35;

	private final double SLOW_TURN_SPEED = .2;

	private final double ELEVATOR_SPEED = 1;// TODO tune

	private final double MAX_GIMBALING_ANGLE = 18;// in degrees

	private final double MIN_GIMBALING_ANGLE = -18;// in degrees

	private final double MOUNT_ANGLE = 64;// TODO figure out the actual number.

	private final double RELATIVE_GOAL_HEIGHT_METERS = 1.93;

	private final double FLYWHEEL_RADIUS_METERS = 0.0508;

	private final double FLYWHEEL_SPEED_CORRECTION_CONSTANT = .3;// TODO tune

	/**
	 * factor the gimbal encoder must be set to (distance per pulse)
	 *  for degrees
	 */
	private final double GIMBAL_ENCODER_FACTOR = .00022448;
}

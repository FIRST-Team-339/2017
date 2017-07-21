package org.usfirst.frc.team339.Utils;

/**
 * Describes a device which is controlled by a PID loop and needs to be tuned.
 * 
 * @author Alexander Kneipp
 * @written 4/27/17
 *
 */
public interface PIDTunable
{
/**
 * Sets the P parameter for this PID device
 * 
 * @param P
 *            The p parameter
 * @return
 *         The p parameter if it was correctly set, Double.MIN_VALUE
 *         otherwise
 */
public double setP (double P);

/**
 * Sets the I parameter for this PID device
 * 
 * @param I
 *            The I parameter
 * @return
 *         The i parameter if it was correctly set, Double.MIN_VALUE
 *         otherwise.
 */
public double setI (double I);

/**
 * Sets the D parameter for this PID device
 * 
 * @param D
 *            The D parameter
 * @return
 *         The D parameter if it was correctly set, Double.MIN_VALUE
 *         otherwise.
 */
public double setD (double D);

/**
 * Sets the P, I, and D parameters for this PID device
 * 
 * @param P
 *            The P parameter for this PID device
 * @param I
 *            Sets the I parameter for this PID device
 * @param D
 *            Sets the D parameter for this PID device
 * @return
 *         The sum of all 3 parameters if all three were successfully,
 *         Double.MIN_VALUE if at least one failed to be set
 */
public double setPID (double P, double I, double D);

/**
 * @return
 *         The current P parameter for this PID device
 */
public double getP ();

/**
 * @return
 *         The current I parameter for this PID device
 */
public double getI ();

/**
 * @return
 *         The current D parameter for this PID device
 */
public double getD ();

/**
 * Sets the threshold of acceptable error on this device. The range of the
 * acceptable error is target +- threshold.
 * 
 * @param threshold
 *            The amount, above and below the setpoint, in which we are willing
 *            to accept the error.
 * @return
 *         The threshold if it was set correctly , -1 otherwise
 */
public double setErrorThreshold (double threshold);

/**
 * @return
 *         The amount, above and below the setpoint, within which we're willing
 *         to accept the error
 */
public double getErrorThreshold ();

/**
 * @return
 *         True if we're in the error zone set by setErrorThreshold, false
 *         otherwise. Essentially asks the PID device if we're willing to accept
 *         the error level we're at right now.
 */
public boolean getIsInAcceptableErrorZone ();

/**
 * Sets the target value this PID device will attempt to achieve. Can be in
 * terms of distance, velocity, temperature, any controllable value.
 * 
 * @param setpoint
 *            The value the PID device will target
 * @return
 *         The setpoint if it was successfully set, Double.MIN_VALUE otherwise
 */
public double setSetpoint (double setpoint);

/**
 * @return
 *         The value the PID device is currently targeting.
 */
public double getSetpoint ();
}

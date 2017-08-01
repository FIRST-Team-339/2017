package org.usfirst.frc.team339.Utils.pidTuning;
/**
 * Describes a class which tunes a PID device or which is a PID device which can be tuned
 * 
 * @author Alexander Kneipp
 * @written 4/27/17
 */
public interface PIDTunable
{
/**
 * Sets the P parameter for this PID device
 * 
 * @param P
 *            The p parameter
 */
public void setP (double P);

/**
 * Sets the I parameter for this PID device
 * 
 * @param I
 *            The I parameter
 */
public void setI (double I);

/**
 * Sets the D parameter for this PID device
 * 
 * @param D
 *            The D parameter
 */
public void setD (double D);

/**
 * Sets the P, I, and D parameters for this PID device
 * 
 * @param P
 *            The P parameter for this PID device
 * @param I
 *            Sets the I parameter for this PID device
 * @param D
 *            Sets the D parameter for this PID device
 */
public void setPID (double P, double I, double D);

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
 */
public void setErrorThreshold (double threshold);

/**
 * @return
 *      The current error the PID device is reporting
 *      (The difference between the target value and the feedback value)
 */
public double getError();
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
 */
public void setSetpoint (double setpoint);

/**
 * @return
 *         The value the PID device is currently targeting.
 */
public double getSetpoint ();
/**
 * @return
 *        The value the PID device is currently outputting.
 */
public double getOutput ();
/**
 * @return
 *        The value the PID device is currently receiving 
 *        from it's feedback device.
 */
public double getInput ();

}

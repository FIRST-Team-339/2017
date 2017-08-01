package org.usfirst.frc.team339.Utils.pidTuning;

/*
 * DEVELOPER NOTES:
 * 
 * Consider scrapping this or writing a wrapper for the
 * PIDController that isn't as specific. That class
 * seems to do most of what we need here.
 */

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * 
 * A class to control any motor controller which extends WPI's
 * SpeedController class via a PID loop.
 * Note: DO NOT use this with a CANTalon. Use the
 * CanTalonPIDTuner class instead. It uses the internal PID
 * controller in the motor controller, rather than a
 * software one here.
 * 
 * @author Alexander Kneipp
 * @written 7/20/17
 *
 */
public class MotorControllerPIDTuner implements PIDTunable
{
private PIDController pid = null;

private SpeedController motorController = null;

private PIDSource source = null;

public enum ControlType
    {
POSITION, VELOCITY
    }


/**
 * Constructes a new MotorControllerPIDTuner object.
 * See https://en.wikipedia.org/wiki/PID_controller for more information on PID
 * Controllers
 * See https://en.wikipedia.org/wiki/Feed_forward_(control) For more information
 * on the F parameter
 * 
 * @param p
 *            The proportional value to initialize the underlying PIDController
 *            with.
 * @param i
 *            The integral value to initialize the underlying PIDController
 *            with.
 * @param d
 *            The Derivitave value to initialize the underlying PIDController
 *            with
 * @param f
 *            The Feed-Forward value to initialize the underlying PIDController
 *            with
 * @param source
 *            The PIDSource class that provides feedback for the PIDController
 * @param controller
 *            The motor controller we want to tune the PID loop for
 * @param samplingPeriod
 *            How rapidly the PID controller will calculate the PID loop
 * @param outputIsContinuous
 *            If the beginning and end of the output are the same position (E.g.
 *            like 0 degrees and 360 degrees on a circle) and there is no hard
 *            or soft stop to prevent continuous motion, set to true, otherwise
 *            false.
 */
public MotorControllerPIDTuner (double p, double i,
        double d, double f, PIDSource source,
        SpeedController controller, double samplingPeriod,
        boolean outputIsContinuous)
{
    // Set up the PID controller for the motor
    this.pid = new PIDController(p, i, d, f, source, controller,
            samplingPeriod);
    // Save the motor controller, though I'm not sure that's actually necessary
    this.motorController = controller;
    this.source = source;
    // tell the PID controller if the in/output is continuous or not
    this.pid.setContinuous(outputIsContinuous);
}

@Override
public void setP (double P)
{
    this.setPID(P, this.getI(), this.getD());
}

@Override
public void setI (double I)
{
    this.setPID(this.getP(), I, this.getD());
}

@Override
public void setD (double D)
{
    this.setPID(this.getP(), this.getI(), D);
}

@Override
public void setPID (double P, double I, double D)
{
    this.pid.setPID(P, I, D);
}

@Override
public double getP ()
{
    return this.pid.getP();
}

@Override
public double getI ()
{
    return this.pid.getI();
}

@Override
public double getD ()
{
    return this.pid.getD();
}

@Override
public void setErrorThreshold (double threshold)
{
    this.pid.setAbsoluteTolerance(threshold);
}

/**
 * @deprecated
 *             The underlying PIDController class does not support the
 *             functionality of this method.
 * @return
 *         Always 0
 */
@Override
@Deprecated
public double getErrorThreshold ()
{
    return 0;
}

@Override
public boolean getIsInAcceptableErrorZone ()
{
    return this.pid.onTarget();
}

@Override
public void setSetpoint (double setpoint)
{
    this.pid.setSetpoint(setpoint);
}

@Override
public double getSetpoint ()
{
    return this.pid.getSetpoint();
}

@Override
public double getError ()
{
    return this.pid.getError();
}

@Override
public double getOutput ()
{
    return this.pid.get();
}

@Override
public double getInput ()
{
    return this.source.pidGet();
}

}

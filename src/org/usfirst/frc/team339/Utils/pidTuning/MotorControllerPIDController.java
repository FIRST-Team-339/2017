package org.usfirst.frc.team339.Utils.pidTuning;

/*
 * DEVELOPER NOTES:
 * 
 * Consider scrapping this or writing a wrapper for the 
 * PIDController that isn't as specific.  That class 
 * seems to do most of what we need here.
 */

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * 
 * A class to control any motor controller which extends WPI's 
 * SpeedController class via a PID loop.  
 * Note: DO NOT use this with a CANTalon.  Use the 
 * CanTalonPIDTuner class instead.  It uses the internal PID 
 * controller in the motor controller, rather than a 
 * software one here.
 * 
 * @author Alexander Kneipp
 * @written 7/20/17
 *
 */
public class MotorControllerPIDController implements PIDTunable
{
private PIDController pid= null;

private SpeedController motorController = null;

public enum ControlType
{
    POSITION,VELOCITY
}


/**
 * 
 * @param p
 * @param i
 * @param d
 * @param f
 * @param source
 * @param controller
 * @param samplingPeriod
 * @param outputIsContinuous
 */
public MotorControllerPIDController(double p, double i, 
        double d, double f, PIDSource source, 
        SpeedController controller, double samplingPeriod, 
        boolean outputIsContinuous)
{
    this.pid = new PIDController(p, i, d, f,source, controller,samplingPeriod);
    this.motorController = controller;
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
    this.setPID(this.getP(),I, this.getD());
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
 *      The underlying PIDController class does not support the functionality of this method.
 * @return
 *      Always 0
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

}

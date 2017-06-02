package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class to make tuning PID loops easier, either by the smartdashboard, or in
 * the code via some other sensor (you'll have to write the code in teleop or
 * something for another sensor)
 * 
 * @author Alex Kneipp
 * @written 1/21/17
 */
public class CanTalonPIDTuner implements PIDTuner
{
private CANTalon tunedMotorController = null;

private double F;

private double P = Double.MIN_VALUE;

private double I = Double.MIN_VALUE;

private double D = Double.MIN_VALUE;

private double setpoint = Double.MIN_VALUE;

private boolean smartDashboard;

private double errorThresh = 20;

/**
 * Constructs a PID tuner object for a specific CANTalon motor.
 * 
 * @param talon
 *            The motor we're tuning on. It needs to have some form of rate
 *            sensor attached to it's breakout board.
 * @param errorThreshold
 *            The maximum error we find it acceptable to have (always positive,
 *            we use absolute value of the actual error).
 */
public CanTalonPIDTuner (CANTalon talon, double errorThreshold)
{
    this.tunedMotorController = talon;
    this.P = 0;
    this.I = 0;
    this.D = 0;
    this.setpoint = 0;
    this.errorThresh = errorThreshold;
}

private boolean wasIncorrect = false;

private Timer time = new Timer();

private FeedbackDevice feedbackType;

// TODO reference CTRE doc
/**
 * Initializes all the CAN stuff for the motor controller.
 * 
 * @param feedbackType
 *            The type of feedback device in the MotorController.
 * @param tunetype
 *            Speed or position. See the CTRE doc for more info
 * @param codesPerRev
 *            The number of signals the feedback devices gives per rotation, so
 *            the setpoint can function in rpm or revolutions.
 * @param reverseSensor
 *            Is the feedback device reversed.
 */
public void setupMotorController (FeedbackDevice feedbackType,
        TalonControlMode tunetype, int codesPerRev,
        boolean reverseSensor)
{
    this.feedbackType = feedbackType;
    this.tunedMotorController
            .setFeedbackDevice(feedbackType);
    this.tunedMotorController.changeControlMode(tunetype);
    this.tunedMotorController.configEncoderCodesPerRev(codesPerRev);
    this.tunedMotorController.reverseSensor(reverseSensor);
    this.tunedMotorController.setProfile(0);
    this.tunedMotorController.configPeakOutputVoltage(12f, -12f);
    this.tunedMotorController.configNominalOutputVoltage(0f, 0f);
    wasIncorrect = false;
    time.stop();
    time.reset();
}


public void update ()
{
    this.tunedMotorController.setPID(this.P, this.I, this.D);
    this.tunedMotorController.setF(this.F);
    this.tunedMotorController.setSetpoint(this.setpoint);
}

/**
 * 
 * @param F
 *            The Feed-forward value for the FPID loop;
 */
public void setF (double F)
{
    this.F = F;
    this.update();
}

/**
 * 
 * @param P
 *            The proportional constant for the PID loop.
 */
@Override
public double setP (double P)
{
    this.P = P;
    this.update();
    return this.getP();// TODO incorrect implementation
}

/**
 * 
 * @param I
 *            The integral constant for the PID loop.
 */
@Override
public double setI (double I)
{
    this.I = I;
    this.update();
    return this.getI();
}

/**
 * 
 * @param D
 *            The derivative constant for the PID loop.
 */
@Override
public double setD (double D)
{
    this.D = D;
    this.update();
    return this.getD();
}

/**
 * 
 * @return
 *         the feed-forward value of the FPID loop.
 */
public double getF ()
{
    return this.F;
}

/**
 * 
 * @param P
 *            The proportional constant for the PID loop.
 * @param I
 *            The integral constant for the PID loop.
 * @param D
 *            The derivative constant for the PID loop.
 * @deprecated by Alex Kneipp. Use setFPID(double, double, double, double)
 *             instead.
 */
@Deprecated
@Override
public double setPID (double p, double i, double d)
{
    this.P = p;
    this.I = i;
    this.D = d;
    this.update();
    // TODO does not return correctly on failed set
    return this.getP() + this.getI() + this.getD();
}

/**
 * 
 * @param f
 *            The feed-forward constant for the FPID loop.
 * @param p
 *            The proportional constant for the FPID loop.
 * @param i
 *            The integral constant for the FPID loop.
 * @param d
 *            The derivative constant for the FPID loop.
 */
public void setFPID (double f, double p, double i, double d)
{
    this.setPID(p, i, d);
    this.F = f;
    this.update();
}

/**
 * 
 * @return
 *         The current proportional constant for the PID loop
 */
@Override
public double getP ()
{
    return this.P;
}

/**
 * 
 * @return
 *         The current integral constant for the PID loop
 */
@Override
public double getI ()
{
    return this.I;
}

/**
 * 
 * @return
 *         The current derivative constant for the PID loop
 */
@Override
public double getD ()
{
    return this.D;
}

/**
 * 
 * @param errThresh
 *            Sets the error threshold that we find acceptable. Always positive.
 */
@Override
public double setErrorThreshold (double threshold)
{
    this.errorThresh = threshold;
    // TODO incorrect implementation of error return, but should never fail to
    // write?
    return this.getErrorThreshold();
}

/**
 * 
 * @return
 *         The current error threshold.
 */
@Override
public double getErrorThreshold ()
{
    return this.errorThresh;
}

/**
 * @param setpoint
 *            The target velocity or position of the motor being tuned.
 *            in revolutions or rpm depending on PID type.
 */
@Override
public double setSetpoint (double setpoint)
{
    this.setpoint = setpoint;
    this.update();
    return this.getSetpoint();
}

/**
 * 
 * @return
 *         The current setpoint.
 */
@Override
public double getSetpoint ()
{
    return this.setpoint;
}

@Override
public boolean getIsInAcceptableErrorZone ()
{
    return Math.abs(this.tunedMotorController.getError()) < this
            .getErrorThreshold();
}
}

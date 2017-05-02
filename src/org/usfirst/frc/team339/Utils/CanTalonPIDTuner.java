package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to make tuning PID loops easier, either by the smartdashboard, or in
 * the code via some other sensor (you'll have to write the code in teleop or
 * something for another sensor)
 * 
 * @author Alex Kneipp
 * @written 1/21/17
 */
public class CanTalonPIDTuner implements PIDTunable
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

/**
 * Initializes everything on the smartDashboard if we have one. If not, does
 * nothing.
 * Puts P, I, D, Setpoint, Error, and Speed.
 * P, I, D, and Setpoint are all editable and affect the function of the code.
 * Setpoint is in units of RPM or revolutions if you provided a number other
 * than 1 for the codesPerRev argument to setupMotorController.
 */
public void setupDashboard ()
{
    this.setpoint = 0;
    if (this.smartDashboard)
        {
        SmartDashboard.putNumber("P", this.P);
        SmartDashboard.putNumber("I", this.I);
        SmartDashboard.putNumber("D", this.D);
        SmartDashboard.putNumber("Setpoint", this.setpoint);
        SmartDashboard.putNumber("Error",
                this.setpoint - this.tunedMotorController.getSpeed());
        SmartDashboard.putNumber("Speed",
                this.tunedMotorController.getSpeed());
        System.out.println(
                "PID: " + this.P + ", " + this.I + ", " + this.D);
        System.out.println("Setpoint, error: " + this.setpoint + ", "
                + this.tunedMotorController.getClosedLoopError());
        System.out.println(
                "Speed: " + this.tunedMotorController.getSpeed());
        }
}

/**
 * Gets information from the smartDashboard, puts more info back onto the
 * smartDashboard, and updates the values on the tuned motor controller.
 * Prints out when an error beyond the provided threshold is detected, and the
 * time for the PID loop to correct for it.
 */
public void update ()
{
    P = SmartDashboard.getNumber("P", this.P);
    I = SmartDashboard.getNumber("I", this.I);
    D = SmartDashboard.getNumber("D", this.D);
    this.setpoint = SmartDashboard.getNumber("Setpoint", this.setpoint);
    SmartDashboard.putNumber("Error",
            this.setpoint - this.tunedMotorController.getSpeed());
    SmartDashboard.putNumber("Speed",
            this.tunedMotorController.getSpeed());
    this.tunedMotorController.set(this.setpoint);
    this.tunedMotorController.setPID(this.P, this.I, this.D);
    /*
     * if the absolute value of the error (/4 if it's some form of quadrature)
     * is greater than our threshold, tell the RIOlog and time how long it takes
     * to return.
     */
    double tempError = 0;
    if (this.feedbackType == FeedbackDevice.CtreMagEncoder_Relative
            || this.feedbackType == FeedbackDevice.CtreMagEncoder_Absolute
            || this.feedbackType == FeedbackDevice.QuadEncoder)
        {
        tempError = Math.abs(
                this.tunedMotorController.getClosedLoopError() / 4.0);
        }
    else
        {
        tempError = Math
                .abs(this.tunedMotorController.getClosedLoopError());
        }
    if (tempError >= this.errorThresh
            && this.wasIncorrect == false)
        {
        this.wasIncorrect = true;
        this.time.reset();
        this.time.start();
        System.out.println("Error detected, timing...");
        }
    if (Math.abs(
            this.tunedMotorController
                    .getClosedLoopError()) <= this.errorThresh
            && this.wasIncorrect == true)
        {
        this.wasIncorrect = false;
        this.time.stop();
        System.out.println("Time to correct error: " + this.time.get());
        }
}

/**
 * 
 * @param F
 *            The Feed-forward value for the FPID loop;
 */
public void setF (double F)
{
    this.F = F;
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

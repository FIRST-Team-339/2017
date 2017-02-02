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
public class CANPIDTuner
{
private int CANID = 0;// TODO either use or remove

private CANTalon tunedMotorController = null;

private double P;

private double I;

private double D;

private double setpoint;

private boolean smartDashboard;

private double errorThresh = 20;

/**
 * Constructs a PID tuner object for a specific CANTalon motor.
 * 
 * @param canId
 *            Not currently used. There for future potential use.
 * @param talon
 *            The motor we're tuning on. It needs to have some form of rate
 *            sensor attached to it's breakout board.
 * @param smartDashboardAvailable
 *            If the smartdashboard is setup and used. We won't bother updating
 *            it if we have it.
 * @param errorThreshold
 *            The maximum error we find it acceptable to have (always positive,
 *            we use absolute value of the actual error).
 */
public CANPIDTuner (int canId, CANTalon talon,
        boolean smartDashboardAvailable, double errorThreshold)// TODO remove
                                                               // the
                                                               // smartDashboard
                                                               // thingy
{
    this.tunedMotorController = talon;
    this.CANID = canId;
    this.P = 0;
    this.I = 0;
    this.D = 0;
    this.setpoint = 0;
    this.smartDashboard = smartDashboardAvailable;
    this.errorThresh = errorThreshold;
}

private boolean wasIncorrect = false;

private Timer time = new Timer();

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
    this.P = 0;
    this.I = 0;
    this.D = 0;
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
    if (this.smartDashboard)
        {
        P = SmartDashboard.getNumber("P", 0);
        I = SmartDashboard.getNumber("I", 0);
        D = SmartDashboard.getNumber("D", 0);
        this.setpoint = SmartDashboard.getNumber("Setpoint", 0);
        SmartDashboard.putNumber("Error",
                this.setpoint - this.tunedMotorController.getSpeed());
        SmartDashboard.putNumber("Speed",
                this.tunedMotorController.getSpeed());
        }
    this.tunedMotorController.set(this.setpoint);
    this.tunedMotorController.setPID(this.P, this.I, this.D);
    if (Math.abs(
            this.tunedMotorController
                    .getClosedLoopError()) >= this.errorThresh
            && wasIncorrect == false)
        {
        wasIncorrect = true;
        time.reset();
        time.start();
        System.out.println("Error detected, timing...");
        }
    if (Math.abs(
            this.tunedMotorController
                    .getClosedLoopError()) <= this.errorThresh
            && wasIncorrect == true)
        {
        wasIncorrect = false;
        time.stop();
        System.out.println("Time to correct error: " + time.get());
        }
}

/**
 * 
 * @param P
 *            The proportional constant for the PID loop.
 */
public void setP (double P)
{
    this.P = P;
}

/**
 * 
 * @param I
 *            The integral constant for the PID loop.
 */
public void setI (double I)
{
    this.I = I;
}

/**
 * 
 * @param D
 *            The derivative constant for the PID loop.
 */
public void setD (double D)
{
    this.D = D;
}

/**
 * 
 * @param P
 *            The proportional constant for the PID loop.
 * @param I
 *            The integral constant for the PID loop.
 * @param D
 *            The derivative constant for the PID loop.
 */
public void setPID (double p, double i, double d)
{
    this.P = p;
    this.I = i;
    this.D = d;
}

/**
 * 
 * @return
 *         The current proportional constant for the PID loop
 */
public double getP ()
{
    return this.P;
}

/**
 * 
 * @return
 *         The current integral constant for the PID loop
 */
public double getI ()
{
    return this.I;
}

/**
 * 
 * @return
 *         The current derivative constant for the PID loop
 */
public double getD ()
{
    return this.D;
}

/**
 * 
 * @param errThresh
 *            Sets the error threshold that we find acceptable. Always positive.
 */
public void setErrorThreshold (double errThresh)
{
    this.errorThresh = errThresh;
}

/**
 * 
 * @return
 *         The current error threshold.
 */
public double getErrorThreshold ()
{
    return this.errorThresh;
}

/**
 * 
 * @param setpoint
 *            The target velocity or position of the motor being tuned.
 *            in revolutions or rpm depending on PID type.
 */
public void setSetpoint (double setpoint)
{
    this.setpoint = setpoint;
}

/**
 * 
 * @return
 *         The current setpoint.
 */
public double getSetpoint ()
{
    return this.setpoint;
}

}

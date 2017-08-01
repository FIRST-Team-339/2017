package org.usfirst.frc.team339.Utils.pidTuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPIDTunerDevice
{
public PIDTunable tunable = null;

private byte debugOutputMask = 0x0;

private Timer errTime = new Timer();

/**
 * Defines the type of debug information the tuner wants to see as output
 * 
 * The variables the program can output are:
 * <ul>
 * <li>The current value of the output</li>
 * <li>When the controller leaves the acceptable error value and how long it
 * takes to return</li>
 * <li>The current error in the output</li>
 * <li>The current values for the F, P, I, and D variables in the
 * controller</li>
 * <li>Whether or not the controller is currently on target</li>
 * <li>The current target for the controller</li>
 * <li>The current feedback value</li>
 * </ul>
 * 
 * @author Alexander Kneipp
 */
public enum DebugType
    {
/**
 * Print out ALL debug information
 */
ALL ((byte)0b01111111),
/**
 * Prints out a single message when the controller leaves the acceptable target
 * range, and another when we reenter the target range with the time difference
 * between leaving and reentering the acceptable range
 */
RETURN_TIME ((byte)0b00100000),
/**
 * Prints out the current target velocity, the current error, whether the
 * controller is currently on target, the current feedback value,
 * and the current output
 */
CONTROLLER_STATUS((byte)0b01010111),
/**
 * Only prints out the error reported by the controller
 */
CONTROLLER_ERROR((byte)0b00010000),
/**
 * Prints out the F, P, I, and D values for the controller
 */
CONTROL_VALUES ((byte)0b00001000),
/**
 * Prints out whether or not the PID controller is currently on target
 */
CONTROLLER_IS_ON_TARGET((byte)0b00000100),
/**
 * prints out nothing
 */
NONE ((byte)0b00000000);

private byte equivalentDebugCodeVal;

DebugType(byte debugVarEquivalent)
{
    this.equivalentDebugCodeVal = debugVarEquivalent;
}

byte getDebugCodeEquivalent()
{
    return this.equivalentDebugCodeVal;
}
    }

public SmartDashboardPIDTunerDevice(PIDTunable tuner, byte debug)
{
    this.debugOutputMask = debug;
    this.tunable = tuner;
}

public SmartDashboardPIDTunerDevice(PIDTunable tuner, DebugType debug)
{
    this(tuner,debug.getDebugCodeEquivalent());
}

public SmartDashboardPIDTunerDevice (PIDTunable tuner)
{
    this(tuner, DebugType.NONE);
}


public void update ()
{
    double P = SmartDashboard.getNumber("DB/Slider 0", 0.0);
    double I = SmartDashboard.getNumber("DB/Slider 1", 0.0);
    double D = SmartDashboard.getNumber("DB/Slider 2", 0.0);
    double setPoint = SmartDashboard.getNumber("DB/Slider 3", 0.0);
    this.tunable.setP(P);
    this.tunable.setI(I);
    this.tunable.setD(D);
    this.tunable.setSetpoint(setPoint);
}

public void setDebugOutputType (DebugType type)
{
    this.debugOutputMask = type.getDebugCodeEquivalent();
}
/**
 * <p>
 * The debug information printed to the console is controlled via a one-byte bit mask.
 * (Each bit in the byte represents whether or not to print out a particular available
 * debug value) Following is a list of the available debug information, in order from 
 * the MSB (Most significant bit) to LSB (Least significant bit)
 * </p>
 * <ol>
 * <li>DO NOT SET THIS BIT.  IT DOES NOTHING</li>
 * <li>The current value of the output</li>
 * <li>When the controller leaves the acceptable error value and how long it
 * takes to return</li>
 * <li>The current error in the output</li>
 * <li>The current values for the F, P, I, and D variables in the
 * controller</li>
 * <li>Whether or not the controller is currently on target</li>
 * <li>The current target for the controller</li>
 * <li>The current feedback value</li>
 * </ol>
 * @param debugMask
 *      A one byte bit mask determining what debug information to print out 
 *      (See list above).  Java has a nifty feature where you can directly 
 *      define binary literals by prepending the binary number with <em>0b</em>.
 *      e.g. To print out the output value, the error in the controller, 
 *      and the current feedback value, you should pass the argument 
 *      <em>0b01010001</em>
 *      
 * @author Alexander Kneipp
 */
public void setDebugOutputBitmask(byte debugMask)
{
    this.debugOutputMask = debugMask;
}

public void printOutDebugInformation ()
{
    if((this.debugOutputMask & 0b01000000) != 0)
        {
            System.out.println("PID output: " + this.tunable.getOutput());
        }
    if((this.debugOutputMask & 0b00100000) != 0)
        {
            if(this.tunable.getIsInAcceptableErrorZone() ==false && wasOffTarget == false)
                {
                    System.out.println("Tunable is off target, recording time...");
                    this.errTime.reset();
                    this.errTime.start();
                }
            else if(this.tunable.getIsInAcceptableErrorZone() == true && wasOffTarget == true)
                {
                    this.errTime.stop();
                    System.out.println("Tunable has returned to target, took " 
                            + this.errTime.get() + " Seconds");
                }
        wasOffTarget = !this.tunable.getIsInAcceptableErrorZone();
        }
    if((this.debugOutputMask & 0b00010000) != 0)
        {
            System.out.println("PID error: " + this.tunable.getError());
        }
    if((this.debugOutputMask & 0b00001000) != 0)
        {
        System.out.println("PID Tunable values: (" + this.tunable.getP() 
            + ", " + this.tunable.getI() + ", " + this.tunable.getD() + ")");
        }
    if((this.debugOutputMask & 0b00000100)!= 0)
        {
        System.out.println("PID tunable is on target?" + 
                (this.tunable.getIsInAcceptableErrorZone()?"YES!":"NO :("));
        
        }
    if((this.debugOutputMask & 0b00000010) != 0)
        {
            System.out.println("PID tunable target: " + this.tunable.getSetpoint());
        }
    if((this.debugOutputMask & 0b00000001) != 0)
        {
            System.out.println("PID tunable feedback value: " + this.tunable.getInput());
        }
}
private boolean wasOffTarget = false;

}

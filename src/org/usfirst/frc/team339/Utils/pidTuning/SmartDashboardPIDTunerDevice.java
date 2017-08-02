package org.usfirst.frc.team339.Utils.pidTuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//TODO correct the author's note.
/**
 * Author's note:
 * <p>
 * Only ONE instance of this class will work at a time.  Do NOT
 * try to tune multiple PID devices with multiple instances of 
 * this class.  Tune your devices one at a time
 * </p>
 * 
 * <p>
 * A class for tuning PID devices which implement the PIDTunable
 * interface.
 * </p>
 * @author Alexander Kneipp
 */
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
/**
 * Creates a new SmartDashboardPIDTunerDevice with debug information
 * @param tunable
 *      The PIDTunable object you want to tune with the smart dashboard
 * @param debug
 *      The debug information bitmask you want while you're tuning, see
 *      setDebugOutputBitmask(byte) for more information.
 */
public SmartDashboardPIDTunerDevice(PIDTunable tunable, byte debug)
{
    this.debugOutputMask = debug;
    this.tunable = tunable;
}
/**
 * Creates a new SmartDashboardPIDTunerDevice with debug information
 * @param tunable
 *      The PIDTunable object you want to tune with the smart dashboard
 * @param debug
 *      The debug information DebugType you want while you're tuning, see
 *      setDebugOutputType(DebugType) for more information.
 */
public SmartDashboardPIDTunerDevice(PIDTunable tunable, DebugType debug)
{
    this(tunable,debug.getDebugCodeEquivalent());
}
/**
 * Creates a new SmartDashboardPIDTuner object without debug information
 * @param tunable
 *      The PIDTunable object you want to tune with the smart dashboard
 */
public SmartDashboardPIDTunerDevice (PIDTunable tunable)
{
    this(tunable, DebugType.NONE);
}

/**
 * <h1>
 * WARNING!!!
 * </h1>
 * <p>
 * It is STRONGLY recommended by the author of this class that you
 * call either setDebugOutputType(DebugType) or 
 * setDebugOutputBitmask(byte) OR you make sure that you constructed 
 * your SmartDashboardPIDTuner object with one of the constructors 
 * that accepts debug information (see the constructor doc) BEFORE 
 * you call this method.  You only need to call one of them once 
 * with the desired bitmask or DebugType for them to have the desired
 * effect.  Somewhere in an init will do for the setDebugOutput... methods.
 * perfectly.
 * </p>
 * <p>
 *  This should be the primary or only method call in your PID tuning
 *  loop; it should handle everything you need it to handle, so long
 *  as you call the debug setup methods before your tuning loop.
 * </p> 
 * <p>
 * Multi purpose method.
 * First pulls the setpoint, P, I, and D information from
 * the smart dashboard sliders.  It then sets that information
 * to the tunable device, and finally prints out the debug
 * information set with setDebugOutputType(DebugType) or
 * setDebugOutputBitmask(byte) (See those methods for more
 * information.)  The debug phase prints out nothing if
 * neither of these methods have been called.
 * </p>
 */
public void update ()
{
    //Pull the P, I, D, and setpoint information from the smart dashboard
    double P = SmartDashboard.getNumber("DB/Slider 0", 0.0);
    double I = SmartDashboard.getNumber("DB/Slider 1", 0.0);
    double D = SmartDashboard.getNumber("DB/Slider 2", 0.0);
    double setPoint = SmartDashboard.getNumber("DB/Slider 3", 0.0);
    //Set the P, I, D, and setpoint info to the PID device
    this.tunable.setP(P);
    this.tunable.setI(I);
    this.tunable.setD(D);
    this.tunable.setSetpoint(setPoint);
    //Print out the previously set debug information
    this.printOutDebugInformation();
}
/**
 *<p>
 * Sets the debug information to output to the console via 
 * predetermined Enum values
 * </p>
 * @param type
 *      <p>
 *      The DebugType Enum deciding the debug information to
 *      output.  See the documentation in DebugType for more info
 *      </p>
 */
public void setDebugOutputType (DebugType type)
{
    /*
     * Get the preset debug mask equivalent from the
     * Enum, and save that for debugging output
     */
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
/**
 * Prints debug information according to the set debugStatus.
 * The default debug status is NONE; nothing is printed out.
 * If you actually want debug information, you need to set it
 * with either of the setDebugOutput... methods.
 * See the enum DebugType and/or the method setDebugOutputBitmask
 * for more documentation 
 */
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

package org.usfirst.frc.team339.Utils.pidTuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPIDTunerDevice
{
public PIDTunable tuner = null;

private byte debugOutput = 0x0;

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
    this.debugOutput = debug;
    this.tuner = tuner;
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
    this.tuner.setP(P);
    this.tuner.setI(I);
    this.tuner.setD(D);
    this.tuner.setSetpoint(setPoint);
}

public void setDebugOutputType (DebugType type)
{
    this.debugOutput = type.getDebugCodeEquivalent();
}

public void printOutDebugInformation ()
{

}

}

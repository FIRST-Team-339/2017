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
ALL (),
/**
 * Prints out a single message when the controller leaves the acceptable target
 * range, and another when we reenter the target range with the time difference
 * between leaving and reentering the acceptable range
 */
RETURN_TIME,
/**
 * Prints out the current target velocity, the current error, whether the
 * controller is currently on target, the current feedback value,
 * and the current output
 */
CONTROLLER_STATUS,
/**
 * Only prints out the error reported by the controller
 */
CONTROLLER_ERROR,
/**
 * Prints out the F, P, I, and D values for the controller
 */
CONTROL_VALUES,
/**
 * Prints out whether or not the PID controller is currently on target
 */
CONTROLLER_IS_ON_TARGET,
/**
 * prints out nothing
 */
NONE
    }

public SmartDashboardPIDTunerDevice (PIDTunable tuner)
{
    this.tuner = tuner;
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

}

public void printOutDebugInformation ()
{

}

}

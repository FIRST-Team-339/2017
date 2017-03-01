package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.Servo;

public class KilroyServo extends Servo
{

/**
 * Kilroy Servo
 * 
 * @param portNumber
 *            the port the servo is on
 * @param maxDegrees
 *            the maximum degrees the servo can go
 * @author Becky Button
 */
public KilroyServo (int portNumber, double maxDegrees)
{
    // sets the port number to the one set in KilroyServo
    super(portNumber);
    // sets up shorthand where this.maxDegrees is the same as maxDegrees
    this.maxDegrees = maxDegrees;
}

// sets maxDegrees equal to 0
private double maxDegrees = 0; // max degrees servo can go

/**
 * Scales raw set value to degrees
 * 
 * @param degree
 *            you want to set the servo
 */
// sets angle of the servo
public void setAngle (double degree)
{
    // divides the setAngle value by the maxDegrees to scale it from 0 to 1
    double setVal = degree / this.maxDegrees; // scaled value for servo
    // sets the angle using the set() from KilroyServo
    super.set(setVal);
}

/**
 * scales raw value to degrees
 * 
 * @return Angle value in degrees
 */
//
public double getAngle ()
{
    // gets angle value thats already scaled from 0-1, and then scales it to
    // degrees
    // by multiplying it by the maxDegrees
    double angleValue = super.get() * this.maxDegrees; // scaled value for
                                                       // servo
    // returns angle value
    return angleValue;

}
}

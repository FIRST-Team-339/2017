package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.Servo;

public class KilroyServo // extends Servo
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
    // super(portNumber);
    this.servo = new Servo(portNumber);
    this.maxDegrees = maxDegrees;
}

private double maxDegrees = 0;

/**
 * Scales raw set value to degrees
 * 
 * @param degree
 *            you want to set the servo
 */
public void setAngle (double degree)
{
    double setVal = degree / this.maxDegrees; // scaled value for servo
    this.servo.set(setVal);
}

/**
 * scales raw value to degrees
 * 
 * @return Angle value in degrees
 */
public double getAngle ()
{
    double angleValue = this.servo.get() * this.maxDegrees;
    return angleValue;

}

private Servo servo;


}

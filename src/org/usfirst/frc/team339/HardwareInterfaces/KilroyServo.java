package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.Servo;

public class KilroyServo // extends Servo
{

/**
 * @param portNumber
 * @param maxDegrees
 */
public KilroyServo (int portNumber, double maxDegrees)
{
    // super(portNumber);
    this.servo = new Servo(portNumber);
    this.maxDegrees = maxDegrees;
}

private double maxDegrees = 0;

public void setAngle (double degree)
{
    double setVal = degree / this.maxDegrees; // scaled value for servo
    this.servo.set(setVal);
}

public double getAngle ()
{
    double angleValue = this.servo.get() * this.maxDegrees;
    return angleValue;

}

private Servo servo;


}

package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class KilroyGyro
{
private final ADXRS450_Gyro gyro;

private boolean hasGyro = true;

public KilroyGyro (boolean hasGyro)
{
    // set this.hasGyro equal to hasGyro-essentially setting up shorthand
    this.hasGyro = hasGyro;
    // if we do not have a gyro
    if (hasGyro == false)
        {
        // print out Gyro not connected
        System.out.println("Gyro not connected");
        }
    //// if we have a gyro
    if (hasGyro == true)
        {
        // then declare as a new gyro
        this.gyro = new ADXRS450_Gyro();
        }
    // if we don't have a gyro, but we don't not have a gyro, return null
    else
        // if neither then return null
        this.gyro = null;
}

// calibrates the gyro
public void calibrate ()
{
    // if we do not have a gyro
    if (this.hasGyro == false)
        {
        // then return void
        return;
        }
    // then calibrate gyro
    this.gyro.calibrate();
}

// resets the gyro
public void reset ()
{
    // if we do not have a gyro
    if (this.hasGyro == false)
        {
        // return void
        return;
        }
    // then reset the gyro
    this.gyro.reset();
}

// will return the gyro angle in degrees
public double getAngle ()
{
    // if we don't have a gyro
    if (this.hasGyro == false)
        {
        // return 339339
        return 339339;
        }
    // return the angle of the gyro in degrees
    return this.gyro.getAngle();

}

// returns the rate of rotation for the gyro
public double getRate ()
{
    // if we don't have a gyro
    if (this.hasGyro == false)
        {
        // return the random value 339339
        return 339339;
        }
    // return the rate of rotation of the gyro
    return this.gyro.getRate();
}
}

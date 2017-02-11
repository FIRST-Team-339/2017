package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class KilroyGyro
{
private final ADXRS450_Gyro gyro;

private boolean hasGyro = true;

public KilroyGyro (boolean hasGyro)
{
    this.hasGyro = hasGyro;
    if (hasGyro == false)
        {
        System.out.println("Gyro not connected");

        }
    if (hasGyro)
        {
        this.gyro = new ADXRS450_Gyro();

        }
    else
        this.gyro = null;
}

public void calibrate ()
{
    if (this.hasGyro == false)
        {
        return;
        }
    this.calibrate();
}

public void reset ()
{
    if (this.hasGyro == false)
        {
        return;
        }
    this.reset();
}

public double getAngle ()
{
    if (this.hasGyro == false)
        {
        return 339339;
        }
    return this.getAngle();

}

public double getRate ()
{
    if (this.hasGyro == false)
        {
        return 339339;
        }
    return this.getRate();
}
}

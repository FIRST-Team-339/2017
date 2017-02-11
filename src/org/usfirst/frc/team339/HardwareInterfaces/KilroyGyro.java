package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class KilroyGyro
{
private final ADXRS450_Gyro gyro;

private boolean hasGyro = true;

public KilroyGyro (boolean hasGyro)
{
    if (hasGyro == false)
        {
        System.out.println("Gyro not connected");
        this.hasGyro = false;
        }
    if (hasGyro)
        {
        this.gyro = new ADXRS450_Gyro();
        this.hasGyro = true;
        }
    else
        this.gyro = null;
}

public void calibrate ()
{
    if (hasGyro == false)
        {
        return;
        }
    this.calibrate();
}

public void reset ()
{
    if (hasGyro == false)
        {
        return;
        }
    this.reset();
}

public double getAngle ()
{
    if (hasGyro == false)
        {
        return 339339;
        }
    return this.getAngle();

}

public double getRate ()
{
    if (hasGyro == false)
        {
        return 339339;
        }
    return this.getRate();
}
}

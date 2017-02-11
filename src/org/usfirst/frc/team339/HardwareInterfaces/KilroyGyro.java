package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;


public class KilroyGyro
{
private final ADXRS450_Gyro gyro;
private SPI m_spi;

public KilroyGyro (boolean hasGyro)
{
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

public void calibrateToZero ()
{
    if (m_spi == null)
        {
        return;
        }
    m_spi.resetAccumulator();
    m_spi.setAccumulatorCenter(0);
}

public void reset ()
{
    if (m_spi == null)
        {
        return;
        }
    m_spi.resetAccumulator();
}

public double getAngle ()
{
    if (m_spi == null)
        {
        return 888888;
        }
    return m_spi.getAccumulatorValue();

}
}

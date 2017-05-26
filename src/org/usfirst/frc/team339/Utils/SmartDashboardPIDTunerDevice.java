package org.usfirst.frc.team339.Utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPIDTunerDevice
{
private PIDTuner tuner = null;

public SmartDashboardPIDTunerDevice (PIDTuner tuner)
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

}

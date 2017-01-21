package org.usfirst.frc.team339.Utils;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANPIDTuner
{
private int CANID = 0;

private CANTalon tunedMotorController = null;

private double P;

private double I;

private double D;

private double setpoint;

private boolean smartDashboard;

public CANPIDTuner (int canId, CANTalon talon,
        boolean smartDashboardAvailable)
{
    this.tunedMotorController = talon;
    this.CANID = canId;
    this.P = 0;
    this.I = 0;
    this.D = 0;
    this.setpoint = 0;
    this.smartDashboard = smartDashboardAvailable;
}

public void setupMotorController ()
{

}

public void setupDashboard ()
{
    SmartDashboard.putNumber("P", this.P);
    SmartDashboard.putNumber("I", this.I);
    SmartDashboard.putNumber("D", this.D);
    SmartDashboard.putNumber("Setpoint", this.setpoint);
    SmartDashboard.putNumber("Error",
            this.setpoint - this.tunedMotorController.getSpeed());
    SmartDashboard.putNumber("Speed", this.tunedMotorController.getSpeed())
}


}

package org.usfirst.frc.team339.Utils;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Victor;

/**
 * Software representation of the ball intake for the 2017 FIRST SteamWorks fuel
 * intake.
 * 
 * @author Alexander H. Kneipp
 *
 */
public class BallIntake
{
private Victor intakeMotor = null;

private Relay agitatorMotor = null;

/**
 * Creates a new ball intake motor.
 * 
 * @param motor
 *            the Spark motor controller that runs the intake.
 * @param agitator
 *            TODO
 */
public BallIntake (Victor motor, Relay agitator)
{
    this.intakeMotor = motor;
    this.agitatorMotor = agitator;
}

/**
 * Run the intake motors to suck in all the balls. Also starts agitator.
 */
public void startIntake ()
{
    this.intakeMotor.set(this.INTAKE_SPEED);
    this.agitatorMotor.set(Value.kForward);
}

/**
 * Stop the intake motors to stop sucking in all the balls. Also stops agitator.
 */
public void stopIntake ()
{
    this.intakeMotor.set(0.0);
    this.agitatorMotor.set(Value.kOff);
}

private final double INTAKE_SPEED = .4;
}

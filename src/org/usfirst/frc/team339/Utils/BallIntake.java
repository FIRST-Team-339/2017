package org.usfirst.frc.team339.Utils;

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

/**
 * Creates a new ball intake motor.
 * 
 * @param motor
 *            the Spark motor controller that runs the intake.
 */
public BallIntake (Victor motor)
{
    this.intakeMotor = motor;
}

/**
 * Run the intake motors to suck in all the balls.
 */
public void startIntake ()
{
    this.intakeMotor.set(this.INTAKE_SPEED);
}

/**
 * Stop the intake motors to stop sucking in all the balls.
 */
public void stopIntake ()
{
    this.intakeMotor.set(0.0);
}

private final double INTAKE_SPEED = .4;
}

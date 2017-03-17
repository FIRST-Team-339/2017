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

private Victor agitatorMotor = null;

/**
 * Creates a new ball intake motor.
 * 
 * @param motor
 *            the Spark motor controller that runs the intake.
 * @param agitatorMotor2
 *            TODO
 */
public BallIntake (Victor motor, Victor agitatorMotor2)
{
    this.intakeMotor = motor;
    this.agitatorMotor = agitatorMotor2;
}

/**
 * Run the intake motors to suck in all the balls. Also starts agitator.
 */
public void startIntake ()
{
    this.intakeMotor.set(this.INTAKE_SPEED);
    // this.agitatorMotor.set(AGITATOR_SPEED);TODO turn back on once fixed
}

/**
 * Pushes out balls in our hopper. Only use as an override.
 */
public void reverseIntake ()
{
    this.intakeMotor.set(-this.INTAKE_SPEED);
    // this.agitatorMotor.set(AGITATOR_SPEED);
}

/**
 * Stop the intake motors to stop sucking in all the balls. Also stops agitator.
 */
public void stopIntake ()
{
    this.intakeMotor.set(0.0);
    // this.agitatorMotor.set(0.0);
}

private final double INTAKE_SPEED = .4;

private final double AGITATOR_SPEED = 1.0;
}

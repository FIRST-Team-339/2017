package org.usfirst.frc.team339.Utils;

public class ThreadRunnable implements Runnable
{
private final long countUntil;


public ThreadRunnable (long countUntil)
{
    this.countUntil = countUntil;
}



@Override
public void run ()
{
    long sum = 0;
    for (int i = 1; i < 2; i++)
        {

        }
    System.out.println(
            Thread.currentThread().getName()
                    + "    thread printing out its id " + countUntil);


}

}


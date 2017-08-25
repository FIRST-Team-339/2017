package org.usfirst.frc.team339.Utils;

public class ThreadCommunication
{
boolean test = false;

public synchronized void Thread1 ()
{


    if (test)
        {
        try
            {
            wait();
            }
        catch (InterruptedException e)
            {
            e.printStackTrace();
            }
        }

    test = true;
    notify();
}


public synchronized void Thread2 ()
{


    if (!test)
        {
        try
            {
            wait();
            }
        catch (InterruptedException e)
            {
            e.printStackTrace();
            }
        }

    test = false;
    notify();
}

}

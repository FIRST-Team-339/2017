package org.usfirst.frc.team339.Utils;

import com.ni.vision.NIVision.Image;

public class Threading extends Thread
{

public long threadID;

public String threadName;

public boolean runThread = true;

public int valueForTeleopInteger = 0;

public double valueForTeleopDouble = 0;

public String valueForTeleopString = "";

public volatile Image imageForTeleop;

public ThreadingTestClass threadTester = new ThreadingTestClass();


public Threading (String name)
{
    super(name);
    threadID = this.getId();

    this.setName(name);

    threadName = name;


}


public void run ()
{
    System.out.println(threadName + " ID number: " + threadID);
    System.out.println(Threading.currentThread().getName());

    while (runThread == true)
        {


        }
}




}

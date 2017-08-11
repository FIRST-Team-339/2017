package org.usfirst.frc.team339.Utils;

import com.ni.vision.NIVision.Image;
import org.usfirst.frc.team339.robot.Teleop;

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

        System.out.println("thread1?" + Teleop.thread1.isAlive());

        Teleop.thread1.valueForTeleopInteger = 1;
        Teleop.thread1.valueForTeleopDouble = 3.14;
        Teleop.thread1.valueForTeleopString = "Hello Wold";

        System.out.println(
                "Thread 1 int return : "
                        + Teleop.thread1.valueForTeleopInteger);
        System.out.println(
                "Thread 1 double return : "
                        + Teleop.thread1.valueForTeleopDouble);
        System.out.println(
                "Thread 1 string return : "
                        + Teleop.thread1.valueForTeleopString);
        System.out.println("thread 1 id" + Teleop.thread1.getId());

        System.out.println("This is running on thread id"
                + Thread.currentThread().getId());
        }
}




}

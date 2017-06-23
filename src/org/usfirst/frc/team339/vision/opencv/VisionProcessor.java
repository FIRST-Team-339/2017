package org.usfirst.frc.team339.vision.opencv;

import java.util.Comparator;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

/**
 * This class contains vision code that uses OpenCV and the auto-generated
 * code from the program GRIP. To properly set up GRIP to work with this
 * class, make sure to set the package name, the class name to
 * AutoGenVision.java
 * and the directory to the correct package.
 * 
 * NOTE: The GRIP project MUST end with a "filter contours" modifier
 * 
 * @author Ryan McGee
 * @written 6/22/17
 *
 */
public class VisionProcessor extends AutoGenVision
{

/**
 * A class that holds several statistics about particles.
 * 
 * The measures include:
 * area: The area, in pixels of the blob
 * boundingRect: The rectangle around the blob
 * center: the point of the center of the blob
 * 
 * @author Ryan McGee
 *
 */
public class ParticleReport implements Comparator<ParticleReport>,
        Comparable<ParticleReport>
{
public double area;

public Rect boundingRect;

public Point center;


@Override
public int compare (ParticleReport r1, ParticleReport r2)
{
    return (int) (r1.area - r2.area);
}

@Override
public int compareTo (ParticleReport r)
{
    return (int) (r.area - this.area);
}
}

public enum ImageSource
    {
    IPCAM, USBCAM
    }

private final ImageSource sourceType;

private int usbPort = 0;

private String ip = "";

private volatile VideoCapture source = null;

private volatile Mat image = new Mat();

private volatile ParticleReport[] particleReports = null;

private volatile boolean pauseThread = true;

private volatile boolean endThread = false;

/**
 * Creates the object and sets the IP
 * 
 * @param ip
 *            the IP of the .mjpg the axis camera outputs
 */
public VisionProcessor (String ip)
{
    this.sourceType = ImageSource.IPCAM;
    this.ip = ip;
    init();
}

/**
 * Creates the object and sets the usb port number
 * 
 * @param port
 *            the port number that the USB camera is on. The default is 0 and
 *            increments for each camera added.
 */
public VisionProcessor (int port)
{
    this.sourceType = ImageSource.USBCAM;
    this.usbPort = port;
    init();
}

/**
 * Upon construction, the separate thread will start.
 */
private void init ()
{
    // The new thread made will run the method runThread over and over.
    (new Thread(new Runnable()
    {
    public void run ()
    {
        // wait for the camera to initialize...
        while (initCamera() == false)
            ;

        // While loop will run independently from the rest of the code.
        while (endThread == false)
            {// If the user has chosen to stop processing, then just skip the
             // code.
            if (pauseThread == false)
                {
                runThread();
                }
            }
    }
    })).start();
}

/**
 * Initialize the capture source and return whether or not it has been opened.
 * 
 * @return whether or not the camera has been set up yet
 */
private boolean initCamera ()
{
    // IF the source has not been initialized, do so and open the port.
    if (source == null)
        {
        // When the source is initialized with a parameter, it will
        // automatically try to open it.
        if (sourceType == ImageSource.IPCAM)
            {
            source = new VideoCapture(ip);
            }
        else if (sourceType == ImageSource.USBCAM)
            {
            source = new VideoCapture(usbPort);
            }
        }

    return source.isOpened();
}

// ==========================END INIT===================================


/**
 * The method that is run over and over in the separate thread
 */
private void runThread ()
{
    source.read(image);
    super.process(image);
    createParticleReports(super.filterContoursOutput());

}

/**
 * Takes the base OpenCV list of contours and changes the output to be easier to
 * work with.
 * 
 * @param contours
 *            The input from the base OpenCV contours output
 */
private void
        createParticleReports (List<MatOfPoint> contours)
{
    ParticleReport[] reports = new ParticleReport[contours.size()];

    for (int i = 0; i < reports.length; i++)
        {
        reports[i] = new ParticleReport();
        Rect r = Imgproc.boundingRect(contours.get(i));
        reports[i].area = r.area();
        reports[i].center = new Point(r.x + (r.width / 2),
                r.y + (r.height / 2));
        reports[i].boundingRect = r;
        }

    this.particleReports = reports;
}

/**
 * Sorts the particle analysis reports by their area in descending order
 */
private void sortParticleReportsBySize ()
{

}

// ==================END THREAD MANAGMENT===============================





}

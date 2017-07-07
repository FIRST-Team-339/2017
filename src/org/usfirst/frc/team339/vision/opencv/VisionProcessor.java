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
/**
 * The area of the bounding rectangle around the blob
 */
public double area = 0;

/**
 * The rectangle around the blob
 */
public Rect boundingRect = new Rect(new Point(0, 0), new Point(0, 0));

/**
 * the center of the bounding rectangle around the blob
 */
public Point center = new Point(0, 0);


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

/**
 * The type of source for the input pictures.
 * 
 * @author Ryan McGee
 *
 */
public enum ImageSource
    {
    /**
     * The IP for the axis camera; must include the final .mpeg
     * extension
     */
    IPCAM,
    /**
     * The port for the USB camera. 0 is the default, and 1 is the alternate.
     */
    USBCAM
    }

private final ImageSource sourceType;

private int usbPort = 0;

private String ip = "";

private volatile VideoCapture source = null;

private volatile Mat image = new Mat();

private volatile ParticleReport[] particleReports = new ParticleReport[0];


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
    initCamera();
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
    initCamera();
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
 * The method that processes the image and inputs it into the particle reports
 */
public void processImage ()
{
    source.read(image);
    super.process(image);
    createParticleReports(super.filterContoursOutput());
    sortParticleReportsBySize();
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
    if (particleReports.length == 0)
        return;

    ParticleReport[] newReports = new ParticleReport[particleReports.length];
    // goes through the newly created reports array and populates it.
    for (int i = 0; i < newReports.length; i++)
        {
        int largestIndex = 0;

        // Finds the next largest particle and assigns it to the next
        // position in the new array, and sets the old one to null.
        for (int k = 1; k < newReports.length; k++)
            {
            if (particleReports[k] != null
                    && particleReports[k].area > particleReports[largestIndex].area)
                {
                largestIndex = k;
                }
            }
        newReports[i] = particleReports[largestIndex];
        particleReports[largestIndex] = null;
        }
    particleReports = newReports;
}

// S=====================USER ACCESSABLE METHODS========================
/**
 * 
 * @return the list of blobs generated after processing the image
 */
public ParticleReport[] getParticleReports ()
{
    return particleReports;
}

/**
 * @param n
 *            The index of the size requested. 0 is the largest, and
 *            gradually gets smaller until the end of the array is reached.
 * @return The blob thats the Nth largest in the particleReports array.
 */
public ParticleReport getNthSizeBlob (int n)
{
    return particleReports[n];
}





}

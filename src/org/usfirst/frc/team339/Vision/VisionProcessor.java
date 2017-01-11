package org.usfirst.frc.team339.Vision;

import java.awt.Point;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Comparator;

/**
 * This is the class that does the processing of the image, and can be called as
 * many times as necessary.
 * <p>
 * 		PROCESSOR COMMANDS: 
 * <br>			-1: process image and send it over the socket 
 * <br>			-2: set the threshold values from the socket 
 * <br>			-3: reserved just in case for the main socket for requesting new sockets 
 * <br>			-4: sets the operations and their parameters (such as Dilation and Erosion)
 * <br>			-5: requests the thread to continuously process images
 * <br>			-6: requests to save the raw image to the destination provided
 * <br>			-7: requests to save the processed image to the destination provided
 * <br>			-8: load a configuration file saved from the thresholdutility program
 * </p>
 * @author Ryan McGee
 *
 */
public class VisionProcessor extends Thread
{
	int port;

	public VisionProcessor(int port)
	{
		this.port = port;
		this.start();
	}

	private ArrayList<int[]> operations = new ArrayList<int[]>();

	boolean requestSingleProcessedImage = false;

	/**
	 * <br>[0] = blueLowerBound 
	 * <br>[1] = greenLowerBound 
	 * <br>[2] = redLowerBound 
	 * <br>[3] = blueUpperBound 
	 * <br>[4] = greenUpperBound 
	 * <br>[5] = redUpperBound 
	 * <br>[6] = brightness
	 */
	static int[] thresholdValues = new int[9];

	/**
	 * ArrayList of integer arrays, containing blob information.
	 *<br> (They are all rectangles)
	 * <p>
	 *<br> 			blobs[0] = x coordinate
	 *<br> 			blobs[1] = y coordinate
	 *<br> 			blobs[2] = width
	 *<br> 			blobs[3] = height
	 *</p>
	 **/
	public ArrayList<int[]> blobs = null;

	/**
	 * Will display true only if the program just received new blobs.
	 * You can change this if you are finished using this set of blobs
	 * and want to wait for the next to send.
	 */
	public volatile boolean blobsAreNew = false;

	@SuppressWarnings(
	{ "unchecked" })
	public void run()
	{
		try
		{
			System.out.println("VisionProcessor output port: " + port);
			ServerSocket listener = new ServerSocket(port);
			System.out.println("Vision Processor Listeners Created");
			Socket socket = listener.accept();
			System.out.println("Vision Processor Sockets Created");
			ObjectOutputStream oos = new ObjectOutputStream(new BufferedOutputStream(socket.getOutputStream()));
			oos.flush();
			ObjectInputStream ois = new ObjectInputStream(new BufferedInputStream(socket.getInputStream()));

			InputStreamReader iis = new InputStreamReader(ois);

			System.out.println("Vision Processor I/O streams created");
			Thread.sleep(1000);
			boolean alreadyRequested = false;
			Object o = null;

			while (true)
			{
				o = iis.getObject();
				if (blobsAreNew && o != null)
				{
					blobs = (ArrayList<int[]>) o;
				}
				if (sendOperations)
				{
					oos.writeInt(-4);
					oos.writeObject(operations);
					sendOperations = false;
				}

				if (saveRawImage)
				{
					System.out.println("Requesting to save unprocessed image");
					oos.writeInt(-6);
					oos.writeObject(destination);
					saveRawImage = false;
				}

				if (saveProcessedImage)
				{
					System.out.println("Requesting to save processed image");
					oos.writeInt(-7);
					oos.writeObject(destination);
					saveProcessedImage = false;
				}
				// Requests an array of rectangles' x and y coordinates
				if (requestSingleProcessedImage)
				{
					if (alreadyRequested == false)
					{
						oos.writeInt(-1);
						// System.out.println("requested processed image");
						alreadyRequested = true;
					}
					requestSingleProcessedImage = false;
					alreadyRequested = false;

				}

				if (stopThread)
				{
					try
					{ // Increase this time delay in case of connection abort
						// error ONLY.
						Thread.sleep(500);
					} catch (InterruptedException e)
					{
						e.printStackTrace();
					}
					break;
				}

				if (loadConfig)
				{
					oos.writeInt(-8);
					oos.writeObject(configDestination);
					System.out.println("Loading configuration from file");
					loadConfig = false;
				}
				oos.flush();

			}

			ois.close();
			oos.close();
			socket.close();
			listener.close();
		} catch (IOException | InterruptedException e)
		{
			e.printStackTrace();

		}

	}

	/**
	 *  Load a configuration file saved from the Threshold Utility.
	 * @param destination @param destination the path to the file; 
	 * 			it MUST contain the FULL path, right down to the .cfg at the end.
	 */
	public void loadConfig(String destination)
	{
		this.configDestination = destination;
		loadConfig = true;
	}

	private String configDestination = "";
	private boolean loadConfig = false;

	/**
	 * Sends a request to process a SINGLE image ONLY if it is not continuously processing images
	 * 
	 */
	public void requestSingleProcessedImage()
	{
		// if (!isRunningContinuously)
		requestSingleProcessedImage = true;
	}

	/**
	 * Gets the widest blob from the ArrayList of blobs
	 * @param blobs
	 * @return int array: The widest blob with 
	 * 			[0] = top left x coordinate, 
	 * 			[1] = y, 
	 * 			[2] = width, 
	 * 			[3] = height
	 */
	public int[] getWidestBlob(ArrayList<int[]> blobs)
	{
		int[] temp =
		{ 0, 0, 0, 0 };
		if (blobs != null)
			for (int i = 0; i < blobs.size(); i++)
			{
				if (blobs.get(i)[2] > temp[2])
				{
					temp = blobs.get(i);
				}
			}
		return temp;
	}

	/**
	 * Sets the threshold boundaries in BGR format, with an added Scalar for
	 * brightness
	 * 
	 * @param blueLowerBound
	 * @param greenLowerBound
	 * @param redLowerBound
	 * @param blueUpperBound
	 * @param greenUpperBound
	 * @param redUpperBound
	 * @param brightness
	 * 
	 * @author Ryan McGee
	 */
	public void threshold(int blueLowerBound, int greenLowerBound, int redLowerBound, int blueUpperBound,
			int greenUpperBound, int redUpperBound, int brightness, int colorCode)
	{
		thresholdValues[0] = 3;
		thresholdValues[1] = blueLowerBound;
		thresholdValues[2] = greenLowerBound;
		thresholdValues[3] = redLowerBound;
		thresholdValues[4] = blueUpperBound;
		thresholdValues[5] = greenUpperBound;
		thresholdValues[6] = redUpperBound;
		thresholdValues[7] = brightness;
		thresholdValues[8] = colorCode;

		operations.add(thresholdValues);
	}

	public static final int BGR = 0;
	public static final int HSV = 1;

	/**
	 * Sets the threshold values in BGR format, ranging from 0-255 and brightness from -255 to 255.
	 * This will automatically set the lower and upper boundaries based on the range and input values.
	 * 
	 * @param blue Average blue value
	 * @param green Average green value
	 * @param red Average red value
	 * @param brightness brightness value, added to BGR matrix as a scalar
	 * @param percent percentage value expressed as a number between 0 and 1
	 */
	public void threshold(int blue, int green, int red, int brightness, double percent)
	{
		thresholdValues[0] = 3;
		thresholdValues[1] = (int) testForNegative(blue - (255 * percent));
		thresholdValues[2] = (int) testForNegative(green - (255 * percent));
		thresholdValues[3] = (int) testForNegative(red - (255 * percent));
		thresholdValues[4] = (int) (blue + (255 * percent));
		thresholdValues[5] = (int) (green + (255 * percent));
		thresholdValues[6] = (int) (red + (255 * percent));
		thresholdValues[7] = brightness;
		operations.add(thresholdValues);
	}

	private double testForNegative(double i)
	{
		if (i < 0)
		{
			return 0.0;
		} else
		{
			return i;
		}
	}

	boolean sendOperations = false;

	/**
	 * Tells the Raspberry pi what must be done in what order, and with which parameters.
	 * This runs after operations have been set (dilation, erosion, threshold),
	 * and should NOT be run continuously.
	 */
	public void sendOperations()
	{
		sendOperations = true;
	}

	/**
	 * Dilate blobs on the image (make all blobs larger from their middle).
	 * Useful for resizing the blobs back to normal after erosion, or closing a blob.
	 * @param size How large the dilation should make the blobs
	 * @param iterations How many times the dilation should run
	 */
	public void dilate(int size, int iterations)
	{
		int[] dilation =
		{ 1, size, iterations };
		operations.add(dilation);
	}

	/**
	 * Erode the image (make all blobs smaller from their middle).
	 * Useful for removing small objects.
	 * @param size How small the erosion should make the blobs
	 * @param iterations How many times it should be eroded
	 */
	public void erode(int size, int iterations)
	{
		int[] erosion =
		{ 2, size, iterations };
		operations.add(erosion);
	}

	/**
	 * Removes objects with an area less than the size given, without
	 * altering the size of other blobs (does not use erosion).
	 * @param size size of the blobs that should be removed
	 */
	public void removeSmallObjects(int size)
	{
		int[] rmSmObs =
		{ 4, size };
		operations.add(rmSmObs);
	}

	boolean saveRawImage = false;

	/**
	 * Requests that the Raspberry Pi saves the camera picture(WITH brightness change)
	 * before processing it. This is useful for finding threshold values.
	 * @param destination The path you want the file to save to. DO NOT add the file name or extension.
	 */
	public void saveRawImage(String destination)
	{
		this.destination = destination;
		saveRawImage = true;
	}

	boolean saveProcessedImage = false;

	/**
	 * Requests that the Raspberry Pi saves the camera picture after
	 *  processing it. This is useful for finding the correct values for aiming.
	 * @param destination The path you want the file to save to. DO NOT add the file name or extension.
	 */
	public void saveProcessedImage(String destination)
	{
		this.destination = destination;
		saveProcessedImage = true;
	}

	private boolean stopThread = false;

	/**
	 * Halts requests for images, closes listeners, sockets and I/O streams, and ends the thread.
	 * Any requests made for single images will be discarded unless creating a new object.
	 * <p>
	 * NOTE: to stop the Vision Processor threads (in the case of enabling / disabling) the RaspberryPi object MUST call the stopAllThreads 
	 * function before the VisionProcessor object does. (It is a case of timing between sockets). If you are still getting a Connection abort:
	 * socket write error on EITHER the rio or the pi, increase the time delay in VisionProcessor.run().
	 */
	public void stopThread()
	{
		this.stopThread = true;
	}

	/**
	 * Gets an array of the information of all the blobs
	 */
	public void getParticleReport()
	{
		ParticleReport[] particles = new ParticleReport[blobs.size()];
		for (int i = 0; i < this.blobs.size(); i++)
		{
			particles[i] = new ParticleReport(blobs.get(i));
		}
		this.particleReports = particles;
	}

	/**
	 * Filters out all blobs that do not have the aspect ratio falling in the range of the lowerBound and upperBound specified, or the given
	 * aspect ratio give or take the tolerance times that aspect ratio.
	 * <br> NOTE: the aspect ratio is (width / height), so a value above one is wider than tall, etc.
	 * @param arg0 If AspectRatio is set to BOUNDARY, this is the lower boundary of acceptable aspect ratios.
	 *				Else if it is set to TOLERANCE, this is the desired aspect ratio.
	 * @param arg1 If AspectRatio is set to BOUNDARY, this is the upper boundary of acceptable aspect ratios.
	 * 				Else if it is set to TOLERANCE, this is the tolerance.
	 * @param useTolerance 
	 */
	public void filterAspectRatio(double arg0, double arg1, AspectRatio a)
	{
		ArrayList<ParticleReport> filteredReports = new ArrayList<ParticleReport>();
		switch (a)
		{
		case BOUNDARY:
			for (int i = 0; i < this.particleReports.length; i++)
			{
				if (this.particleReports[i].blobAspectRatio > arg0 && this.particleReports[i].blobAspectRatio < arg1)
					filteredReports.add(this.particleReports[i]);
			}
			break;
		case TOLERANCE:
			for (int i = 0; i < this.particleReports.length; i++)
			{
				if (this.particleReports[i].blobAspectRatio > (arg0 - (arg0 * arg1))
						&& this.particleReports[i].blobAspectRatio < (arg0 + (arg0 * arg1)))
					filteredReports.add(this.particleReports[i]);
			}
		}

		this.particleReports = (ParticleReport[]) filteredReports.toArray();
	}

	public static enum AspectRatio
	{
		BOUNDARY, TOLERANCE
	}

	public ParticleReport[] particleReports;

	String destination;

	/**
	 * A simple object that can continuously read an object from an objectInputStream
	 * WITHOUT blocking the code.
	 * 
	 * @author Ryan McGee
	 *
	 */
	private class InputStreamReader extends Thread
	{
		private ObjectInputStream ois;

		public InputStreamReader(ObjectInputStream ois)
		{
			this.ois = ois;
			this.start();

		}

		public void run()
		{
			while (true)
			{
				try
				{
					o = ois.readObject();
					blobsAreNew = true;
				} catch (ClassNotFoundException | IOException e)
				{
					break;
				}
				try
				{
					Thread.sleep(1);
				} catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}

		private volatile Object o = null;

		/**
		 * 
		 * @return the last received object from the ObjectInputStream. Will return null if no object has been found yet.
		 */
		public Object getObject()
		{
			return o;
		}

	}

	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>
	{

		// TODO add missing camera code and Particle Report

		public Point topLeft;

		public Point bottemRight;

		public Point centerOfRect;

		public int rectWidth;
		public int rectHeight;

		public int rectArea;

		public int imageHeight;
		public int imageWidth;

		public double blobAspectRatio;

		public ParticleReport(int[] info)
		{
			this.topLeft = new Point(info[0], info[1]);

			this.rectWidth = info[2];
			this.rectHeight = info[3];

			this.rectArea = this.rectWidth * this.rectHeight;

			this.bottemRight = new Point(this.topLeft.x + this.rectWidth, this.topLeft.y + this.rectHeight);

			this.centerOfRect = new Point(this.topLeft.x + (int) Math.round(this.rectWidth / 2.0),
					this.topLeft.y + (int) Math.round(this.rectHeight / 2.0));

			this.blobAspectRatio = this.rectWidth / (double) this.rectHeight;

		}

		@Override
		public int compareTo(ParticleReport arg0)
		{
			if (this.rectArea > arg0.rectArea)
				return 1;
			else if (this.rectArea < arg0.rectArea)
				return -1;
			else
				return 0;
		}

		@Override
		public int compare(ParticleReport arg0, ParticleReport arg1)
		{
			if (arg0.rectArea > arg1.rectArea)
				return 1;
			else if (arg0.rectArea < arg1.rectArea)
				return -1;
			else
				return 0;
		}

	}

}

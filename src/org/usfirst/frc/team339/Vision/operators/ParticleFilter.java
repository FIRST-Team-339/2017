package org.usfirst.frc.team339.Vision.operators;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.MeasurementType;
import com.ni.vision.NIVision.ParticleFilterCriteria2;
import com.ni.vision.NIVision.ParticleFilterOptions2;

/**
 * Create multiple filters that remove any particles that do not fall in
 * the defined criteria. The user can choose which blob measurement value
 * (for example center of mass, area, etc.) and the range for the values,
 * and whether or not a positive falls inside or outside the range.
 * 
 * @author Ryan McGee
 * @written 5/18/17
 *
 */
public class ParticleFilter implements VisionOperatorInterface
{
/**
 * Contains all of the filters added to this object
 */
private ParticleFilterCriteria2[] allCriteria = new ParticleFilterCriteria2[0];

/**
 * Adds a filter to the current list. You can chain criteria by
 * using the form: new
 * ParticleFilter().addCriteria(a,b,x,y,z).addCriteria(a,b,x,y,z)
 * to set multiple filters on the image.
 * 
 * @param type
 *            the information that is referenced when filtering out blobs.
 * @param lower
 *            The lower range of this measurement type
 * @param upper
 *            The upper range of this measurement type
 * @param calibrated
 *            Set to 1 for calibrated measurements, 0 for pixel measurements
 * @param exclude
 *            Set to 1 to indicate a positive if outside the range, 0 for inside
 *            the range.
 * @return the object you are referencing: so that you can chain criteria
 *         when adding to the vision script.
 */
public ParticleFilter addCriteria (MeasurementType type, int lower,
        int upper, int calibrated, int exclude)
{
    // Create a new array that can contain 1 more item than the current,
    // and copy all of the current object references into the new array.
    // Then, add the new reference to the new array.
    ParticleFilterCriteria2[] newCriteriaArray = new ParticleFilterCriteria2[allCriteria.length
            + 1];
    System.arraycopy(allCriteria, 0, newCriteriaArray, 0,
            allCriteria.length);

    newCriteriaArray[newCriteriaArray.length
            - 1] = new ParticleFilterCriteria2(type, lower, upper,
                    calibrated, exclude);

    // Set the current array to the new array.
    allCriteria = newCriteriaArray;

    return this;
}


public Image operate (Image Source)
{
    // If we have criteria, then use the filter. If not, just return the
    // original image.
    if (allCriteria.length > 0)
        {
        Image out = NIVision
                .imaqCreateImage(ImageType.IMAGE_U8, 0);
        NIVision.imaqParticleFilter4(out, Source, allCriteria,
                new ParticleFilterOptions2(0, 0, 0, 0), null);
        Source.free();
        return out;
        }

    return Source;
}

}

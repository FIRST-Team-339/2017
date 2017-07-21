/**
 *
 */
package org.usfirst.frc.team339.vision.operators;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

/** @author Kilroy */
public class RemoveLargeObjectsOperator
        implements VisionOperatorInterface
{

private final int erosions;

private final boolean connectivity8;

public RemoveLargeObjectsOperator (int iterations,
        boolean connectivity8)
{
    this.erosions = iterations;
    this.connectivity8 = connectivity8;
}

@Override
public Image operate (Image source)
{
    Image alteredImage = null;
    if (this.connectivity8 == true)
        {
        NIVision.imaqSizeFilter(alteredImage, source, 0, this.erosions,
                NIVision.SizeType.KEEP_SMALL,
                new NIVision.StructuringElement(
                        3, 3, 0));
        }
    else
        {
        NIVision.imaqSizeFilter(alteredImage, source, 1, this.erosions,
                NIVision.SizeType.KEEP_SMALL,
                new NIVision.StructuringElement(
                        3, 3, 0));
        }
    source.free();
    return alteredImage;
}

}

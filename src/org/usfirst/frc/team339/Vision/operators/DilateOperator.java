package org.usfirst.frc.team339.Vision.operators;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

public class DilateOperator implements VisionOperatorInterface
{
private int iterations;

public DilateOperator (int iterations)
{
    this.iterations = iterations;
}

@Override
public Image operate (Image Source)
{
    Image returnImage = null;
    Image input = Source;
    for (int i = 0; i < this.iterations; i++)
        {
        Image out = NIVision
                .imaqCreateImage(ImageType.IMAGE_U8, 0);
        NIVision.imaqGrayMorphology(out, input,
                NIVision.MorphologyMethod.DILATE,
                new NIVision.StructuringElement());
        Source.free();
        returnImage = out;
        input = out;
        }
    return (returnImage);
}

}

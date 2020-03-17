package us.ihmc.euclid.referenceFrame.api;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * Implement this interface to create builders for frame types such as {@code FramePoint2D}.
 * <p>
 * The frame objects created using this builder should contain random values changing from one
 * object to the next.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RandomFrameTypeBuilder
{
   /**
    * Creates a new instance of the frame type.
    * <p>
    * The frame objects created using this builder should contain random values changing from one
    * object to the next.
    * </p>
    *
    * @param referenceFrame the reference frame in which the returned frame object should be expressed
    *                       in.
    * @return the next random frame object.
    */
   ReferenceFrameHolder newInstance(Random random, ReferenceFrame referenceFrame);
}
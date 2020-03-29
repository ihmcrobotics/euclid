package us.ihmc.euclid.referenceFrame.api;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * Implement this interface to create builders for frame types such as {@code FrameQuaternion}.
 * <p>
 * The frame objects created using this builder should be initialized using the given reference
 * frame and frameless object.
 * </p>
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public interface FrameTypeCopier
{
   /**
    * Creates a new instance of the frame type.
    * <p>
    * The frame objects created using this builder should be initialized using the given reference
    * frame and frameless object.
    * </p>
    *
    * @param referenceFrame  the reference frame in which the returned frame object should be expressed
    *                        in.
    * @param framelessObject the frameless object to use for initializing the values of the new frame
    *                        object.
    * @return the new frame object.
    */
   ReferenceFrameHolder newInstance(ReferenceFrame referenceFrame, Object framelessObject);
}
package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FrameVector2DBasics extends FixedFrameVector2DBasics, FrameTuple2DBasics, FrameChangeable
{
   /**
    * Performs a transformation of the vector such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this vector 2D can result in a vector 3D. This method projects the result of the
    * transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the vector is to be expressed.
    */
   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}

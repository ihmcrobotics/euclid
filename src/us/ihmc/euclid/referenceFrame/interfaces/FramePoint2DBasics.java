package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FramePoint2DBasics extends FixedFramePoint2DBasics, FrameTuple2DBasics, FrameChangeable
{
   /**
    * Performs a transformation of the point such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this point 2D can result in a point 3D. This method projects the result of the
    * transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the point is to be expressed.
    */
   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}

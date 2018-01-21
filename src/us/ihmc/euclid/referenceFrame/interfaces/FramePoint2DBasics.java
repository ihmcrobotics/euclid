package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

/**
 * Write and read interface for a 2D point expressed in a changeable reference frame, i.e. the
 * reference frame in which this point is expressed can be changed.
 * <p>
 * In addition to representing a {@link Point2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePoint2DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePoint2DBasics} extends {@code Point2DBasics}, it is compatible with methods
 * only requiring {@code Point2DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePoint2DBasics}.
 * </p>
 */
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

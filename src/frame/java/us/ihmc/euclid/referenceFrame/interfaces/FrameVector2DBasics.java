package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * Write and read interface for a 2D vector expressed in a changeable reference frame, i.e. the
 * reference frame in which this vector is expressed can be changed.
 * <p>
 * In addition to representing a {@link Vector2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector2DBasics}. This allows, for instance, to enforce, at runtime, that operations
 * on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector2DBasics} extends {@code Vector2DBasics}, it is compatible with
 * methods only requiring {@code Vector2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector2DBasics}.
 * </p>
 */
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

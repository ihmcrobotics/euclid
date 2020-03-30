package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

/**
 * Write and read interface for a 2D point expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * In addition to representing a {@link Point2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFramePoint2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFramePoint2DBasics} extends {@code Point2DBasics}, it is compatible with
 * methods only requiring {@code Point2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFramePoint2DBasics}.
 * </p>
 */
public interface FixedFramePoint2DBasics extends FramePoint2DReadOnly, FixedFrameTuple2DBasics, Point2DBasics
{
   /**
    * Sets this point coordinate to the given {@code referenceFrame}'s origin coordinate in this frame
    * tuple current frame.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }
}

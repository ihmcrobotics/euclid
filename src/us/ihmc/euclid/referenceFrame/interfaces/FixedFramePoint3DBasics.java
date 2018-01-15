package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Write and read interface for a 3D point expressed in a constant reference frame, i.e. this point
 * is always expressed in the same reference frame.
 * <p>
 * In addition to representing a {@link Point3DBasics}, a {@link ReferenceFrame} is associated to
 * a {@code FixedFramePoint3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFramePoint3DBasics} extends {@code Point3DBasics}, it is compatible with
 * methods only requiring {@code Point3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFramePoint3DBasics}.
 * </p>
 */
public interface FixedFramePoint3DBasics extends FramePoint3DReadOnly, FixedFrameTuple3DBasics, Point3DBasics
{
   /**
    * Sets this point coordinate to the given {@code referenceFrame}'s origin coordinate in this
    * frame tuple current frame.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }
}

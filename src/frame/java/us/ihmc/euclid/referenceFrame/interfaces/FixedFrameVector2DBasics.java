package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * Write and read interface for a 2D vector expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * In addition to representing a {@link Vector2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFrameVector2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFrameVector2DBasics} extends {@code Vector2DBasics}, it is compatible with
 * methods only requiring {@code Vector2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameVector2DBasics}.
 * </p>
 */
public interface FixedFrameVector2DBasics extends FrameVector2DReadOnly, FixedFrameTuple2DBasics, Vector2DBasics
{
   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNormalize(FrameVector2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Vector2DBasics.super.setAndNormalize(other);
   }
   
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return FrameVector2DReadOnly.super.geometricallyEquals(geometry, epsilon);
   }
}

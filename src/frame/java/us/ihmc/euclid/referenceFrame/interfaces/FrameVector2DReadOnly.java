package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Read-only interface for a 2D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector2DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameVector2DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector2DReadOnly} extends {@code Vector2DReadOnly}, it is compatible with
 * methods only requiring {@code Vector2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector2DReadOnly}.
 * </p>
 */
public interface FrameVector2DReadOnly extends Vector2DReadOnly, FrameTuple2DReadOnly
{
   /**
    * Calculates and returns the angle in radians from this frame vector to {@code other}.
    * <p>
    * The computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param other the other frame vector used to compute the angle. Not modified.
    * @return the value of the angle from this frame vector to {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double angle(FrameVector2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Vector2DReadOnly.super.angle(other);
   }

   /**
    * Calculates and returns the value of the cross product of this frame vector with {@code tuple}.
    *
    * @param tuple the second term in the cross product. Not modified.
    * @return the value of the cross product.
    */
   default double cross(FrameTuple2DReadOnly tuple)
   {
      checkReferenceFrameMatch(tuple);
      return Vector2DReadOnly.super.cross(tuple);
   }
}

package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Read-only interface for a 4D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple4DReadOnly}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple4DReadOnly}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple4DReadOnly} extends {@code Tuple4DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple4DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple4DReadOnly}.
 * </p>
 */
public interface FrameTuple4DReadOnly extends Tuple4DReadOnly, ReferenceFrameHolder
{
   /**
    * Tests on a per component basis if this tuple is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two tuples have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other tuple to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameTuple4DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple4DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this tuple is exactly equal to {@code other}.
    * <p>
    * If the two tuples have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other tuple to compare against this. Not modified.
    * @return {@code true} if the two tuples are exactly equal component-wise and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameTuple4DReadOnly other)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple4DReadOnly.super.equals(other);
   }
}

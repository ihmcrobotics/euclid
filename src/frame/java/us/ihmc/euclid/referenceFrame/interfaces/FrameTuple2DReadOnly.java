package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Read-only interface for a 2D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple2DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple2DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple2DReadOnly} extends {@code Tuple2DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple2DReadOnly}.
 * </p>
 */
public interface FrameTuple2DReadOnly extends Tuple2DReadOnly, ReferenceFrameHolder
{
   /**
    * Tests on a per component basis if this tuple is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two tuples have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameTuple2DReadOnly))
         return false;
      FrameTuple2DReadOnly other = (FrameTuple2DReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this tuple is exactly equal to {@code other}.
    * <p>
    * If the two tuples have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other tuple to compare against this. Not modified.
    * @return {@code true} if the two tuples are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameTuple2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple2DReadOnly.super.equals(other);
   }

   /**
    * Provides a {@code String} representation of this tuple2D. as follows: (x, y)-referenceFrame.
    *
    * @return a string representation of this tuple2D.
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple2DString(format, this);
   }

   /**
    * Tests if this tuple2D is geometrically equal to the given {@code object} to an {@code epsilon}.
    * <p>
    * If the two tuple2Ds have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are geometrically equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameTuple2DReadOnly))
         return false;
      FrameTuple2DReadOnly other = (FrameTuple2DReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return distance(other) <= epsilon;
   }
}

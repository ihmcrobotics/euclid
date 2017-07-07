package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public interface FrameTuple3DReadOnly extends Tuple3DReadOnly, ReferenceFrameHolder
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
   default boolean epsilonEquals(FrameTuple3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple3DReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean equals(FrameTuple3DReadOnly other)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Tuple3DReadOnly.super.equals(other);
   }
}

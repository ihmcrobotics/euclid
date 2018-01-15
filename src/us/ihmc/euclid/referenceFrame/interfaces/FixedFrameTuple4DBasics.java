package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;

public interface FixedFrameTuple4DBasics extends FrameTuple4DReadOnly, Tuple4DBasics
{
   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default void set(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple4DBasics.super.set(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default void setAndAbsolute(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple4DBasics.super.setAndAbsolute(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default void setAndNegate(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple4DBasics.super.setAndNegate(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setAndNormalize(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple4DBasics.super.setAndNormalize(other);
   }
}

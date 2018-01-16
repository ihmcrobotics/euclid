package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public interface FixedFrameTuple4DBasics extends FrameTuple4DReadOnly, Tuple4DBasics
{
   /**
    * Sets this frame tuple to {@code tuple4DReadOnly} and checks that its current frame equal
    * {@code referenceFrame}.
    * 
    * @param referenceFrame the coordinate system in which the given {@code tuple4DReadOnly} is
    *           expressed.
    * @param tuple4DReadOnly the geometry object used to update the geometry object in {@code this}.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(tuple4DReadOnly);
   }

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

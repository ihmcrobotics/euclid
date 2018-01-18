package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

public interface FixedFrameVector2DBasics extends FrameVector2DReadOnly, FixedFrameTuple2DBasics, Vector2DBasics
{
   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setAndNormalize(FrameVector2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Vector2DBasics.super.setAndNormalize(other);
   }
}

package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector4D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * {@code FixedFrameTuple4DBasics} is the base implementation for {@link FrameQuaternion} and
 * {@link FrameVector4D}.
 * <p>
 * In addition to representing a {@link Tuple4DBasics}, a constant {@link ReferenceFrame} is
 * associated to a {@code FixedFrameTuple4DBasics}. This allows, for instance, to enforce, at
 * runtime, that operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * When using this interface, the reference frame of this tuple is assumed to be immutable, i.e. the
 * tuple is always expressed in the same reference frame.
 * </p>
 * <p>
 * Because a {@code FixedFrameTuple4DBasics} extends {@code Tuple4DBasics}, it is compatible with
 * methods only requiring {@code Tuple4DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameTuple4DBasics}.
 * </p>
 */
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
    * Sets this frame tuple components to {@code x}, {@code y}, {@code z}, and {@code s} and checks
    * that its current frame equal {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given components ares expressed.
    * @param x the new x component.
    * @param y the new y component.
    * @param z the new z component.
    * @param s the new s component.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y, z, s);
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
    * Sets this frame tuple to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameTuple4DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setMatchingFrame(FrameTuple4DReadOnly other)
   {
      Tuple4DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndNormalize(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple4DBasics.super.setAndNormalize(other);
   }
}

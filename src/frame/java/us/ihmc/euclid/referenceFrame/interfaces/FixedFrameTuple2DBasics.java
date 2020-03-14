package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * {@code FixedFrameTuple2DBasics} is the base implementation for {@link FramePoint2D} and
 * {@link FrameVector2D}.
 * <p>
 * In addition to representing a {@link Tuple2DBasics}, a constant {@link ReferenceFrame} is
 * associated to a {@code FixedFrameTuple2DBasics}. This allows, for instance, to enforce, at
 * runtime, that operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * When using this interface, the reference frame of this tuple is assumed to be immutable, i.e. the
 * tuple is always expressed in the same reference frame.
 * </p>
 * <p>
 * Because a {@code FixedFrameTuple2DBasics} extends {@code Tuple2DBasics}, it is compatible with
 * methods only requiring {@code Tuple2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameTuple2DBasics}.
 * </p>
 */
public interface FixedFrameTuple2DBasics extends FrameTuple2DReadOnly, Tuple2DBasics
{
   /**
    * Sets this frame tuple to {@code tuple2DReadOnly} and checks that its current frame equal
    * {@code referenceFrame}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple2DReadOnly} is
    *                        expressed.
    * @param tuple2DReadOnly the geometry object used to update the geometry object in {@code this}.
    *                        Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(tuple2DReadOnly);
   }

   /**
    * Sets this frame tuple components to {@code x}, and {@code y} and checks that its current frame
    * equal {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given components are expressed.
    * @param x              the new x component.
    * @param y              the new y component.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, double x, double y)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void set(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.set(other);
   }

   /**
    * Set this frame tuple using the x and y coordinate of the given {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void set(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      Tuple2DBasics.super.set(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameTuple2DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame tuple to set this to. Not modified.
    */
   default void setMatchingFrame(FrameTuple2DReadOnly other)
   {
      Tuple2DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndAbsolute(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndAbsolute(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndNegate(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndNegate(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this frame tuple.
    * @param other  the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndScale(double scalar, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndScale(scalar, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMax(double)}.
    *
    * @param max   the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMax(double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndClipToMax(max, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMin(double)}.
    *
    * @param min   the minimum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMin(double min, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndClipToMin(min, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMinMax(double, double)}.
    *
    * @param min   the minimum value for each component of this frame tuple.
    * @param max   the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMinMax(double min, double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.setAndClipToMinMax(min, max, other);
   }

   /**
    * Adds the given frame tuple to this frame tuple.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other frame tuple to add to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void add(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.add(other);
   }

   /**
    * Sets this frame tuple to the sum of the two given frame tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void add(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple2DBasics.super.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param tuple2      the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple2DBasics.super.add(frameTuple1, tuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple2DBasics.super.add(tuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple to this frame tuple.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other frame tuple to subtract to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void sub(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.sub(other);
   }

   /**
    * Sets this frame tuple to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void sub(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple2DBasics.super.sub(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple2DBasics.super.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2      the second tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple2DBasics.super.sub(frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and adds {@code other}.
    * <p>
    * this = scalar * this + other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other  the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.scaleAdd(scalar, other);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 + frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple2DBasics.super.scaleAdd(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 + frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code tuple1}.
    * @param tuple1      the first tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleAdd(double scalar, Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple2DBasics.super.scaleAdd(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 + tuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param tuple2      the second tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple2DBasics.super.scaleAdd(scalar, frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and subtracts {@code other}.
    * <p>
    * this = scalar * this - other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other  the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.scaleSub(scalar, other);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 - frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple2DBasics.super.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param tuple1      the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleSub(double scalar, Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple2DBasics.super.scaleSub(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 - tuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param tuple2      the second tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple2DBasics.super.scaleSub(scalar, frameTuple1, tuple2);
   }

   /**
    * Performs a linear interpolation from this frame tuple to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other frame tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              this frame tuple, while a value of 1 is equivalent to setting this frame tuple to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void interpolate(FrameTuple2DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Tuple2DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void interpolate(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple2DBasics.super.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code tuple1}, while a value of 1 is equivalent
    *                    to setting this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple2DBasics.super.interpolate(tuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param tuple2      the second tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame tuple to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple2DBasics.super.interpolate(frameTuple1, tuple2, alpha);
   }
}

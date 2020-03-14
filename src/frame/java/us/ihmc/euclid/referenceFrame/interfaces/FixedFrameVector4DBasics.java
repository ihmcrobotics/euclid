package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

/**
 * Write and read interface for a 4D vector expressed in a constant reference frame, i.e. this
 * vector is always expressed in the same reference frame.
 * <p>
 * In addition to representing a {@link Vector4DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFrameVector4DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFrameVector4DBasics} extends {@code Vector4DBasics}, it is compatible with
 * methods only requiring {@code Vector4DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameVector4DBasics}.
 * </p>
 */
public interface FixedFrameVector4DBasics extends FrameVector4DReadOnly, FixedFrameTuple4DBasics, Vector4DBasics
{
   /**
    * Sets this 4D frame vector to represent the given 3D frame vector
    * <p>
    * this.xyz = frameVector3D<br>
    * this.s = 0.0
    * </p>
    *
    * @param frameVector3D the 3D frame vector used to set this 4D frame vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameVector3D} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void set(FrameVector3DReadOnly frameVector3D)
   {
      checkReferenceFrameMatch(frameVector3D);
      Vector4DBasics.super.set(frameVector3D);
   }

   /**
    * Sets this 4D frame vector to represent the given 3D frame point
    * <p>
    * this.xyz = framePoint3D<br>
    * this.s = 1.0
    * </p>
    *
    * @param framePoint3D the 3D frame point used to set this 4D frame vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code framePoint3D} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly framePoint3D)
   {
      checkReferenceFrameMatch(framePoint3D);
      Vector4DBasics.super.set(framePoint3D);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then scales it {@link #scale(double)}.
    *
    * @param scalar               the scale factor to use on this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void setAndScale(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.setAndScale(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then calls {@link #clipToMax(double)}.
    *
    * @param max                  the maximum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void setAndClipToMax(double max, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.setAndClipToMax(max, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then calls {@link #clipToMin(double)}.
    *
    * @param min                  the minimum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void setAndClipToMin(double min, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.setAndClipToMin(min, frameTuple4DReadOnly);
   }

   /**
    * Sets this vector to {@code frameTuple4DReadOnly} and then calls
    * {@link #clipToMinMax(double, double)}.
    *
    * @param min                  the minimum value for each component of this tuple.
    * @param max                  the maximum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void setAndClipToMinMax(double min, double max, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.setAndClipToMinMax(min, max, frameTuple4DReadOnly);
   }

   /**
    * Adds the given tuple to this vector.
    * <p>
    * this = this + frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the tuple to add to this vector.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void add(FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.add(frameTuple4DReadOnly);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first tuple to sum. Not modified.
    * @param tuple2      the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector4DBasics.super.add(frameTuple1, tuple2);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple to sum. Not modified.
    * @param frameTuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector4DBasics.super.add(tuple1, frameTuple2);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first tuple to sum. Not modified.
    * @param frameTuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void add(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Vector4DBasics.super.add(frameTuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple from this frame vector.
    * <p>
    * this = this - frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the frame tuple to subtract from this frame vector.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void sub(FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.sub(frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2      the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector4DBasics.super.sub(frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector4DBasics.super.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void sub(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Vector4DBasics.super.sub(frameTuple1, frameTuple2);
   }

   /**
    * Scales this frame vector and adds {@code frameTuple4DReadOnly}.
    * <p>
    * this = scalar * this + frameTuple4DReadOnly
    * </p>
    *
    * @param scalar               the scale factor to use.
    * @param frameTuple4DReadOnly the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.scaleAdd(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the sum of {@code frameTuple1} scaled and {@code tuple2}.
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
   default void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector4DBasics.super.scaleAdd(scalar, frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the sum of {@code tuple1} scaled and {@code frameTuple2}.
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
   default void scaleAdd(double scalar, Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector4DBasics.super.scaleAdd(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
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
   default void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Vector4DBasics.super.scaleAdd(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Scales this frame vector and subtracts {@code frameTuple4DReadOnly}.
    * <p>
    * this = scalar * this - frameTuple4DReadOnly
    * </p>
    *
    * @param scalar               the scale factor to use.
    * @param frameTuple4DReadOnly the frame tuple to subtract to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.scaleSub(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the difference of {@code frameTuple1} scaled and {@code tuple2}.
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
   default void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector4DBasics.super.scaleSub(scalar, frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code tuple1}.
    * @param tuple1      the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleSub(double scalar, Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector4DBasics.super.scaleSub(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the difference of {@code frameTuple1} scaled and {@code frameTuple2}.
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
   default void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Vector4DBasics.super.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Performs a linear interpolation from this frame vector to {@code frameTuple4DReadOnly} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the frame tuple used for the interpolation. Not modified.
    * @param alpha                the percentage used for the interpolation. A value of 0 will result
    *                             in not modifying this vector, while a value of 1 is equivalent to
    *                             setting this vector to {@code frameTuple4DReadOnly}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void interpolate(FrameTuple4DReadOnly frameTuple4DReadOnly, double alpha)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      Vector4DBasics.super.interpolate(frameTuple4DReadOnly, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first tuple used in the interpolation. Not modified.
    * @param tuple2      the second tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame vector to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame vector to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector4DBasics.super.interpolate(frameTuple1, tuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame vector to {@code tuple1}, while a value of 1 is equivalent
    *                    to setting this frame vector to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector4DBasics.super.interpolate(tuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * frameTuple2
    * </p>
    *
    * @param frameTuple1 the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame vector to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame vector to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void interpolate(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Vector4DBasics.super.interpolate(frameTuple1, frameTuple2, alpha);
   }
}

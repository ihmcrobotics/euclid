package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * {@code FrameVector4D} is a 4D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector4DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector4D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameVector4D} extends {@code Vector4DBasics}, it is compatible with methods
 * only requiring {@code Vector4DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector4D}.
 * </p>
 */
public class FrameVector4D extends FrameTuple4D<FrameVector4D, Vector4D> implements FrameVector4DReadOnly, Vector4DBasics
{
   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector4D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector4D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Vector4D());
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      super(referenceFrame, new Vector4D(x, y, z, s));
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      super(referenceFrame, new Vector4D(vectorArray));
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple4DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple4DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      super(referenceFrame, new Vector4D(tuple4DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector4D(FrameTuple4DReadOnly other)
   {
      super(other.getReferenceFrame(), new Vector4D(other));
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      tuple.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      tuple.setY(y);
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      tuple.setZ(z);
   }

   /** {@inheritDoc} */
   @Override
   public void setS(double s)
   {
      tuple.setS(s);
   }

   /**
    * Sets this 4D frame vector to represent the given 3D frame vector
    * <p>
    * this.xyz = frameVector3D<br>
    * this.s = 0.0
    * </p>
    *
    * @param frameVector3D the 3D frame vector used to set this 4D frame vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameVector3D} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void set(FrameVector3DReadOnly frameVector3D)
   {
      checkReferenceFrameMatch(frameVector3D);
      tuple.set(frameVector3D);
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
    *            frame as {@code this}.
    */
   public final void set(FramePoint3DReadOnly framePoint3D)
   {
      checkReferenceFrameMatch(framePoint3D);
      tuple.set(framePoint3D);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then scales it
    * {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void setAndScale(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.setAndScale(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then calls
    * {@link #clipToMax(double)}.
    *
    * @param max the maximum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void setAndClipToMax(double max, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.setAndClipToMax(max, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to {@code frameTuple4DReadOnly} and then calls
    * {@link #clipToMin(double)}.
    *
    * @param min the minimum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void setAndClipToMin(double min, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.setAndClipToMin(min, frameTuple4DReadOnly);
   }

   /**
    * Sets this vector to {@code frameTuple4DReadOnly} and then calls
    * {@link #clipToMinMax(double, double)}.
    *
    * @param min the minimum value for each component of this tuple.
    * @param max the maximum value for each component of this tuple.
    * @param frameTuple4DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void setAndClipToMinMax(double min, double max, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.setAndClipToMinMax(min, max, frameTuple4DReadOnly);
   }

   /**
    * Adds the given tuple to this vector.
    * <p>
    * this = this + frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the tuple to add to this vector.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void add(FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.add(frameTuple4DReadOnly);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first tuple to sum. Not modified.
    * @param tuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1, tuple2);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple to sum. Not modified.
    * @param frameTuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(tuple1, frameTuple2);
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first tuple to sum. Not modified.
    * @param frameTuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void add(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple from this frame vector.
    * <p>
    * this = this - frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the frame tuple to subtract from this frame vector.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void sub(FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.sub(frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2 the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract from {@code tuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void sub(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   /**
    * Scales this frame vector and adds {@code frameTuple4DReadOnly}.
    * <p>
    * this = scalar * this + frameTuple4DReadOnly
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param frameTuple4DReadOnly the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.scaleAdd(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the sum of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 + tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param tuple2 the second tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scalar, frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the sum of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 + frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleAdd(double scalar, Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 + frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void scaleAdd(double scalar, FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Scales this frame vector and subtracts {@code frameTuple4DReadOnly}.
    * <p>
    * this = scalar * this - frameTuple4DReadOnly
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param frameTuple4DReadOnly the frame tuple to subtract to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.scaleSub(scalar, frameTuple4DReadOnly);
   }

   /**
    * Sets this frame vector to the difference of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 - tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param tuple2 the second tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleSub(scalar, frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scalar, Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the difference of {@code frameTuple1} scaled and
    * {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 - frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void scaleSub(double scalar, FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Performs a linear interpolation from this frame vector to {@code frameTuple4DReadOnly} given
    * the percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * frameTuple4DReadOnly
    * </p>
    *
    * @param frameTuple4DReadOnly the frame tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this vector, while a value of 1 is equivalent to setting this vector to
    *           {@code frameTuple4DReadOnly}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple4DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void interpolate(FrameTuple4DReadOnly frameTuple4DReadOnly, double alpha)
   {
      checkReferenceFrameMatch(frameTuple4DReadOnly);
      tuple.interpolate(frameTuple4DReadOnly, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first tuple used in the interpolation. Not modified.
    * @param tuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame vector to {@code frameTuple1}, while a value of 1 is equivalent to
    *           setting this frame vector to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(FrameTuple4DReadOnly frameTuple1, Tuple4DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.interpolate(frameTuple1, tuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame vector to {@code tuple1}, while a value of 1 is equivalent to setting
    *           this frame vector to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(Tuple4DReadOnly tuple1, FrameTuple4DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(tuple1, frameTuple2, alpha);
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
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame vector to {@code frameTuple1}, while a value of 1 is equivalent to
    *           setting this frame vector to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void interpolate(FrameTuple4DReadOnly frameTuple1, FrameTuple4DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /**
    * Gets the read-only reference to the vector used in {@code this}.
    *
    * @return the vector of {@code this}.
    */
   public final Vector4DReadOnly getVector()
   {
      return tuple;
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame vectors
    * are geometrically similar, i.e. the length of the distance between them is
    * less than or equal to {@code epsilon}.
    *
    * @param other the frame vector to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    * @return {@code true} if the two frame vectors represent the same geometry,
    *            {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(FrameVector4D other, double epsilon)
   {
      checkReferenceFrameMatch(other);

      return Vector4DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame vectors
    * are geometrically similar, i.e. the length of the distance between them is
    * less than or equal to {@code epsilon}.
    *
    * @param other the frame vector to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    * @return {@code true} if the two frame vectors represent the same geometry,
    *            {@code false} otherwise.
    */
   public boolean geometricallyEquals(FrameVector4DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);

      return Vector4DBasics.super.geometricallyEquals(other, epsilon);
   }
}

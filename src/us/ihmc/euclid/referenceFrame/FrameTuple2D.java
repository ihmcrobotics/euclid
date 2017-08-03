package us.ihmc.euclid.referenceFrame;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FrameTuple2D} is the base implementation for {@link FramePoint2D} and
 * {@link FrameVector2D}.
 * <p>
 * In addition to representing a {@link Tuple2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple2D}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple2D} extends {@code Tuple2DBasics}, it is compatible with methods only
 * requiring {@code Tuple2DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameTuple2D}.
 * </p>
 */
public abstract class FrameTuple2D<S extends FrameTuple2D<S, T>, T extends Tuple2DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple2DReadOnly, Tuple2DBasics, Serializable
{
   private static final long serialVersionUID = 6275308250031489785L;

   /** Tuple used to perform the operations. */
   protected final T tuple;

   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new frame tuple and initializes its current reference frame and tuple.
    * <p>
    * The given {@code tuple}'s reference is saved internally for performing all the future
    * operations with this {@code FrameTuple2D}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given tuple is expressed in.
    * @param tuple the tuple that is to be used internally. Reference saved. Will be modified.
    */
   public FrameTuple2D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public final void setX(double x)
   {
      tuple.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public final void setY(double y)
   {
      tuple.setY(y);
   }

   /**
    * Sets this frame tuple to {@code other}.
    * 
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void set(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.set(other);
   }

   /**
    * Set this frame tuple using the x and y coordinate of the given {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public final void set(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      tuple.set(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndAbsolute(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndAbsolute(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndNegate(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndScale(double scaleFactor, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndScale(scaleFactor, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMax(double)}.
    *
    * @param max the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMax(double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMax(max, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMin(double)}.
    *
    * @param min the minimum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMin(double min, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMin(min, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMinMax(double, double)}.
    *
    * @param min the minimum value for each component of this frame tuple.
    * @param max the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMinMax(double min, double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMinMax(min, max, other);
   }

   /**
    * Sets this frame tuple's components to {@code x} and {@code y} and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param x the new value for the x-component of this tuple.
    * @param y the new value for the y-component of this tuple.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y)
   {
      this.referenceFrame = referenceFrame;
      set(x, y);
   }

   /**
    * Sets this frame tuple to {@code tuple2DReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param tuple2DReadOnly the tuple to copy the values from. Not modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param tuple3DReadOnly the tuple to copy the values from. Not modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      this.referenceFrame = referenceFrame;
      this.tuple.set(tuple3DReadOnly);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param tupleArray the array containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startIndex, tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given column vector
    * starting to read from its first row index and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleDenseMatrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given column vector
    * starting to read from {@code startRow} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, tupleDenseMatrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given matrix starting to
    * read from {@code startRow} at the column index {@code column} and sets this tuple frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column the column index to read in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, column, tupleDenseMatrix);
   }

   /**
    * Sets this frame tuple from the reference frame, x and y components of the given
    * {@code frameTuple3DReadOnly}.
    * 
    * @param frameTuple3DReadOnly the tuple to copy the values and reference frame from. Not
    *           modified.
    */
   public final void setIncludingFrame(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this.referenceFrame = frameTuple3DReadOnly.getReferenceFrame();
      tuple.set(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   public final void setIncludingFrame(FrameTuple2DReadOnly frameTuple2d)
   {
      referenceFrame = frameTuple2d.getReferenceFrame();
      tuple.set(frameTuple2d);
   }

   /**
    * Adds the given frame tuple to this frame tuple.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other frame tuple to add to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void add(FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1);
   }

   /**
    * Sets this frame tuple to the sum of the two given frame tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void add(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param tuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(Tuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple to this frame tuple.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other frame tuple to subtract to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void sub(FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1);
   }

   /**
    * Sets this frame tuple to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void sub(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2 the second tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and adds {@code other}.
    * <p>
    * this = scalar * this + other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scaleFactor, frameTuple1);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
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
   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code tuple1} scaled and {@code frameTuple2}.
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
   public final void scaleAdd(double scaleFactor, Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code tuple2}.
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
   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scaleFactor, frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and subtracts {@code other}.
    * <p>
    * this = scalar * this - other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void scaleSub(double scalar, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleSub(scalar, other);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code frameTuple2}.
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
   public final void scaleSub(double scalar, FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param tuple1 the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scalar, Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 - tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scalar, FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleSub(scalar, frameTuple1, tuple2);
   }

   /**
    * Performs a linear interpolation from this frame tuple to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other frame tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this frame tuple, while a value of 1 is equivalent to setting this frame
    *           tuple to {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void interpolate(FrameTuple2DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      tuple.interpolate(other, alpha);
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
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code frameTuple1}, while a value of 1 is equivalent to setting
    *           this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void interpolate(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code tuple1}, while a value of 1 is equivalent to setting this
    *           frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(Tuple2DReadOnly tuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(tuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param tuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code frameTuple1}, while a value of 1 is equivalent to setting
    *           this frame tuple to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.interpolate(frameTuple1, tuple2, alpha);
   }

   /** {@inheritDoc} */
   @Override
   public final void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of
       * checking both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /**
    * Performs a transformation of the tuple such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this tuple 2D can result in a tuple 3D. This method projects the result of the
    * transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the tuple is to be expressed.
    */
   public final void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final double getX()
   {
      return tuple.getX();
   }

   /** {@inheritDoc} */
   @Override
   public final double getY()
   {
      return tuple.getY();
   }

   /** {@inheritDoc} */
   @Override
   public final void get(Tuple2DBasics tuple2dToPack)
   {
      tuple2dToPack.set(tuple);
   }
}

package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * {@code FrameTuple4D} is the base implementation for {@link FramePoint4D} and
 * {@link FrameVector4D}.
 * <p>
 * In addition to representing a {@link Tuple4DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple4D}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple4D} extends {@code Tuple4DBasics}, it is compatible with methods only
 * requiring {@code Tuple4DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameTuple4D}.
 * </p>
 */
public abstract class FrameTuple4D<S extends FrameTuple4D<S, T>, T extends Tuple4DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple4DReadOnly, Tuple4DBasics
{
   /** Tuple used to perform the operations. */
   protected final T tuple;

   /**
    * Creates a new frame tuple and initializes its current reference frame and tuple.
    * <p>
    * The given {@code tuple}'s reference is saved internally for performing all the future
    * operations with this {@code FrameTuple4D}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given tuple is expressed in.
    * @param tuple the tuple that is to be used internally. Reference saved. Will be modified.
    */
   public FrameTuple4D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public final void set(double x, double y, double z, double s)
   {
      tuple.set(x, y, z, s);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void set(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.set(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndAbsolute(FrameTuple4DReadOnly other)
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
   public final void setAndNegate(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
   }

   /**
    * Sets this frame tuple's components to {@code x}, {@code y}, {@code z} and {@code s} and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param x the new value for the x-component of this tuple.
    * @param y the new value for the y-component of this tuple.
    * @param z the new value for the z-component of this tuple.
    * @param s the new value for the s-component of this tuple.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(x, y, z, s);
   }

   /**
    * Sets this frame tuple to {@code tuple4DReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param tuple4DReadOnly the tuple to copy the values from. Not modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      this.referenceFrame = referenceFrame;
      this.tuple.set(tuple4DReadOnly);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
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
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
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
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from its first row index and sets this tuple frame to
    * {@code referenceFrame}.
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
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from {@code startRow} and sets this tuple frame to
    * {@code referenceFrame}.
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
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given matrix
    * starting to read from {@code startRow} at the column index {@code column} and sets this tuple
    * frame to {@code referenceFrame}.
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
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   public final void setIncludingFrame(FrameTuple4DReadOnly other)
   {
      referenceFrame = other.getReferenceFrame();
      tuple.set(other);
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
   public final double getZ()
   {
      return tuple.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public final double getS()
   {
      return tuple.getS();
   }

   /** {@inheritDoc} */
   @Override
   public final void get(Tuple4DBasics tuple4dToPack)
   {
      tuple4dToPack.set(tuple);
   }
}

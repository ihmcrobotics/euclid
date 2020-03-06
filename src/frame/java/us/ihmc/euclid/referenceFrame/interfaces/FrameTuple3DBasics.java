package us.ihmc.euclid.referenceFrame.interfaces;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FrameTuple3DBasics} is the base implementation for {@link FramePoint3D} and
 * {@link FrameVector3D}.
 * <p>
 * In addition to representing a {@link Tuple3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple3DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * This interface allows for changing the reference frame in which this tuple is expressed.
 * </p>
 * <p>
 * Because a {@code FrameTuple3DBasics} extends {@code Tuple3DBasics}, it is compatible with methods
 * only requiring {@code Tuple3DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameTuple3DBasics}.
 * </p>
 */
public interface FrameTuple3DBasics extends FixedFrameTuple3DBasics
{
   /**
    * Sets the reference frame of this tuple without updating or modifying its x, y, and z components.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets all the components of this frame tuple to zero and sets the current reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this tuple.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets all the components of this tuple to {@link Double#NaN} and sets the current reference frame
    * to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this tuple.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this frame tuple's components to {@code x}, {@code y}, and {@code z} and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param x              the new value for the x-component of this tuple.
    * @param y              the new value for the y-component of this tuple.
    * @param z              the new value for the z-component of this tuple.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      setReferenceFrame(referenceFrame);
      set(x, y, z);
   }

   /**
    * Sets this frame tuple x and y components to {@code tuple2DReadOnly}'s x and y components, the
    * z-component to the given {@code z}, and sets the current frame to {@code referenceFrame}.
    *
    * @param referenceFrame  the new reference frame for this frame tuple.
    * @param tuple2DReadOnly the tuple to copy the values from. Not modified.
    * @param z               the new z-coordinate for this frame tuple.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly, double z)
   {
      setReferenceFrame(referenceFrame);
      set(tuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame  the new reference frame for this frame tuple.
    * @param tuple3DReadOnly the tuple to copy the values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(tuple3DReadOnly);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param tupleArray     the array containing the new values for this tuple's components. Not
    *                       modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
   {
      setReferenceFrame(referenceFrame);
      set(tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startIndex     the first index to start reading from in the array.
    * @param tupleArray     the array containing the new values for this tuple's components. Not
    *                       modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
   {
      setReferenceFrame(referenceFrame);
      set(startIndex, tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from its first row index and sets this tuple frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param matrix         the column vector containing the new values for this tuple's components.
    *                       Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, DMatrixRMaj matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from {@code startRow} and sets this tuple frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow       the first row index to start reading in the dense-matrix.
    * @param matrix         the column vector containing the new values for this tuple's components.
    *                       Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DMatrixRMaj matrix)
   {
      setReferenceFrame(referenceFrame);
      set(startRow, matrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given matrix
    * starting to read from {@code startRow} at the column index {@code column} and sets this tuple
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow       the first row index to start reading in the dense-matrix.
    * @param column         the column index to read in the dense-matrix.
    * @param matrix         the column vector containing the new values for this tuple's components.
    *                       Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DMatrixRMaj matrix)
   {
      setReferenceFrame(referenceFrame);
      set(startRow, column, matrix);
   }

   /**
    * Sets this frame tuple x and y components to {@code tuple2DReadOnly}'s x and y components, the
    * z-component to the given {@code z}, and sets the current frame to
    * {@code frameTuple2DReadOnly.getReferenceFrame()}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the values and reference frame from. Not modified.
    * @param z                    the new z-coordinate for this frame tuple.
    */
   default void setIncludingFrame(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
   {
      setReferenceFrame(frameTuple2DReadOnly.getReferenceFrame());
      set((Tuple2DReadOnly) frameTuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   default void setIncludingFrame(FrameTuple3DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set((Tuple3DReadOnly) other);
   }

}

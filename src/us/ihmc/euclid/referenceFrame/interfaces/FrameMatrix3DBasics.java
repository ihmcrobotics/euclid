package us.ihmc.euclid.referenceFrame.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Write and read interface for generic matrix 3D expressed in a changeable reference frame, i.e.
 * the reference frame in which this matrix is expressed can be changed.
 * <p>
 * In addition to representing a {@link Matrix3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameMatrix3DBasics}. This allows, for instance, to enforce, at runtime, that operations
 * on points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameMatrix3DBasics} extends {@code Matrix3DBasics}, it is compatible with
 * methods only requiring {@code Matrix3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameMatrix3DBasics}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FrameMatrix3DBasics extends FixedFrameMatrix3DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this matrix without updating or modifying any of its coefficients.
    *
    * @param referenceFrame the new reference frame for this frame matrix.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets all the coefficients of this matrix to zero and sets the current reference frame to
    * {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this matrix.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets all the coefficients of this matrix to {@link Double#NaN} and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this matrix.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this matrix to {@code matrix3DReadOnly} and sets its reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param matrix3DReadOnly the other matrix to copy the values of. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(matrix3DReadOnly);
   }

   /**
    * Sets this matrix to {@code other}.
    *
    * @param other the other matrix to copy the values and reference frame from. Not modified.
    */
   default void setIncludingFrame(FrameMatrix3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Copies the values in the given array into this matrix as follows:
    *
    * <pre>
    *     / matrixArray[0]  matrixArray[1]  matrixArray[2] \
    * m = | matrixArray[3]  matrixArray[4]  matrixArray[5] |
    *     \ matrixArray[6]  matrixArray[7]  matrixArray[8] /
    * </pre>
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param matrixArray the array containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double[] matrixArray)
   {
      setReferenceFrame(referenceFrame);
      set(matrixArray);
   }

   /**
    * Copies the values in the given array into this matrix as follows:
    *
    * <pre>
    *     / matrixArray[startIndex + 0]  matrixArray[startIndex + 1]  matrixArray[startIndex + 2] \
    * m = | matrixArray[startIndex + 3]  matrixArray[startIndex + 4]  matrixArray[startIndex + 5] |
    *     \ matrixArray[startIndex + 6]  matrixArray[startIndex + 7]  matrixArray[startIndex + 8] /
    * </pre>
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param startIndex the first index to start reading from in the array.
    * @param matrixArray the array containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] matrixArray)
   {
      setReferenceFrame(referenceFrame);
      set(startIndex, matrixArray);
   }

   /**
    * Copies the values in the given dense-matrix into this matrix.
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param matrix the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
   }

   /**
    * Copies the values in the given dense-matrix into this matrix given index offsets for the row
    * and column.
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param startRow the first row index to start reading from in the dense-matrix.
    * @param startColumn the first column index to start reading from in the dense-matrix.
    * @param matrix the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int startColumn, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(startRow, startColumn, matrix);
   }
}

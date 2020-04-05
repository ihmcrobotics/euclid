package us.ihmc.euclid.referenceFrame.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read interface for a 3-by-3 matrix object with a restricted writing interface.
 * <p>
 * The matrix is expressed in a mutable reference frame, i.e. the reference frame of this object can
 * be changed via this interface.
 * </p>
 * <p>
 * In this interface, the matrix is assumed to be generic purpose. Therefore, the algorithms used
 * here are limited to generic applications without violating potential constraints of more specific
 * matrices such as a rotation matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameCommonMatrix3DBasics extends FixedFrameCommonMatrix3DBasics, FrameChangeable
{
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
    * @param matrix         the other matrix to copy the values of. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
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
    * @param matrixArray    the array containing the new values for this matrix. Not modified.
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
    * @param startIndex     the first index to start reading from in the array.
    * @param matrixArray    the array containing the new values for this matrix. Not modified.
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
    * @param matrix         the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
   }

   /**
    * Copies the values in the given dense-matrix into this matrix given index offsets for the row and
    * column.
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param startRow       the first row index to start reading from in the dense-matrix.
    * @param startColumn    the first column index to start reading from in the dense-matrix.
    * @param matrix         the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int startColumn, DenseMatrix64F matrix)
   {
      setReferenceFrame(referenceFrame);
      set(startRow, startColumn, matrix);
   }

   /**
    * Sets the 9 coefficients of this matrix to the given ones and sets its reference frame.
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param m00            the new 1st row 1st column coefficient for this matrix.
    * @param m01            the new 1st row 2nd column coefficient for this matrix.
    * @param m02            the new 1st row 3rd column coefficient for this matrix.
    * @param m10            the new 2nd row 1st column coefficient for this matrix.
    * @param m11            the new 2nd row 2nd column coefficient for this matrix.
    * @param m12            the new 2nd row 3rd column coefficient for this matrix.
    * @param m20            the new 3rd row 1st column coefficient for this matrix.
    * @param m21            the new 3rd row 2nd column coefficient for this matrix.
    * @param m22            the new 3rd row 3rd column coefficient for this matrix.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21,
                                  double m22)
   {
      setReferenceFrame(referenceFrame);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each row and sets its
    * reference frame.
    *
    * <pre>
    *        /  firstRow.getX()  firstRow.getY()  firstRow.getZ() \
    * this = | secondRow.getX() secondRow.getY() secondRow.getZ() |
    *        \  thirdRow.getX()  thirdRow.getY()  thirdRow.getZ() /
    * </pre>
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param firstRow       the tuple holding onto the values of the first row. Not modified.
    * @param secondRow      the tuple holding onto the values of the second row. Not modified.
    * @param thirdRow       the tuple holding onto the values of the third row. Not modified.
    */
   default void setRowsIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly firstRow, Tuple3DReadOnly secondRow, Tuple3DReadOnly thirdRow)
   {
      setReferenceFrame(referenceFrame);
      setRows(firstRow, secondRow, thirdRow);
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each row and sets its
    * reference frame.
    *
    * <pre>
    *        /  firstRow.getX()  firstRow.getY()  firstRow.getZ() \
    * this = | secondRow.getX() secondRow.getY() secondRow.getZ() |
    *        \  thirdRow.getX()  thirdRow.getY()  thirdRow.getZ() /
    * </pre>
    *
    * @param firstRow  the tuple holding onto the values of the first row. Not modified.
    * @param secondRow the tuple holding onto the values of the second row. Not modified.
    * @param thirdRow  the tuple holding onto the values of the third row. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   default void setRowsIncludingFrame(FrameTuple3DReadOnly firstRow, FrameTuple3DReadOnly secondRow, FrameTuple3DReadOnly thirdRow)
   {
      firstRow.checkReferenceFrameMatch(secondRow, thirdRow);
      setRowsIncludingFrame(firstRow.getReferenceFrame(), firstRow, secondRow, thirdRow);
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each column and sets its
    * reference frame.
    *
    * <pre>
    *        / firstColumn.getX() secondColumn.getX() thirdColumn.getX() \
    * this = | firstColumn.getY() secondColumn.getY() thirdColumn.getY() |
    *        \ firstColumn.getZ() secondColumn.getZ() thirdColumn.getZ() /
    * </pre>
    *
    * @param referenceFrame the new reference frame to be associated with this matrix.
    * @param firstColumn    the tuple holding onto the values of the first column. Not modified.
    * @param secondColumn   the tuple holding onto the values of the second column. Not modified.
    * @param thirdColumn    the tuple holding onto the values of the third column. Not modified.
    */
   default void setColumnsIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly firstColumn, Tuple3DReadOnly secondColumn, Tuple3DReadOnly thirdColumn)
   {
      setReferenceFrame(referenceFrame);
      setColumns(firstColumn, secondColumn, thirdColumn);
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each column and sets its
    * reference frame.
    *
    * <pre>
    *        / firstColumn.getX() secondColumn.getX() thirdColumn.getX() \
    * this = | firstColumn.getY() secondColumn.getY() thirdColumn.getY() |
    *        \ firstColumn.getZ() secondColumn.getZ() thirdColumn.getZ() /
    * </pre>
    *
    * @param firstColumn  the tuple holding onto the values of the first column. Not modified.
    * @param secondColumn the tuple holding onto the values of the second column. Not modified.
    * @param thirdColumn  the tuple holding onto the values of the third column. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   default void setColumnsIncludingFrame(FrameTuple3DReadOnly firstColumn, FrameTuple3DReadOnly secondColumn, FrameTuple3DReadOnly thirdColumn)
   {
      firstColumn.checkReferenceFrameMatch(secondColumn, thirdColumn);
      setColumnsIncludingFrame(firstColumn.getReferenceFrame(), firstColumn, secondColumn, thirdColumn);
   }
}

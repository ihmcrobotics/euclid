package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for any type of 3-by-3 matrix expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Matrix3DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameMatrix3DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on matrices occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameMatrix3DReadOnly} extends {@code Matrix3DReadOnly}, it is compatible with
 * methods only requiring {@code Matrix3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameMatrix3DReadOnly}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameMatrix3DReadOnly extends Matrix3DReadOnly, ReferenceFrameHolder
{
   /**
    * Packs a column of this matrix into a 3D frame tuple.
    *
    * @param column       the index of the column to pack.
    * @param columnToPack the tuple in which the column of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code column} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code columnToPack} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getColumn(int column, FixedFrameTuple3DBasics columnToPack)
   {
      checkReferenceFrameMatch(columnToPack);
      Matrix3DReadOnly.super.getColumn(column, columnToPack);
   }

   /**
    * Packs a column of this matrix into a 3D frame tuple.
    *
    * @param column       the index of the column to pack.
    * @param columnToPack the tuple in which the column of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void getColumn(int column, FrameTuple3DBasics columnToPack)
   {
      columnToPack.setReferenceFrame(getReferenceFrame());
      Matrix3DReadOnly.super.getColumn(column, columnToPack);
   }

   /**
    * Packs a row of this matrix into a 3D frame tuple.
    *
    * @param row       the index of the row to pack.
    * @param rowToPack the array in which the row of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code row} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code rowToPack} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getRow(int row, FixedFrameTuple3DBasics rowToPack)
   {
      checkReferenceFrameMatch(rowToPack);
      Matrix3DReadOnly.super.getRow(row, rowToPack);
   }

   /**
    * Packs a row of this matrix into a 3D frame tuple.
    *
    * @param row       the index of the row to pack.
    * @param rowToPack the array in which the row of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void getRow(int row, FrameTuple3DBasics rowToPack)
   {
      rowToPack.setReferenceFrame(getReferenceFrame());
      Matrix3DReadOnly.super.getRow(row, rowToPack);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this matrix and add the result to the tuple.
    * <p>
    * tupleToTransform = tupleToTransform + this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and add the result to
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = tupleTransformed + this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to add the result to. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and add the result to
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = tupleTransformed + this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to add the result to. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and add the result to
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = tupleTransformed + this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to add the result to. Modified.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal            the tuple to transform. Not modified.
    * @param tupleTransformed         the tuple to store the result. Modified.
    * @param checkIfRotationInXYPlane whether this method should assert that this matrix represents a
    *                                 transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal            the tuple to transform. Not modified.
    * @param tupleTransformed         the tuple to store the result. Modified.
    * @param checkIfRotationInXYPlane whether this method should assert that this matrix represents a
    *                                 transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal            the tuple to transform. Not modified.
    * @param tupleTransformed         the tuple to store the result. Modified.
    * @param checkIfRotationInXYPlane whether this method should assert that this matrix represents a
    *                                 transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /**
    * Transforms the given 3D matrix by this matrix.
    * <p>
    * matrixToTransform = this * matrixToTransform * this<sup>-1</sup>
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this matrix and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal * this<sup>-1</sup>
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      transform((Matrix3DReadOnly) matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this matrix and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal * this<sup>-1</sup>
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      transform(matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this matrix and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal * this<sup>-1</sup>
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code matrixOriginal} or
    *                                         {@code matrixTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      checkReferenceFrameMatch(matrixTransformed);
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this * vectorToTransform.xyz
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code vectorToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code vectorOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code vectorTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if either {@code vectorOriginal} or
    *                                         {@code vectorTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform(tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if this matrix does not represent a transformation in the
    *                                         XY plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform(tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code tupleOriginal} or
    *                                         {@code tupleTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform * this
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Not modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code matrixTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform(matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code matrixOriginal} or
    *                                         {@code matrixTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector by this matrix.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this<sup>-1</sup> * vectorToTransform.xyz
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code vectorToTransform} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this matrix and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code vectorOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      inverseTransform((Vector4DReadOnly) vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this matrix and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code vectorTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      inverseTransform(vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this matrix and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws SingularMatrixException         if this matrix is not invertible.
    * @throws ReferenceFrameMismatchException if either {@code vectorOriginal} or
    *                                         {@code vectorTransformed} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Tests on a per component basis if this matrix is exactly equal to {@code other}.
    * <p>
    * If the two matrices have different frames, this method returns {@code false}.
    * </p>
    * <p>
    * The method returns {@code false} if the given matrix is {@code null}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @return {@code true} if the two matrices are exactly equal component-wise and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameMatrix3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Matrix3DReadOnly.super.equals(other);
   }

   /**
    * Tests on a per coefficient basis if this matrix is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two matrices have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameMatrix3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Matrix3DReadOnly.super.epsilonEquals(other, epsilon);
   }
}

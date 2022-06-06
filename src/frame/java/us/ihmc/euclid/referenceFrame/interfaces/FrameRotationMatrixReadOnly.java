package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface used for 3-by-3 rotation matrices expressed in a given reference frame.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameRotationMatrixReadOnly extends RotationMatrixReadOnly, FrameOrientation3DReadOnly, FrameMatrix3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void subTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      subTransform(tupleToTransform, tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void subTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.subTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void subTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.subTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void subTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.subTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      RotationMatrixReadOnly.super.inverseTransform(vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      RotationMatrixReadOnly.super.transform(vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      RotationMatrixReadOnly.super.transform(matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      RotationMatrixReadOnly.super.inverseTransform(matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }
   
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameMatrix3DString(format, this);
   }
}

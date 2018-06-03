package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public interface FrameMatrix3DReadOnly extends Matrix3DReadOnly, ReferenceFrameHolder
{
   default void getColumn(int column, FixedFrameTuple3DBasics columnToPack)
   {
      checkReferenceFrameMatch(columnToPack);
      Matrix3DReadOnly.super.getColumn(column, columnToPack);
   }

   default void getColumn(int column, FrameTuple3DBasics columnToPack)
   {
      columnToPack.setReferenceFrame(getReferenceFrame());
      Matrix3DReadOnly.super.getColumn(column, columnToPack);
   }

   default void getRow(int row, FixedFrameTuple3DBasics rowToPack)
   {
      checkReferenceFrameMatch(rowToPack);
      Matrix3DReadOnly.super.getRow(row, rowToPack);
   }

   default void getRow(int row, FrameTuple3DBasics rowToPack)
   {
      rowToPack.setReferenceFrame(getReferenceFrame());
      Matrix3DReadOnly.super.getRow(row, rowToPack);
   }

   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   default void transform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      transform((Matrix3DReadOnly) matrixOriginal, matrixTransformed);
   }

   default void transform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      transform(matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      checkReferenceFrameMatch(matrixTransformed);
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, tupleTransformed);
   }

   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform(tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform, true);
   }

   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform(tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform(matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      inverseTransform((Vector4DReadOnly) vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      inverseTransform(vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   default boolean epsilonEquals(FrameMatrix3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Matrix3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean equals(FrameMatrix3DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Matrix3DReadOnly.super.equals(other);
   }
}

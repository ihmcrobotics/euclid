package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public interface FrameRotationMatrixReadOnly extends RotationMatrixReadOnly, FrameOrientation3DReadOnly, FrameMatrix3DReadOnly
{
   default double distance(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return RotationMatrixReadOnly.super.distance(other);
   }

   @Override
   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   @Override
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform);
   }

   @Override
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform);
   }

   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      RotationMatrixReadOnly.super.inverseTransform(vectorToTransform);
   }

   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      RotationMatrixReadOnly.super.transform(vectorToTransform);
   }

   @Override
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform);
   }

   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform);
   }

   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   @Override
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.inverseTransform(tupleToTransform, checkIfTransformInXYPlane);
   }

   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   @Override
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   @Override
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      RotationMatrixReadOnly.super.transform(matrixToTransform);
   }

   @Override
   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      RotationMatrixReadOnly.super.inverseTransform(matrixToTransform);
   }

   @Override
   default void transform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      RotationMatrixReadOnly.super.transform(tupleToTransform, checkIfTransformInXYPlane);
   }

   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      RotationMatrixReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      RotationMatrixReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      RotationMatrixReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      RotationMatrixReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      RotationMatrixReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   @Override
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      RotationMatrixReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   default boolean geometricallyEquals(FrameRotationMatrixReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return RotationMatrixReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

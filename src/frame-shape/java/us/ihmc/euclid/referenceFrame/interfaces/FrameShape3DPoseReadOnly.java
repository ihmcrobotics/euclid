package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface FrameShape3DPoseReadOnly extends Shape3DPoseReadOnly, ReferenceFrameHolder
{
   @Override
   FrameRotationMatrixReadOnly getShapeOrientation();

   @Override
   FramePoint3DReadOnly getShapePosition();

   @Override
   FrameVector3DReadOnly getXAxis();

   @Override
   FrameVector3DReadOnly getYAxis();

   @Override
   FrameVector3DReadOnly getZAxis();

   default void getRotation(FixedFrameOrientation3DBasics orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      Shape3DPoseReadOnly.super.getRotation(orientationToPack);
   }

   default void getRotation(FrameOrientation3DBasics orientationToPack)
   {
      orientationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.getRotation(orientationToPack);
   }

   default void getRotation(FixedFrameRotationMatrixBasics rotationMatrixToPack)
   {
      checkReferenceFrameMatch(rotationMatrixToPack);
      Shape3DPoseReadOnly.super.getRotation(rotationMatrixToPack);
   }

   default void getRotation(FrameRotationMatrixBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.getRotation(rotationMatrixToPack);
   }

   default void getRotation(FixedFrameVector3DBasics rotationVectorToPack)
   {
      checkReferenceFrameMatch(rotationVectorToPack);
      Shape3DPoseReadOnly.super.getRotation(rotationVectorToPack);
   }

   default void getRotation(FrameVector3DBasics rotationVectorToPack)
   {
      rotationVectorToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.getRotation(rotationVectorToPack);
   }

   default void getRotationEuler(FixedFrameVector3DBasics eulerAnglesToPack)
   {
      checkReferenceFrameMatch(eulerAnglesToPack);
      Shape3DPoseReadOnly.super.getRotationEuler(eulerAnglesToPack);
   }
   
   default void getRotationEuler(FrameVector3DBasics eulerAnglesToPack)
   {
      eulerAnglesToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.getRotationEuler(eulerAnglesToPack);
   }

   default void getTranslation(FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(translationToPack);
      Shape3DPoseReadOnly.super.getTranslation(translationToPack);
   }

   default void getTranslation(FrameTuple3DBasics translationToPack)
   {
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.getTranslation(translationToPack);
   }

   default void get(FixedFrameOrientation3DBasics orientationToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(orientationToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(orientationToPack, translationToPack);
   }

   default void get(FrameOrientation3DBasics orientationToPack, FrameTuple3DBasics translationToPack)
   {
      orientationToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(orientationToPack, translationToPack);
   }

   default void get(FixedFrameRotationMatrixBasics rotationMatrixToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(rotationMatrixToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(rotationMatrixToPack, translationToPack);
   }

   default void get(FrameRotationMatrixBasics rotationMatrixToPack, FrameTuple3DBasics translationToPack)
   {
      rotationMatrixToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(rotationMatrixToPack, translationToPack);
   }

   default void get(FixedFrameVector3DBasics rotationVectorToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(rotationVectorToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(rotationVectorToPack, translationToPack);
   }

   default void get(FrameVector3DBasics rotationVectorToPack, FrameTuple3DBasics translationToPack)
   {
      rotationVectorToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(rotationVectorToPack, translationToPack);
   }

   default void transform(FixedFramePoint3DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform);
   }

   default void transform(FramePoint3DReadOnly pointOriginal, FixedFramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   default void transform(FramePoint3DReadOnly pointOriginal, FramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   default void transform(FixedFrameVector3DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   default void transform(FrameVector3DReadOnly vectorOriginal, FixedFrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FrameVector3DReadOnly vectorOriginal, FrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      Shape3DPoseReadOnly.super.transform(orientationToTransform);
   }

   default void transform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      Shape3DPoseReadOnly.super.transform(orientationOriginal, orientationTransformed);
   }

   default void transform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(orientationOriginal, orientationTransformed);
   }

   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FixedFramePoint2DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform);
   }

   default void transform(FixedFramePoint2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform, checkIfTransformInXYPlane);
   }

   default void transform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   default void transform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   default void transform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   default void transform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   default void transform(FixedFrameVector2DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   default void transform(FixedFrameVector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform, checkIfTransformInXYPlane);
   }

   default void transform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   default void transform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   default void transform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.transform(matrixToTransform);
   }

   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   default void transform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   default void transform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.transform(matrixToTransform);
   }

   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(FixedFramePoint3DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform);
   }

   default void inverseTransform(FramePoint3DReadOnly pointOriginal, FixedFramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   default void inverseTransform(FramePoint3DReadOnly pointOriginal, FramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   default void inverseTransform(FixedFrameVector3DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   default void inverseTransform(FrameVector3DReadOnly vectorOriginal, FixedFrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FrameVector3DReadOnly vectorOriginal, FrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(orientationToTransform);
   }

   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FixedFramePoint2DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform);
   }

   default void inverseTransform(FixedFramePoint2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FixedFrameVector2DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   default void inverseTransform(FixedFrameVector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(matrixToTransform);
   }

   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(matrixToTransform);
   }

   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   default boolean equals(FrameShape3DPoseReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }

   default boolean epsilonEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Shape3DPoseReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Shape3DPoseReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

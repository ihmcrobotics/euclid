package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public interface FixedFrameShape3DPoseBasics extends Shape3DPoseBasics, FrameShape3DPoseReadOnly
{
   @Override
   FixedFrameRotationMatrixBasics getShapeOrientation();

   @Override
   FixedFramePoint3DBasics getShapePosition();

   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   default void set(FramePose3DReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   default void set(FrameShape3DPoseReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation, translation);
   }

   default void set(FrameOrientation3DReadOnly orientation, FrameTuple3DReadOnly translation)
   {
      orientation.checkReferenceFrameMatch(translation);
      set(orientation.getReferenceFrame(), orientation, translation);
   }

   default void set(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(rotationMatrix, translation);
   }

   default void set(FrameRotationMatrixReadOnly rotationMatrix, FrameTuple3DReadOnly translation)
   {
      rotationMatrix.checkReferenceFrameMatch(translation);
      set(rotationMatrix.getReferenceFrame(), rotationMatrix, translation);
   }

   default void setAndInvert(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.setAndInvert(other);
   }

   default void setRotationAndZeroTranslation(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(orientation);
   }

   default void setRotationAndZeroTranslation(FrameRotationMatrixReadOnly rotationMatrix)
   {
      checkReferenceFrameMatch(rotationMatrix);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(rotationMatrix);
   }

   default void setRotationAndZeroTranslation(FrameVector3DReadOnly rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(rotationVector);
   }

   default void setRotationEulerAndZeroTranslation(FrameVector3DReadOnly eulerAngles)
   {
      checkReferenceFrameMatch(eulerAngles);
      Shape3DPoseBasics.super.setRotationEulerAndZeroTranslation(eulerAngles);
   }

   default void setTranslationAndIdentityRotation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.setTranslationAndIdentityRotation(translation);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePose3DReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameOrientation3DReadOnly orientation, FrameTuple3DReadOnly translation)
   {
      orientation.checkReferenceFrameMatch(translation);
      setMatchingFrame(orientation.getReferenceFrame(), orientation, translation);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameRotationMatrixReadOnly rotationMatrix, FrameTuple3DReadOnly translation)
   {
      rotationMatrix.checkReferenceFrameMatch(translation);
      setMatchingFrame(rotationMatrix.getReferenceFrame(), rotationMatrix, translation);
   }

   default void appendTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.appendTranslation(translation);
   }

   default void prependTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.prependTranslation(translation);
   }

   default void multiply(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiply(other);
   }

   default void multiplyInvertThis(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiplyInvertThis(other);
   }

   default void multiplyInvertOther(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiplyInvertOther(other);
   }

   default void preMultiply(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiply(other);
   }

   default void preMultiplyInvertThis(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiplyInvertThis(other);
   }

   default void preMultiplyInvertOther(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiplyInvertOther(other);
   }
}

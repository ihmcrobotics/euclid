package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read and write interface for representing the pose of a shape 3D expressed in a constant
 * reference frame, i.e. the reference frame of this object cannot be changed via this interface.
 * <p>
 * While the main use-case of a {@code FixedFrameShape3DPoseBasics} is to describe the pose of a
 * shape 3D, it is also used to represent the transform from the shape local coordinate system to
 * the reference frame coordinates, such that it can be used to transform geometry back and forth
 * between the two coordinate systems.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameShape3DPoseBasics extends Shape3DPoseBasics, FrameShape3DPoseReadOnly
{
   /** {@inheritDoc} */
   @Override
   FixedFrameRotationMatrixBasics getShapeOrientation();

   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getShapePosition();

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose 3D to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D.
    *
    * @param pose the pose 3D to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   /**
    * Sets this pose to the given rigid-body transform.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the rigid-body transform to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   /**
    * Sets this pose to the given rigid-body transform.
    *
    * @param pose the rigid-body transform to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param orientation    the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation, translation);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    *
    * @param orientation the orientation used to set the rotation part of this pose. Not modified.
    * @param translation the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameOrientation3DReadOnly orientation, FrameTuple3DReadOnly translation)
   {
      orientation.checkReferenceFrameMatch(translation);
      set(orientation.getReferenceFrame(), orientation, translation);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param rotationMatrix the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(rotationMatrix, translation);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    *
    * @param rotationMatrix the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameRotationMatrixReadOnly rotationMatrix, FrameTuple3DReadOnly translation)
   {
      rotationMatrix.checkReferenceFrameMatch(translation);
      set(rotationMatrix.getReferenceFrame(), rotationMatrix, translation);
   }

   /**
    * Sets this pose to {@code other} and then inverts it.
    *
    * @param other the other pose transform to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndInvert(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.setAndInvert(other);
   }

   /**
    * Sets the rotation part of this pose to the given orientation and sets the translation part to
    * zero.
    *
    * @param orientation the orientation used to set the rotation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setRotationAndZeroTranslation(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(orientation);
   }

   /**
    * Sets the rotation part of this pose to the given rotation matrix and sets the translation part to
    * zero.
    *
    * @param rotationMatrix the orientation used to set the rotation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setRotationAndZeroTranslation(FrameRotationMatrixReadOnly rotationMatrix)
   {
      checkReferenceFrameMatch(rotationMatrix);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(rotationMatrix);
   }

   /**
    * Sets the rotation part of this pose to the given rotation vector and sets the translation part to
    * zero.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part of this pose. Not
    *                       modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setRotationAndZeroTranslation(FrameVector3DReadOnly rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      Shape3DPoseBasics.super.setRotationAndZeroTranslation(rotationVector);
   }

   /**
    * Sets the rotation part of this pose to represent the same orientation as the given Euler angles
    * {@code eulerAngles} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRollAndZeroTranslation(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setRotationEulerAndZeroTranslation(FrameVector3DReadOnly eulerAngles)
   {
      checkReferenceFrameMatch(eulerAngles);
      Shape3DPoseBasics.super.setRotationEulerAndZeroTranslation(eulerAngles);
   }

   /**
    * Sets the translation part of this pose and sets the rotation part to identity.
    *
    * @param translation tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setTranslationAndIdentityRotation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.setTranslationAndIdentityRotation(translation);
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose 3D to set this to. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePose3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose 3D to set this to. Not modified.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }

   /**
    * Sets this pose to the given rigid-body transform.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the rigid-body transform to copy the values from. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this pose to the given pose.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameShape3DPoseReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose to copy the values from. Not modified.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Orientation3DReadOnly, Tuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param orientation    the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameOrientation3DReadOnly, FrameTuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param orientation    the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference.
    */
   default void setMatchingFrame(FrameOrientation3DReadOnly orientation, FrameTuple3DReadOnly translation)
   {
      orientation.checkReferenceFrameMatch(translation);
      setMatchingFrame(orientation.getReferenceFrame(), orientation, translation);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(RotationMatrixReadOnly, Tuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param rotationMatrix the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the rotation and translation parts of this pose.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameRotationMatrixReadOnly, FrameTuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param rotationMatrix the orientation used to set the rotation part of this pose. Not modified.
    * @param translation    the tuple used to set the translation part of this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference.
    */
   default void setMatchingFrame(FrameRotationMatrixReadOnly rotationMatrix, FrameTuple3DReadOnly translation)
   {
      rotationMatrix.checkReferenceFrameMatch(translation);
      setMatchingFrame(rotationMatrix.getReferenceFrame(), rotationMatrix, translation);
   }

   /**
    * Appends a translation transform to this pose.
    *
    * <pre>
    *               / 1 0 0 translation.x \
    * this = this * | 0 1 0 translation.y |
    *               | 0 0 1 translation.z |
    *               \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this pose.
    * </p>
    *
    * @param translation the translation to append to this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void appendTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.appendTranslation(translation);
   }

   /**
    * Appends the orientation to this pose.
    *
    * @param orientation the orientation to append. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void appendOrientation(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      getRotation().append(orientation);
   }

   /**
    * Prepends a translation transform to this pose.
    *
    * <pre>
    *        / 1 0 0 translation.x \
    * this = | 0 1 0 translation.y | * this
    *        | 0 0 1 translation.z |
    *        \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this pose.
    * </p>
    *
    * @param translation the translation to prepend to this pose. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void prependTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Shape3DPoseBasics.super.prependTranslation(translation);
   }

   /**
    * Prepends the orientation to this pose.
    *
    * @param orientation the orientation to prepend. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void prependOrientation(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Shape3DPoseBasics.super.prependOrientation(orientation);
   }

   /**
    * Performs the multiplication of this pose with {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiply(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiply(other);
   }

   /**
    * Performs the multiplication of the inverse of this pose with {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyInvertThis(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiplyInvertThis(other);
   }

   /**
    * Performs the multiplication of this pose with the inverse of {@code other}.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyInvertOther(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.multiplyInvertOther(other);
   }

   /**
    * Performs the multiplication of {@code other} with this pose.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiply(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiply(other);
   }

   /**
    * Performs the multiplication of {@code other} with the inverse of this pose.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyInvertThis(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiplyInvertThis(other);
   }

   /**
    * Performs the multiplication of the inverse of {@code other} with this pose.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other pose to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyInvertOther(FrameShape3DPoseReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Shape3DPoseBasics.super.preMultiplyInvertOther(other);
   }
}

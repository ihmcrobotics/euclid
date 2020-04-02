package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;

/**
 * Read-only interface for representing the pose of a shape 3D expressed in a given reference frame.
 * <p>
 * While the main use-case of a {@code FrameShape3DPoseReadOnly} is to describe the pose of a shape
 * 3D, it is also used to represent the transform from the shape local coordinate system to the
 * reference frame coordinates, such that it can be used to transform geometry back and forth
 * between the two coordinate systems.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameShape3DPoseReadOnly extends Shape3DPoseReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   @Override
   FrameRotationMatrixReadOnly getShapeOrientation();

   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getShapePosition();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getXAxis();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getYAxis();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getZAxis();

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param orientationToPack the orientation to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void get(FixedFrameOrientation3DBasics orientationToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(orientationToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(orientationToPack, translationToPack);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param orientationToPack the orientation to set to the rotation of this transform. Modified.
    * @param translationToPack the tuple to set to the translation of this transform. Modified.
    */
   default void get(FrameOrientation3DBasics orientationToPack, FrameTuple3DBasics translationToPack)
   {
      orientationToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(orientationToPack, translationToPack);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void get(FixedFrameRotationMatrixBasics rotationMatrixToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(rotationMatrixToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(rotationMatrixToPack, translationToPack);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    */
   default void get(FrameRotationMatrixBasics rotationMatrixToPack, FrameTuple3DBasics translationToPack)
   {
      rotationMatrixToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(rotationMatrixToPack, translationToPack);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector to set to the rotation of this transform.
    *                             Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void get(FixedFrameVector3DBasics rotationVectorToPack, FixedFrameTuple3DBasics translationToPack)
   {
      checkReferenceFrameMatch(rotationVectorToPack, translationToPack);
      Shape3DPoseReadOnly.super.get(rotationVectorToPack, translationToPack);
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector to set to the rotation of this transform.
    *                             Modified.
    * @param translationToPack    the tuple to set to the translation of this transform. Modified.
    */
   default void get(FrameVector3DBasics rotationVectorToPack, FrameTuple3DBasics translationToPack)
   {
      rotationVectorToPack.setReferenceFrame(getReferenceFrame());
      translationToPack.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.get(rotationVectorToPack, translationToPack);
   }

   /**
    * Transforms the given {@code pointToTransform} by this pose.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFramePoint3DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform);
   }

   /**
    * Transforms the given {@code pointOriginal} by this pose and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FramePoint3DReadOnly pointOriginal, FixedFramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   /**
    * Transforms the given {@code pointOriginal} by this pose and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code pointOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FramePoint3DReadOnly pointOriginal, FramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   /**
    * Transforms the given {@code vectorToTransform} by this pose.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameVector3DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this pose and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector3DReadOnly vectorOriginal, FixedFrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this pose and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code vectorOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector3DReadOnly vectorOriginal, FrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code orientationToTransform} by this pose.
    * <p>
    * If the given orientation is expressed in the local frame described by this pose, then the
    * orientation is transformed such that it is, after this method is called, expressed in the base
    * frame in which this pose is expressed.
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      Shape3DPoseReadOnly.super.transform(orientationToTransform);
   }

   /**
    * Transforms the given {@code orientationOriginal} by this pose and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * If the given orientation is expressed in the local frame described by this pose, then the
    * orientation is transformed such that it is, after this method is called, expressed in the base
    * frame in which this pose is expressed.
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      Shape3DPoseReadOnly.super.transform(orientationOriginal, orientationTransformed);
   }

   /**
    * Transforms the given {@code orientationOriginal} by this pose and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * If the given orientation is expressed in the local frame described by this pose, then the
    * orientation is transformed such that it is, after this method is called, expressed in the base
    * frame in which this pose is expressed.
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code orientationOriginal} is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void transform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(orientationOriginal, orientationTransformed);
   }

   /**
    * Transforms the given {@code vectorToTransform} by this pose.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this pose and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this pose and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given vector is expressed in the local frame described by this pose, then the vector is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code vectorOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code pointToTransform} by this transform.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFramePoint2DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform);
   }

   /**
    * Transforms the given {@code pointOriginal} by this transform and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   /**
    * Transforms the given {@code pointOriginal} by this transform and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed);
   }

   /**
    * Transforms the given {@code pointToTransform} by this transform.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointToTransform          the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and the
    *                                         rotation part of this transform is not a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFramePoint2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.transform(pointToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code pointOriginal} by this transform and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal             the point to transform. Not modified.
    * @param pointTransformed          the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and the
    *                                         rotation part of this transform is not a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code pointOriginal} by this transform and stores the result in
    * {@code pointTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param pointOriginal             the point to transform. Not modified.
    * @param pointTransformed          the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *                               of this transform is not a transformation in the XY plane.
    */
   default void transform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code vectorToTransform} by this transform.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameVector2DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this transform and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this transform and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation in
    *                               the XY plane.
    */
   default void transform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given {@code vectorToTransform} by this transform.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorToTransform         the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FixedFrameVector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.transform(vectorToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this transform and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal            the vector to transform. Not modified.
    * @param vectorTransformed         the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code vectorOriginal} by this transform and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * If the given point is expressed in the local frame described by this pose, then the point is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param vectorOriginal            the vector to transform. Not modified.
    * @param vectorTransformed         the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation in
    *                               the XY plane.
    */
   default void transform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code matrixToTransform} by this pose.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.transform(matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this pose and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this pose and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given {@code matrixToTransform} by this pose.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.transform(matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this pose and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given {@code matrixOriginal} by this pose and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this pose, then the matrix is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given {@code poseToTransform} by this pose.
    * <p>
    * If the given pose is expressed in the local frame described by this pose, then the pose is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param poseToTransform the pose to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameShape3DPoseBasics poseToTransform)
   {
      checkReferenceFrameMatch(poseToTransform);
      Shape3DPoseReadOnly.super.transform(poseToTransform);
   }

   /**
    * Transforms the given {@code poseOriginal} by this pose and stores the result in
    * {@code poseTransformed}.
    * <p>
    * If the given pose is expressed in the local frame described by this pose, then the pose is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param poseOriginal    the pose to transform. Not modified.
    * @param poseTransformed the pose in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameShape3DPoseReadOnly poseOriginal, FixedFrameShape3DPoseBasics poseTransformed)
   {
      checkReferenceFrameMatch(poseOriginal, poseTransformed);
      Shape3DPoseReadOnly.super.transform(poseOriginal, poseTransformed);
   }

   /**
    * Transforms the given {@code poseOriginal} by this pose and stores the result in
    * {@code poseTransformed}.
    * <p>
    * If the given pose is expressed in the local frame described by this pose, then the pose is
    * transformed such that it is, after this method is called, expressed in the base frame in which
    * this pose is expressed.
    * </p>
    *
    * @param poseOriginal    the pose to transform. Not modified.
    * @param poseTransformed the pose in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code poseOriginal} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void transform(FrameShape3DPoseReadOnly poseOriginal, FrameShape3DPoseBasics poseTransformed)
   {
      checkReferenceFrameMatch(poseOriginal);
      poseTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.transform(poseOriginal, poseTransformed);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFramePoint3DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFramePoint3DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FramePoint3DReadOnly, FixedFramePoint3DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FramePoint3DReadOnly pointOriginal, FixedFramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FramePoint3DReadOnly, FramePoint3DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    */
   default void inverseTransform(FramePoint3DReadOnly pointOriginal, FramePoint3DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameVector3DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameVector3DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector3DReadOnly, FixedFrameVector3DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector3DReadOnly vectorOriginal, FixedFrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector3DReadOnly, FrameVector3DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   default void inverseTransform(FrameVector3DReadOnly vectorOriginal, FrameVector3DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given orientation {@code orientationToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameOrientation3DBasics)} with the inverse
    * of this transform.
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(orientationToTransform);
   }

   /**
    * Performs the inverse of the transform on the given orientation {@code orientationOriginal} and
    * stores the result in {@code orientationTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameOrientation3DReadOnly, FixedFrameOrientation3DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform on the given orientation {@code orientationOriginal} and
    * stores the result in {@code orientationTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameOrientation3DReadOnly, FrameOrientation3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    */
   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameVector4DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vectorToTransform the 4D vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector4DReadOnly, FixedFrameVector4DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector4DReadOnly, FrameVector4DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFramePoint2DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFramePoint2DBasics pointToTransform)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FramePoint2DReadOnly, FixedFramePoint2DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FramePoint2DReadOnly, FramePoint2DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param pointOriginal    the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation in
    *                               the XY plane.
    */
   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFramePoint2DBasics, boolean)} with the
    * inverse of this transform.
    * </p>
    *
    * @param pointToTransform          the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and the
    *                                         rotation part of this transform is not a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFramePoint2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(pointToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FramePoint2DReadOnly, FixedFramePoint2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param pointOriginal             the point to transform. Not modified.
    * @param pointTransformed          the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and the
    *                                         rotation part of this transform is not a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FixedFramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal, pointTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FramePoint2DReadOnly, FramePoint2DBasics, boolean)} with the inverse of this
    * transform.
    * </p>
    *
    * @param pointOriginal             the point to transform. Not modified.
    * @param pointTransformed          the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *                               of this transform is not a transformation in the XY plane.
    */
   default void inverseTransform(FramePoint2DReadOnly pointOriginal, FramePoint2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(pointOriginal);
      pointTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameVector2DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameVector2DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector2DReadOnly, FixedFrameVector2DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameVector2DReadOnly, FrameVector2DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation in
    *                               the XY plane.
    */
   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameVector2DBasics, boolean)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vectorToTransform         the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameVector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(vectorToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameVector2DReadOnly, FixedFrameVector2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vectorOriginal            the vector to transform. Not modified.
    * @param vectorTransformed         the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if the rotation part of this transform is not a
    *                                         transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FixedFrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given vector {@code vectorOriginal} and stores the
    * result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameVector2DReadOnly, FrameVector2DBasics, boolean)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vectorOriginal            the vector to transform. Not modified.
    * @param vectorTransformed         the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of this
    *                                  transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation in
    *                               the XY plane.
    */
   default void inverseTransform(FrameVector2DReadOnly vectorOriginal, FrameVector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameMatrix3DBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(matrixToTransform);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameMatrix3DReadOnly, FixedFrameMatrix3DBasics)}
    * with the inverse of this transform.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(FrameMatrix3DReadOnly, FrameMatrix3DBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform on the given rotation matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameRotationMatrixBasics)} with the inverse
    * of this transform.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(matrixToTransform);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameRotationMatrixReadOnly, FixedFrameRotationMatrixBasics)} with the inverse
    * of this transform.
    * </p>
    *
    * @param matrixOriginal    the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform on the given matrix {@code matrixOriginal} and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameRotationMatrixReadOnly, FrameRotationMatrixBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param matrixOriginal    the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform on the given {@code poseToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(FixedFrameShape3DPoseBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param poseToTransform the pose to transform. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameShape3DPoseBasics poseToTransform)
   {
      checkReferenceFrameMatch(poseToTransform);
      Shape3DPoseReadOnly.super.inverseTransform(poseToTransform);
   }

   /**
    * Performs the inverse of the transform on the given {@code poseOriginal} and stores the result in
    * {@code poseTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameShape3DPoseReadOnly, FixedFrameShape3DPoseBasics)} with the inverse of
    * this transform.
    * </p>
    *
    * @param poseOriginal    the pose to transform. Not modified.
    * @param poseTransformed the pose in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void inverseTransform(FrameShape3DPoseReadOnly poseOriginal, FixedFrameShape3DPoseBasics poseTransformed)
   {
      checkReferenceFrameMatch(poseOriginal, poseTransformed);
      Shape3DPoseReadOnly.super.inverseTransform(poseOriginal, poseTransformed);
   }

   /**
    * Performs the inverse of the transform on the given {@code poseOriginal} and stores the result in
    * {@code poseTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(FrameShape3DPoseReadOnly, FrameShape3DPoseBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param poseOriginal    the pose to transform. Not modified.
    * @param poseTransformed the pose in which the result is stored. Modified.
    */
   default void inverseTransform(FrameShape3DPoseReadOnly poseOriginal, FrameShape3DPoseBasics poseTransformed)
   {
      checkReferenceFrameMatch(poseOriginal);
      poseTransformed.setReferenceFrame(getReferenceFrame());
      Shape3DPoseReadOnly.super.inverseTransform(poseOriginal, poseTransformed);
   }

   /**
    * Tests on a per-component basis if this shape pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two shape poses are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Shape3DPoseReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two shape poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param other   the shape pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two shape poses represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default boolean geometricallyEquals(FrameShape3DPoseReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Shape3DPoseReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this shape pose 3D is exactly equal to {@code other}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other shape pose 3D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameShape3DPoseReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }
}

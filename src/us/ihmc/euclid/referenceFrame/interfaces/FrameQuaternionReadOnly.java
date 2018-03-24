package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for a quaternion expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link QuaternionReadOnly}, a {@link ReferenceFrame} is associated
 * to a {@code FrameQuaternionReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on quaternions occur in the same coordinate system. Also, via the method
 * {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value of a quaternion in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameQuaternionReadOnly} extends {@code QuaternionReadOnly}, it is compatible
 * with methods only requiring {@code QuaternionReadOnly}. However, these methods do NOT assert that
 * the operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameQuaternionReadOnly}.
 * </p>
 */
public interface FrameQuaternionReadOnly extends FrameTuple4DReadOnly, QuaternionReadOnly
{
   /**
    * Computes and returns the distance from this quaternion to {@code other}.
    *
    * @param other the other quaternion to measure the distance. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in [0,
    *         2<i>pi</i>]
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code other} do
    *            not match.
    */
   default double distance(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.distance(other);
   }

   /**
    * Computes and returns the distance from this quaternion to {@code other}.
    * <p>
    * This method is equivalent to {@link #distance(QuaternionReadOnly)} but is more accurate when
    * computing the distance between two quaternions that are very close. Note that it is also more
    * expensive.
    * </p>
    *
    * @param other the other quaternion to measure the distance. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in [0,
    *         2<i>pi</i>]
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code other} do
    *            not match.
    */
   default double distancePrecise(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.distancePrecise(other);
   }

   /**
    * Packs this quaternion in the given {@code quaternionToPack}.
    * 
    * @param quaternionToPack the quaternion in which this quaternion is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code quaternionToPack} do
    *            not match.
    */
   default void get(FixedFrameQuaternionBasics quaternionToPack)
   {
      checkReferenceFrameMatch(quaternionToPack);
      quaternionToPack.set(this);
   }

   /**
    * Packs this quaternion in the given {@code quaternionToPack}.
    * 
    * @param quaternionToPack the quaternion in which this quaternion is stored. Modified.
    */
   default void get(FrameQuaternionBasics quaternionToPack)
   {
      quaternionToPack.setIncludingFrame(this);
   }

   /**
    * Computes and packs the orientation described by this quaternion as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code rotationVectorToPack} do not match.
    */
   default void getRotationVector(FixedFrameVector3DBasics rotationVectorToPack)
   {
      checkReferenceFrameMatch(rotationVectorToPack);
      QuaternionReadOnly.super.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this quaternion as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   default void getRotationVector(FrameVector3DBasics rotationVectorToPack)
   {
      rotationVectorToPack.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this quaternion as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code eulerAnglesToPack} do not match.
    */
   default void getEuler(FixedFrameTuple3DBasics eulerAnglesToPack)
   {
      checkReferenceFrameMatch(eulerAnglesToPack);
      QuaternionReadOnly.super.getEuler(eulerAnglesToPack);
   }

   /**
    * Computes and packs the orientation described by this quaternion as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   default void getEuler(FrameTuple3DBasics eulerAnglesToPack)
   {
      eulerAnglesToPack.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.getEuler(eulerAnglesToPack);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this orientation and adds the result to the tuple.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.addTransform(tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      QuaternionReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      QuaternionReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix does
    *            not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, {@code tupleTransformed} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix does
    *            not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, {@code tupleTransformed} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given quaternion by this quaternion.
    * <p>
    * quaternionToTransform = this * quaternionToTransform <br>
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionToTransform} do not match.
    */
   default void transform(FixedFrameQuaternionBasics quaternionToTransform)
   {
      checkReferenceFrameMatch(quaternionToTransform);
      QuaternionReadOnly.super.transform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void transform(FrameQuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      QuaternionReadOnly.super.transform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionTransformed} do not match.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, FixedFrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionTransformed);
      QuaternionReadOnly.super.transform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void transform(QuaternionReadOnly quaternionOriginal, FrameQuaternionBasics quaternionTransformed)
   {
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code quaternionOriginal}, and {@code quaternionTransformed} do not match.
    */
   default void transform(FrameQuaternionReadOnly quaternionOriginal, FixedFrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      checkReferenceFrameMatch(quaternionTransformed);
      QuaternionReadOnly.super.transform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the given quaternion {@code quaternionOriginal} and stores the result into
    * {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void transform(FrameQuaternionReadOnly quaternionOriginal, FrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this * vectorToTransform.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorToTransform} do not match.
    */
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      QuaternionReadOnly.super.transform(vectorToTransform);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorOriginal} do not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      QuaternionReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorTransformed} do not match.
    */
   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      QuaternionReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   default void transform(Vector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      vectorTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code vectorOriginal}, and {@code vectorTransformed} do not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      QuaternionReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorOriginal} do not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this quaternion
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this quaternion
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this quaternion
    * and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code tupleOriginal}, and {@code tupleTransformed} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      checkReferenceFrameMatch(tupleTransformed);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionToTransform}.
    * <p>
    * quaternionToTransform = this<sup>-1</sup> * quaternionToTransform * this
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Not modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionToTransform} do not match.
    */
   default void inverseTransform(FixedFrameQuaternionBasics quaternionToTransform)
   {
      checkReferenceFrameMatch(quaternionToTransform);
      QuaternionReadOnly.super.inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by this
    * quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void inverseTransform(FrameQuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by this
    * quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionTransformed} do not match.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, FixedFrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionTransformed);
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by this
    * quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, FrameQuaternionBasics quaternionTransformed)
   {
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by this
    * quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code quaternionOriginal}, and {@code quaternionTransformed} do not match.
    */
   default void inverseTransform(FrameQuaternionReadOnly quaternionOriginal, FixedFrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      checkReferenceFrameMatch(quaternionTransformed);
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by this
    * quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void inverseTransform(FrameQuaternionReadOnly quaternionOriginal, FrameQuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector by this quaternion.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this<sup>-1</sup> * vectorToTransform.xyz * this
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorToTransform} do not match.
    */
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      QuaternionReadOnly.super.inverseTransform(vectorToTransform);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this quaternion and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorOriginal} do not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      QuaternionReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this quaternion and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorTransformed} do not match.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      QuaternionReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this quaternion and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      vectorTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this quaternion and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *            {@code vectorOriginal}, and {@code vectorTransformed} do not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      QuaternionReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this quaternion and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz * this
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code vectorOriginal} do not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame quaternions are
    * geometrically similar, i.e. the magnitude of their difference is less than or equal to
    * {@code epsilon}.
    *
    * @param other the frame quaternion to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two frame quaternions represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default boolean geometricallyEquals(FrameQuaternionReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

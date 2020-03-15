package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * Read-only interface for any implementation of an orientation 3D expressed in a given reference
 * frame.
 * <p>
 * Even though the representation used is unknown at this level of abstraction, this interface
 * allows to enforce a minimum set of features that all representations of an orientation should
 * provide, such as transformation functions.
 * </p>
 * <p>
 * Because a {@code FrameOrientation3DReadOnly} extends {@code Orientation3DReadOnly}, it is
 * compatible with methods only requiring {@code Orientation3DReadOnly}. However, these methods do
 * NOT assert that the operation occur in the proper coordinate system. Use this feature carefully
 * and always prefer using methods requiring {@code FrameOrientation3DReadOnly}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameOrientation3DReadOnly extends Orientation3DReadOnly, ReferenceFrameHolder
{
   /**
    * Converts, if necessary, and packs this orientation in a quaternion.
    *
    * @param quaternionToPack the quaternion into which this orientation is to be stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code frameQuaternionToPack} is not expressed in the
    *                                         same frame as this.
    */
   default void get(FixedFrameQuaternionBasics quaternionToPack)
   {
      checkReferenceFrameMatch(quaternionToPack);
      get((QuaternionBasics) quaternionToPack);
   }

   /**
    * Converts, if necessary, and packs this orientation in a quaternion.
    *
    * @param quaternionToPack the quaternion into which this orientation is to be stored. Modified.
    */
   default void get(FrameQuaternionBasics quaternionToPack)
   {
      quaternionToPack.setReferenceFrame(getReferenceFrame());
      get((QuaternionBasics) quaternionToPack);
   }

   /**
    * Converts, if necessary, and packs this orientation in a yaw-pitch-roll.
    *
    * @param yawPitchRollToPack the yaw-pitch-roll into which this orientation is to be stored.
    *                           Modified.
    * @throws ReferenceFrameMismatchException if {@code frameYawPitchRollToPack} is not expressed in
    *                                         the same frame as this.
    */
   default void get(FixedFrameYawPitchRollBasics yawPitchRollToPack)
   {
      checkReferenceFrameMatch(yawPitchRollToPack);
      get((YawPitchRollBasics) yawPitchRollToPack);
   }

   /**
    * Converts, if necessary, and packs this orientation in a yaw-pitch-roll.
    *
    * @param yawPitchRollToPack the yaw-pitch-roll into which this orientation is to be stored.
    *                           Modified.
    */
   default void get(FrameYawPitchRollBasics yawPitchRollToPack)
   {
      yawPitchRollToPack.setReferenceFrame(getReferenceFrame());
      get((YawPitchRollBasics) yawPitchRollToPack);
   }

   default void get(FrameRotationMatrixBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.setReferenceFrame(getReferenceFrame());
      get((RotationMatrixBasics) rotationMatrixToPack);
   }

   default void get(FixedFrameRotationMatrixBasics rotationMatrixToPack)
   {
      checkReferenceFrameMatch(rotationMatrixToPack);
      get((RotationMatrixBasics) rotationMatrixToPack);
   }

   /**
    * Converts and packs this orientation in a 3D rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which this orientation is to be stored.
    *                             Modified.
    * @throws ReferenceFrameMismatchException if {@code rotationVectorToPack} is not expressed in the
    *                                         same frame as this.
    */
   default void getRotationVector(FixedFrameVector3DBasics rotationVectorToPack)
   {
      checkReferenceFrameMatch(rotationVectorToPack);
      getRotationVector((Vector3DBasics) rotationVectorToPack);
   }

   /**
    * Converts and packs this orientation in a 3D rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which this orientation is to be stored.
    *                             Modified.
    */
   default void getRotationVector(FrameVector3DBasics rotationVectorToPack)
   {
      rotationVectorToPack.setReferenceFrame(getReferenceFrame());
      getRotationVector((Vector3DBasics) rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this orientation as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code frameEulerAnglesToPack} is not expressed in the
    *                                         same frame as this.
    */
   default void getEuler(FixedFrameTuple3DBasics eulerAnglesToPack)
   {
      checkReferenceFrameMatch(eulerAnglesToPack);
      getEuler((Tuple3DBasics) eulerAnglesToPack);
   }

   /**
    * Computes and packs the orientation described by this orientation as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   default void getEuler(FrameTuple3DBasics eulerAnglesToPack)
   {
      eulerAnglesToPack.setReferenceFrame(getReferenceFrame());
      getEuler((Tuple3DBasics) eulerAnglesToPack);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      Orientation3DReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      transform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      transform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      transform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
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
    *                                         {@code tupleTransformed} do not match.
    */
   default void addTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      addTransform((Tuple3DBasics) tupleToTransform);
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
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      addTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
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
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      addTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
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
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      addTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
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
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      addTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
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
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void addTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      addTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple by this orientation and subtracts the result to the tuple.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void subTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      subTransform((Tuple3DBasics) tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void subTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      subTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void subTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      subTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   default void subTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      subTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void subTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      subTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void subTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      subTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      transform((Tuple2DBasics) tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this orientation does not represent a transformation in the XY
    *                               plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, {@code tupleTransformed} do not
    *                                         match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      transform((Tuple2DBasics) tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         matrix does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, {@code tupleTransformed} do not
    *                                         match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      transform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given matrix by this orientation.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixToTransform the 3D matrix to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixToTransform} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void transform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      transform((Matrix3DBasics) matrixToTransform);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   default void transform(Matrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixTransformed} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void transform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if either {@code matrixOriginal} or
    *                                         {@code matrixTransformed} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void transform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      transform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * 4D vector, its scalar component {@code s} remains unaffected by this operation.
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorToTransform the 4D tuple to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorToTransform} do not match.
    */
   default void transform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      transform((Vector4DBasics) vectorToTransform);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorOriginal} do not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      transform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorTransformed} do not match.
    */
   default void transform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      transform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    */
   default void transform(Vector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      transform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code vectorOriginal}, and {@code vectorTransformed} do
    *                                         not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      transform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorOriginal} do not match.
    */
   default void transform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      transform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Transforms the given {@code orientationToTransform} by this orientation.
    * <p>
    * The operation is equivalent to prepend this orientation to the given
    * {@code orientationToTransform}.
    * </p>
    *
    * @param orientationToTransform the orientation to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationToTransform} do not match.
    */
   default void transform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      transform((Orientation3DBasics) orientationToTransform);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationOriginal} do not match.
    */
   default void transform(FrameOrientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      transform((Orientation3DReadOnly) orientationOriginal, (Orientation3DBasics) orientationTransformed);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationTransformed} do not match.
    */
   default void transform(Orientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationTransformed);
      transform((Orientation3DReadOnly) orientationOriginal, (Orientation3DBasics) orientationTransformed);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void transform(Orientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      transform((Orientation3DReadOnly) orientationOriginal, (Orientation3DBasics) orientationTransformed);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code orientationOriginal}, and
    *                                         {@code orientationTransformed} do not match.
    */
   default void transform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      transform((Orientation3DReadOnly) orientationOriginal, (Orientation3DBasics) orientationTransformed);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationOriginal} do not match.
    */
   default void transform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      transform((Orientation3DReadOnly) orientationOriginal, (Orientation3DBasics) orientationTransformed);
   }

   default void transform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      transform((RotationMatrixBasics) matrixToTransform);
   }

   default void transform(FrameRotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      transform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void transform(RotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      transform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void transform(RotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      transform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      transform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void transform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      transform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      inverseTransform((Tuple3DBasics) tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * orientation and stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * orientation and stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * orientation and stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple3DReadOnly) tupleOriginal, (Tuple3DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      inverseTransform((Tuple2DBasics) tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException           if this orientation does not represent a transformation
    *                                         in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this orientation does not represent a transformation in the XY
    *                               plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      inverseTransform((Tuple2DBasics) tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this orientation
    *                               does not represent a transformation in the XY plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this orientation
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException           if {@code checkIfTransformInXYPlane == true} and this
    *                                         orientation does not represent a transformation in the XY
    *                                         plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      inverseTransform((Tuple2DReadOnly) tupleOriginal, (Tuple2DBasics) tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given matrix by this orientation.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixToTransform the 3D matrix to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixToTransform} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FixedFrameMatrix3DBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      inverseTransform((Matrix3DBasics) matrixToTransform);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixTransformed} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixTransformed} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if either {@code matrixOriginal} or
    *                                         {@code matrixTransformed} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FixedFrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    * @throws ReferenceFrameMismatchException if {@code matrixOriginal} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void inverseTransform(FrameMatrix3DReadOnly matrixOriginal, FrameMatrix3DBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Matrix3DReadOnly) matrixOriginal, (Matrix3DBasics) matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given 4D vector, its scalar component {@code s} remains unaffected
    * by this operation.
    * <p>
    * If the given vector part is expressed in the base frame in which this orientation is expressed,
    * then the vector part is transformed such that it is, after this method is called, expressed in
    * the local frame described by this orientation.
    * </p>
    *
    * @param vectorToTransform the 4D tuple to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorToTransform} do not match.
    */
   default void inverseTransform(FixedFrameVector4DBasics vectorToTransform)
   {
      checkReferenceFrameMatch(vectorToTransform);
      inverseTransform((Vector4DBasics) vectorToTransform);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorOriginal} do not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorTransformed} do not match.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorTransformed);
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code vectorOriginal}, and {@code vectorTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FixedFrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal, vectorTransformed);
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code vectorOriginal} do not match.
    */
   default void inverseTransform(FrameVector4DReadOnly vectorOriginal, FrameVector4DBasics vectorTransformed)
   {
      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((Vector4DReadOnly) vectorOriginal, (Vector4DBasics) vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the given orientation by this orientation.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the given orientation.
    * </p>
    *
    * @param orientationToTransform the orientation to be transformed. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationToTransform} do not match.
    */
   default void inverseTransform(FixedFrameOrientation3DBasics orientationToTransform)
   {
      checkReferenceFrameMatch(orientationToTransform);
      Orientation3DReadOnly.super.inverseTransform(orientationToTransform, orientationToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationOriginal} do not match.
    */
   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      Orientation3DReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationTransformed} do not match.
    */
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationTransformed);
      Orientation3DReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Orientation3DReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code orientationOriginal}, and
    *                                         {@code orientationTransformed} do not match.
    */
   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FixedFrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal, orientationTransformed);
      Orientation3DReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code orientationOriginal} do not match.
    */
   default void inverseTransform(FrameOrientation3DReadOnly orientationOriginal, FrameOrientation3DBasics orientationTransformed)
   {
      checkReferenceFrameMatch(orientationOriginal);
      orientationTransformed.setReferenceFrame(getReferenceFrame());
      Orientation3DReadOnly.super.inverseTransform(orientationOriginal, orientationTransformed);
   }

   default void inverseTransform(FixedFrameRotationMatrixBasics matrixToTransform)
   {
      checkReferenceFrameMatch(matrixToTransform);
      inverseTransform((RotationMatrixBasics) matrixToTransform);
   }

   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      inverseTransform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixTransformed);
      inverseTransform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FixedFrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal, matrixTransformed);
      inverseTransform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }

   default void inverseTransform(FrameRotationMatrixReadOnly matrixOriginal, FrameRotationMatrixBasics matrixTransformed)
   {
      checkReferenceFrameMatch(matrixOriginal);
      matrixTransformed.setReferenceFrame(getReferenceFrame());
      inverseTransform((RotationMatrixReadOnly) matrixOriginal, (RotationMatrixBasics) matrixTransformed);
   }
}

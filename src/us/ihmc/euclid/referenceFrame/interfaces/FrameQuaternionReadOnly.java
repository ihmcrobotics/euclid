package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public interface FrameQuaternionReadOnly extends FrameTuple4DReadOnly, QuaternionReadOnly
{
   public static final double EPS_UNITARY = 1.0e-7;

   /** {@inheritDoc} */
   default boolean isUnitary(double epsilon)
   {
      return Math.abs(norm() - 1.0) < epsilon;
   }

   /**
    * Tests if this quaternion represents a rotation around the z-axis.
    * <p>
    * This is commonly used to test if the quaternion can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this quaternion represents a rotation around the z-axis, {@code false}
    *         otherwise.
    */
   default boolean isZOnly(double epsilon)
   {
      return Math.abs(getX()) < epsilon && Math.abs(getY()) < epsilon;
   }

   /**
    * Asserts that this quaternion has a norm equal to 1+/-{@value #EPS_UNITARY}.
    *
    * @param epsilon the tolerance to use.
    * @throws RuntimeException if this quaternion is not a proper unit-quaternion.
    */
   default void checkIfUnitary()
   {
      checkIfUnitary(EPS_UNITARY);
   }

   /**
    * Asserts that this quaternion has a norm equal to 1+/-{@code epsilon}.
    *
    * @param epsilon the tolerance to use.
    * @throws RuntimeException if this quaternion is not a proper unit-quaternion.
    */
   default void checkIfUnitary(double epsilon)
   {
      if (!isUnitary(epsilon))
         throw new RuntimeException("This quaternion is not a unit-quaternion.");
   }

   /**
    * Asserts that this quaternion represents a rotation around the z-axis.
    * <p>
    * This is commonly used to test if the quaternion can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @throws NotAMatrix2DException if this quaternion does not represent a rotation around the
    *            z-axis.
    */
   default void checkIfIsZOnly(double epsilon)
   {
      if (!isZOnly(epsilon))
         throw new NotAMatrix2DException("The quaternion is not in XY plane: " + toString());
   }

   /**
    * Efficiently compute the norm of this quaternion.
    */
   @Override
   default double norm()
   {
      return EuclidCoreTools.fastSquareRoot(normSquared());
   }

   /**
    * Computes and returns the distance from this quaternion to {@code other}.
    *
    * @param other the other quaternion to measure the distance. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in
    *         [0, 2<i>pi</i>]
    */
   default double distance(QuaternionReadOnly other)
   {
      return 2.0 * Math.acos(dot(other));
   }

   /**
    * Calculates and returns the angle of the rotation this quaternion represents.
    *
    * @return the angle &in; [-2<i>pi</i>;2<i>pi</i>].
    */
   default double getAngle()
   {
      double sinHalfTheta = Math.sqrt(EuclidCoreTools.normSquared(getX(), getY(), getZ()));
      return 2.0 * Math.atan2(sinHalfTheta, getS());
   }

   /**
    * Computes and packs the orientation described by this quaternion as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   default void get(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertQuaternionToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this quaternion as the yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   default void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(this, yawPitchRollToPack);
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
   default void getEuler(Vector3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(this, eulerAnglesToPack);
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this quaternion.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   default double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this
    * quaternion.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   default double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this quaternion.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   default double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
   }

   /**
    * Transforms the given tuple by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
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
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
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
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
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
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix
    *            does not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given 3D matrix by this quaternion.
    * <p>
    * matrixToTransform = this * matrixToTransform * this<sup>-1</sup>
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this quaternion and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal * this<sup>-1</sup>
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the given quaternion by this quaternion.
    * <p>
    * quaternionToTransform = this * quaternionToTransform <br>
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
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
   default void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this * vectorToTransform.xyz * this<sup>-1</sup>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
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
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      QuaternionTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix by this quaternion.
    * <p>
    * matrixToTransform = this * matrixToTransform
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by this quaternion and stores the
    * result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * quaternion and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this quaternion.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this quaternion.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * quaternion and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this quaternion.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * quaternion and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this quaternion
    *            does not represent a transformation in the XY plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionToTransform}.
    * <p>
    * quaternionToTransform = this<sup>-1</sup> * quaternionToTransform <br>
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void inverseTransform(QuaternionBasics quaternionToTransform)
   {
      inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} and
    * stores the result into {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal <br>
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.inverseTransform(this, quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector by this
    * quaternion.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this<sup>-1</sup> * vectorToTransform.xyz * this
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
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
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      QuaternionTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * quaternion.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform * this
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Not modified.
    */
   default void inverseTransform(Matrix3D matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * quaternion and stores the result in {@code matrixTransformed}.
    * <p>
    * s matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      QuaternionTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix {@code matrixToTransform}
    * by this quaternion.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix {@code matrixOriginal} by
    * this quaternion and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      QuaternionTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Provides a {@code String} representation of this quaternion converted to yaw-pitch-roll angles
    * as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return
    */
   default String toStringAsYawPitchRoll()
   {
      return EuclidCoreIOTools.getStringOf("yaw-pitch-roll: (", ")", ", ", getYaw(), getPitch(), getRoll());
   }

}

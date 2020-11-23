package us.ihmc.euclid.transform.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface LinearTransform3DBasics extends LinearTransform3DReadOnly, Matrix3DBasics
{
   @Override
   default void setToZero()
   {
      setIdentity();
   }

   void resetScale();

   default void setRotationVector(Vector3DReadOnly rotationVector)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, this);
   }

   default void setEuler(Tuple3DReadOnly eulerAngles)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), this);
   }

   default void appendRotation(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(this, false, false, orientation, false, this);
   }

   default void appendRotationInvertThis(Orientation3DReadOnly orientation)
   {
      if (isRotationMatrix())
         Matrix3DTools.multiply(this, true, false, orientation, false, this);
      else
         Matrix3DTools.multiply(this, false, true, orientation, false, this);
   }

   default void appendRotationInvertOther(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(this, false, false, orientation, true, this);
   }

   /**
    * Append a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *               / cos(yaw) -sin(yaw) 0 \
    * this = this * | sin(yaw)  cos(yaw) 0 |
    *               \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      RotationMatrixTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Append a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *               /  cos(pitch) 0 sin(pitch) \
    * this = this * |      0      1     0      |
    *               \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      RotationMatrixTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *               / 1     0          0     \
    * this = this * | 0 cos(roll) -sin(roll) |
    *               \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void appendRollRotation(double roll)
   {
      RotationMatrixTools.appendRollRotation(this, roll, this);
   }

   default void appendScale(double scale)
   {
      appendScale(scale, scale, scale);
   }

   default void appendScale(Tuple3DReadOnly scale)
   {
      appendScale(scale.getX(), scale.getY(), scale.getZ());
   }

   default void appendScale(double x, double y, double z)
   {
      scaleColumns(x, y, z);
   }

   default void prependRotation(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(orientation, false, this, false, false, this);
   }

   default void prependRotationInvertThis(Orientation3DReadOnly orientation)
   {
      if (isRotationMatrix())
         Matrix3DTools.multiply(orientation, false, this, true, false, this);
      else
         Matrix3DTools.multiply(orientation, false, this, false, true, this);
   }

   default void prependRotationInvertOther(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(orientation, true, this, false, false, this);
   }

   /**
    * Prepend a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 | * this
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepend a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      | * this
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) | * this
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.prependRollRotation(roll, this, this);
   }

   default void prependScale(double scale)
   {
      prependScale(scale, scale, scale);
   }

   default void prependScale(Tuple3DReadOnly scale)
   {
      prependScale(scale.getX(), scale.getY(), scale.getZ());
   }

   default void prependScale(double x, double y, double z)
   {
      scaleRows(x, y, z);
   }
}
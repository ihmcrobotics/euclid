package us.ihmc.euclid.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * A {@code QuaternionBasedTransform} represents a 4-by-4 transformation matrix that can rotate and
 * translate.
 * <p>
 * The {@code QuaternionBasedTransform} provides the same features as {@link RigidBodyTransform}.
 * However, it stores the rotation part of the transform as a quaternion.
 * </p>
 * <p>
 * For efficiency and readability, the transform is never stored in a 4-by-4 matrix.
 * </p>
 * <p>
 * A few special cases to keep in mind:
 * <ul>
 * <li>when applying this transform on a {@link Point3DBasics} or {@link Point2DBasics}, this object
 * is rotated, then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is only rotated. It is NOT translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class QuaternionBasedTransform implements Transform, EpsilonComparable<QuaternionBasedTransform>, GeometricallyComparable<QuaternionBasedTransform>,
      Settable<QuaternionBasedTransform>, Clearable
{
   /** The rotation part of this transform. */
   private final Quaternion quaternion = new Quaternion();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new quaternion-based transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public QuaternionBasedTransform()
   {
      setIdentity();
   }

   /**
    * Creates a quaternion-based new transform and initializes it to {@code other}.
    *
    * @param other the other quaternion-based transform to copy. Not modified.
    */
   public QuaternionBasedTransform(QuaternionBasedTransform other)
   {
      set(other);
   }

   /**
    * Creates a new quaternion-based transform and initializes to the given rigid-body transform.
    *
    * @param rigidBodyTransform the rigid-body transform to copy. Not modified.
    */
   public QuaternionBasedTransform(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new quaternion-based transform given the value of its 7 components (4 components for
    * quaternion and 3 for the translation).
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = matrix.get(0, 0) \
    * q = | qy = matrix.get(1, 0) |
    *     | qz = matrix.get(2, 0) |
    *     \ qs = matrix.get(3, 0) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(4, 0) \
    * T = | matrix.get(5, 0) |
    *     \ matrix.get(6, 0) /
    * </pre>
    * </p>
    *
    * @param matrix the column vector containing the values of the 7 components of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Creates a new quaternion-based transform given the value of its 7 components (4 components for
    * quaternion and 3 for the translation).
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = array[0] \
    * q = | qy = array[1] |
    *     | qz = array[2] |
    *     \ qs = array[3] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / array[4] \
    * T = | array[5] |
    *     \ array[6] /
    * </pre>
    * </p>
    *
    * @param array the array containing the values of the 7 components of this transform. Not modified.
    */
   public QuaternionBasedTransform(double[] array)
   {
      set(array);
   }

   /**
    * Creates a new quaternion-based transform and initializes it to the given rotation matrix and
    * translation.
    *
    * @param rotationMatrix the rotation matrix used to initialize the quaternion of this transform.
    *           Not modified.
    * @param translation the tuple used to initialize the translation part of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   /**
    * Creates a new quaternion-based transform and initializes it to the given orientation and
    * translation.
    *
    * @param orientation the orientation used to initialize the quaternion of this transform. Not
    *           modified.
    * @param translation the tuple used to initialize the translation part of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
   }

   /**
    * Resets this quaternion-based transform to represent a zero rotation and zero translation.
    * <p>
    * When set to zero, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public void setIdentity()
   {
      quaternion.setToZero();
      translationVector.setToZero();
   }

   /**
    * Resets this quaternion-based transform to represent a zero rotation and zero translation.
    * <p>
    * When set to zero, this transform has no effect when transforming a geometry object.
    * </p>
    */
   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /**
    * Sets the rotation part to represent a 'zero' rotation.
    */
   public void setRotationToZero()
   {
      quaternion.setToZero();
   }

   /**
    * Sets the translation part to zero.
    */
   public void setTranslationToZero()
   {
      translationVector.setToZero();
   }

   /**
    * Sets all the components of this affine transform making it invalid.
    */
   @Override
   public void setToNaN()
   {
      quaternion.setToNaN();
      translationVector.setToNaN();
   }

   /**
    * Sets all the components of the quaternion to {@link Double#NaN}.
    * <p>
    * See {@link Quaternion#setToNaN()}.
    * </p>
    */
   public void setRotationToNaN()
   {
      quaternion.setToNaN();
   }

   /**
    * Sets all the components of the translation vector to {@link Double#NaN}.
    * <p>
    * See {@link Vector3D#setToNaN()}.
    * </p>
    */
   public void setTranslationToNaN()
   {
      translationVector.setToNaN();
   }

   /**
    * Tests if at least one element of this transform is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this transform is equal to
    *         {@linkplain Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return quaternion.containsNaN() || translationVector.containsNaN();
   }

   /**
    * Sets the 7 components of this transform.
    * <p>
    * The quaternion part will be normalized.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    * @param x the x-component of the translation.
    * @param y the y-component of the translation.
    * @param z the z-component of the translation.
    */
   public void set(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.set(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   /**
    * Sets the 7 components of this transform.
    * <p>
    * Prefer using the method {@link #set(double, double, double, double, double, double, double)} as
    * it normalizes the quaternion part.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    * @param x the x-component of the translation.
    * @param y the y-component of the translation.
    * @param z the z-component of the translation.
    */
   public void setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.setUnsafe(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   /**
    * Sets this quaternion-based transform to the given {@code other}.
    *
    * @param other the other quaternion-based transform. Not modified.
    */
   @Override
   public void set(QuaternionBasedTransform other)
   {
      quaternion.set(other.quaternion);
      translationVector.set(other.translationVector);
   }

   /**
    * Sets this quaternion-based transform to the given rigid-body transform.
    *
    * @param rigidBodyTransform the rigid-body transform to copy the values from. Not modified.
    */
   public void set(RigidBodyTransform rigidBodyTransform)
   {
      quaternion.set(rigidBodyTransform.getRotationMatrix());
      translationVector.set(rigidBodyTransform.getTranslationVector());
   }

   /**
    * Sets the value of this transform's 7 components.
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = matrix.get(0, 0) \
    * q = | qy = matrix.get(1, 0) |
    *     | qz = matrix.get(2, 0) |
    *     \ qs = matrix.get(3, 0) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(4, 0) \
    * T = | matrix.get(5, 0) |
    *     \ matrix.get(6, 0) /
    * </pre>
    * </p>
    *
    * @param matrix the column vector containing the values of the 7 components of this transform. Not
    *           modified.
    */
   public void set(DenseMatrix64F matrix)
   {
      quaternion.set(matrix);
      translationVector.set(4, matrix);
   }

   /**
    * Sets the value of this transform's 7 components.
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = array[0] \
    * q = | qy = array[1] |
    *     | qz = array[2] |
    *     \ qs = array[3] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / array[4] \
    * T = | array[5] |
    *     \ array[6] /
    * </pre>
    * </p>
    *
    * @param array the array containing the values of the 7 components of this transform. Not modified.
    */
   public void set(double[] array)
   {
      quaternion.set(array);
      translationVector.set(4, array);
   }

   /**
    * Sets this quaternion-based transform to the given rotation matrix and translation.
    *
    * @param rotationMatrix the rotation matrix used to set the quaternion of this transform. Not
    *           modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      quaternion.set(rotationMatrix);
      translationVector.set(translation);
   }

   /**
    * Sets this quaternion-based transform to the given orientation and translation.
    *
    * @param orientation the orientation used to set the quaternion of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   public void set(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      quaternion.set(orientation);
      translationVector.set(translation);
   }

   /**
    * Sets the rotation part of this transform to the given orientation.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param orientation the orientation used to set the quaternion of this transform. Not modified.
    */
   public void setRotation(Orientation3DReadOnly orientation)
   {
      quaternion.set(orientation);
   }

   /**
    * Sets the rotation part of this transform to the given rotation matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the rotation matrix used to set the quaternion of this transform. Not
    *           modified.
    */
   public void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      quaternion.set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the quaternion of this transform. Not
    *           modified.
    */
   public void setRotation(Vector3DReadOnly rotationVector)
   {
      quaternion.setRotationVector(rotationVector);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * z-axis of an angle {@code yaw}.
    *
    * <pre>
    *     / qx =     0      \
    * q = | qy =     0      |
    *     | qz = sin(yaw/2) |
    *     \ qs = cos(yaw/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYaw(double yaw)
   {
      quaternion.setToYawQuaternion(yaw);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * y-axis of an angle {@code pitch}.
    *
    * <pre>
    *     / qx =      0       \
    * q = | qy = sin(pitch/2) |
    *     | qz =      0       |
    *     \ qs = cos(pitch/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitch(double pitch)
   {
      quaternion.setToPitchQuaternion(pitch);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * x-axis of an angle {@code roll}.
    *
    * <pre>
    *     / qx = sin(roll/2) \
    * q = | qy =      0      |
    *     | qz =      0      |
    *     \ qs = cos(roll/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRoll(double roll)
   {
      quaternion.setToRollQuaternion(roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / qx =     0      \   / qx =      0       \   / qx = sin(roll/2) \
    * q = | qy =     0      | * | qy = sin(pitch/2) | * | qy =      0      |
    *     | qz = sin(yaw/2) |   | qz =      0       |   | qz =      0      |
    *     \ qs = cos(yaw/2) /   \ qs = cos(pitch/2) /   \ qs = cos(roll/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    * @deprecated Use {@link YawPitchRoll} with {@link #setRotation(Orientation3DReadOnly)}
    */
   public void setRotationYawPitchRoll(double[] yawPitchRoll)
   {
      quaternion.setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / qx =     0      \   / qx =      0       \   / qx = sin(roll/2) \
    * q = | qy =     0      | * | qy = sin(pitch/2) | * | qy =      0      |
    *     | qz = sin(yaw/2) |   | qz =      0       |   | qz =      0      |
    *     \ qs = cos(yaw/2) /   \ qs = cos(pitch/2) /   \ qs = cos(roll/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      quaternion.setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code eulerAngles}.
    *
    * <pre>
    *     / qx =           0          \   / qx =           0          \   / qx = sin(eulerAngles.x/2) \
    * q = | qy =           0          | * | qy = sin(eulerAngles.y/2) | * | qy =           0          |
    *     | qz = sin(eulerAngles.z/2) |   | qz =           0          |   | qz =           0          |
    *     \ qs = cos(eulerAngles.z/2) /   \ qs = cos(eulerAngles.y/2) /   \ qs = cos(eulerAngles.x/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setRotationEuler(Vector3DReadOnly eulerAngles)
   {
      quaternion.setEuler(eulerAngles);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *     / qx =      0      \   / qx =      0      \   / qx = sin(rotX/2) \
    * q = | qy =      0      | * | qy = sin(rotY/2) | * | qy =      0      |
    *     | qz = sin(rotZ/2) |   | qz =      0      |   | qz =      0      |
    *     \ qs = cos(rotZ/2) /   \ qs = cos(rotY/2) /   \ qs = cos(rotX/2) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to {@code this.setRotationYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      quaternion.setEuler(rotX, rotY, rotZ);
   }

   /**
    * Sets the x-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    */
   public void setTranslationX(double x)
   {
      translationVector.setX(x);
   }

   /**
    * Sets the y-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param y the y-component of the translation part.
    */
   public void setTranslationY(double y)
   {
      translationVector.setY(y);
   }

   /**
    * Sets the z-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param z the z-component of the translation part.
    */
   public void setTranslationZ(double z)
   {
      translationVector.setZ(z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    */
   public void setTranslation(double x, double y, double z)
   {
      translationVector.set(x, y, z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   public void setTranslation(Tuple3DReadOnly translation)
   {
      translationVector.set(translation);
   }

   /**
    * Inverts this quaternion-based transform.
    */
   public void invert()
   {
      quaternion.conjugate();
      quaternion.transform(translationVector);
      translationVector.negate();
   }

   /**
    * Inverts only the rotation part of this transform, the translation remains unchanged.
    */
   public void invertRotation()
   {
      quaternion.conjugate();
   }

   /**
    * Performs a linear interpolation from this transform to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation is done on the rotation part and translation part separately.
    * </p>
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other transform used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *           this transform, while a value of 1 is equivalent to setting this transform to
    *           {@code other}.
    */
   public void interpolate(QuaternionBasedTransform other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code firstTransform} to {@code secondTransform} given the
    * percentage {@code alpha}.
    * <p>
    * The interpolation is done on the rotation part and translation part separately.
    * </p>
    * <p>
    * this = (1.0 - alpha) * firstTransform + alpha * secondTransform
    * </p>
    *
    * @param firstTransform the first transform used in the interpolation. Not modified.
    * @param secondTransform the second transform used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this transform to {@code tuple1}, while a value of 1 is equivalent to setting this
    *           transform to {@code tuple2}.
    */
   public void interpolate(QuaternionBasedTransform firstTransform, QuaternionBasedTransform secondTransform, double alpha)
   {
      quaternion.interpolate(firstTransform.getQuaternion(), secondTransform.getQuaternion(), alpha);
      translationVector.interpolate(firstTransform.getTranslationVector(), secondTransform.getTranslationVector(), alpha);
   }

   /**
    * Performs the multiplication of this transform with {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiply(QuaternionBasedTransform other)
   {
      QuaternionTools.addTransform(quaternion, other.getTranslationVector(), translationVector);
      quaternion.multiply(other.getQuaternion());
   }

   /**
    * Performs the multiplication of this transform with {@code rigidBodyTransform}.
    * <p>
    * this = this * Q(rigidBodyTransform) <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void multiply(RigidBodyTransform rigidBodyTransform)
   {
      QuaternionTools.addTransform(quaternion, rigidBodyTransform.getTranslationVector(), translationVector);
      quaternion.append(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this * Q(affineTransform) <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiply(AffineTransform affineTransform)
   {
      QuaternionTools.addTransform(quaternion, affineTransform.getTranslationVector(), translationVector);
      quaternion.append(affineTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiplyInvertThis(QuaternionBasedTransform other)
   {
      translationVector.sub(other.getTranslationVector(), translationVector);
      quaternion.inverseTransform(translationVector, translationVector);
      quaternion.multiplyConjugateThis(other.quaternion);
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code other}.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void multiplyInvertOther(QuaternionBasedTransform other)
   {
      quaternion.multiplyConjugateOther(other.getQuaternion());
      QuaternionTools.subTransform(quaternion, other.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code rigidBodyTransform}.
    * <p>
    * this = this<sup>-1</sup> * Q(rigidBodyTransform) <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void multiplyInvertThis(RigidBodyTransform rigidBodyTransform)
   {
      translationVector.sub(rigidBodyTransform.getTranslationVector(), translationVector);
      quaternion.inverseTransform(translationVector, translationVector);
      quaternion.appendInvertThis(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code rigidBodyTransform}.
    * <p>
    * this = this * Q(rigidBodyTransform)<sup>-1</sup> <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void multiplyInvertOther(RigidBodyTransform rigidBodyTransform)
   {
      quaternion.appendInvertOther(rigidBodyTransform.getRotationMatrix());
      QuaternionTools.subTransform(quaternion, rigidBodyTransform.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this<sup>-1</sup> * Q(affineTransform) <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiplyInvertThis(AffineTransform affineTransform)
   {
      translationVector.sub(affineTransform.getTranslationVector(), translationVector);
      quaternion.inverseTransform(translationVector, translationVector);
      quaternion.appendInvertThis(affineTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this * Q(affineTransform)<sup>-1</sup> <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void multiplyInvertOther(AffineTransform affineTransform)
   {
      quaternion.appendInvertOther(affineTransform.getRotationMatrix());
      QuaternionTools.subTransform(quaternion, affineTransform.getTranslationVector(), translationVector);
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 translation.x \
    * this = this * | 0 1 0 translation.y |
    *               | 0 0 1 translation.z |
    *               \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to append to this transform. Not modified.
    */
   public void appendTranslation(Tuple3DReadOnly translation)
   {
      QuaternionTools.addTransform(quaternion, translation, translationVector);
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 x \
    * this = this * | 0 1 0 y |
    *               | 0 0 1 z |
    *               \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   public void appendTranslation(double x, double y, double z)
   {
      double thisX = translationVector.getX();
      double thisY = translationVector.getY();
      double thisZ = translationVector.getZ();

      translationVector.set(x, y, z);
      quaternion.transform(translationVector);
      translationVector.add(thisX, thisY, thisZ);
   }

   /**
    * Append a rotation about the z-axis to the rotation part 'q' of this transform.
    *
    * <pre>
    *         / qx =     0      \
    * q = q * | qy =     0      |
    *         | qz = sin(yaw/2) |
    *         \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      quaternion.appendYawRotation(yaw);
   }

   /**
    * Append a rotation about the y-axis to the rotation part 'q' of this transform.
    *
    * <pre>
    *         / qx =      0       \
    * q = q * | qy = sin(pitch/2) |
    *         | qz =      0       |
    *         \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      quaternion.appendPitchRotation(pitch);
   }

   /**
    * Append a rotation about the x-axis to the rotation part 'q' of this transform.
    *
    * <pre>
    *         / qx = sin(roll/2) \
    * q = q * | qy =      0      |
    *         | qz =      0      |
    *         \ qs = cos(roll/2) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      quaternion.appendRollRotation(roll);
   }

   /**
    * Performs the multiplication of {@code other} with this transform.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiply(QuaternionBasedTransform other)
   {
      QuaternionTools.transform(other.getQuaternion(), translationVector, translationVector);
      translationVector.add(other.getTranslationVector());
      quaternion.preMultiply(other.getQuaternion());
   }

   /**
    * Performs the multiplication of {@code rigidBodyTransform} with this transform.
    * <p>
    * this = Q(rigidBodyTransform) * this <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void preMultiply(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.transform(translationVector);
      translationVector.add(rigidBodyTransform.getTranslationVector());
      quaternion.prepend(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = Q(affineTransform) * this <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void preMultiply(AffineTransform affineTransform)
   {
      affineTransform.getRotationMatrix().transform(translationVector);
      translationVector.add(affineTransform.getTranslationVector());
      quaternion.prepend(affineTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of {@code other} with the inverse of this transform.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertThis(QuaternionBasedTransform other)
   {
      quaternion.preMultiplyConjugateThis(other.getQuaternion());
      quaternion.transform(translationVector);
      translationVector.sub(other.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of {@code other} with this transform.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertOther(QuaternionBasedTransform other)
   {
      translationVector.sub(other.getTranslationVector());
      other.getQuaternion().inverseTransform(translationVector);
      quaternion.preMultiplyConjugateOther(other.getQuaternion());
   }

   /**
    * Performs the multiplication of {@code rigidBodyTransform} with the inverse of this transform.
    * <p>
    * this = Q(rigidBodyTransform) * this<sup>-1</sup> <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertThis(RigidBodyTransform rigidBodyTransform)
   {
      quaternion.prependInvertThis(rigidBodyTransform.getRotationMatrix());
      quaternion.transform(translationVector);
      translationVector.sub(rigidBodyTransform.getTranslationVector(), translationVector);
   }

   /**
    * Performs the multiplication of the inverse of {@code rigidBodyTransform} with this transform.
    * <p>
    * this = Q(rigidBodyTransform)<sup>-1</sup> * this <br>
    * where Q(rigidBodyTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertOther(RigidBodyTransform rigidBodyTransform)
   {
      translationVector.sub(rigidBodyTransform.getTranslationVector());
      rigidBodyTransform.getRotationMatrix().inverseTransform(translationVector);
      quaternion.prependInvertOther(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with the inverse of this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = Q(affineTransform) * this<sup>-1</sup> <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertThis(AffineTransform affineTransform)
   {
      quaternion.prependInvertThis(affineTransform.getRotationMatrix());
      quaternion.transform(translationVector);
      translationVector.sub(affineTransform.getTranslationVector(), translationVector);

   }

   /**
    * Performs the multiplication of the inverse of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper quaternion-based transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = Q(affineTransform)<sup>-1</sup> * this <br>
    * where Q(affineTransform) is the function that converts a 4-by-4 transformation matrix into a
    * quaternion-based transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   public void preMultiplyInvertOther(AffineTransform affineTransform)
   {
      translationVector.sub(affineTransform.getTranslationVector());
      affineTransform.getRotationMatrix().inverseTransform(translationVector);
      quaternion.prependInvertOther(affineTransform.getRotationMatrix());
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 translation.x \
    * this = | 0 1 0 translation.y | * this
    *        | 0 0 1 translation.z |
    *        \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to prepend to this transform. Not modified.
    */
   public void prependTranslation(Tuple3DReadOnly translation)
   {
      translationVector.add(translation);
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 x \
    * this = | 0 1 0 y | * this
    *        | 0 0 1 z |
    *        \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   public void prependTranslation(double x, double y, double z)
   {
      translationVector.add(x, y, z);
   }

   /**
    * Prepend a rotation about the z-axis to this transform.
    * <p>
    * This method first rotates the translation part 't' and then prepend the yaw-rotation to the
    * rotation part 'q' of this transform.
    * </p>
    *
    * <pre>
    * t = q(yaw) * t
    * q = q(yaw) * q
    * </pre>
    *
    * where:
    *
    * <pre>
    *          / qx =     0      \
    * q(yaw) = | qy =     0      |
    *          | qz = sin(yaw/2) |
    *          \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, translationVector, translationVector);
      quaternion.prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to this transform.
    * <p>
    * This method first rotates the translation part 't' and then prepend the pitch-rotation to the
    * rotation part 'q' of this transform.
    * </p>
    *
    * <pre>
    * t = q(pitch) * t
    * q = q(pitch) * q
    * </pre>
    *
    * where:
    *
    * <pre>
    *            / qx =      0       \
    * q(pitch) = | qy = sin(pitch/2) |
    *            | qz =      0       |
    *            \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, translationVector, translationVector);
      quaternion.prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to the rotation part 'q' of this transform.
    * <p>
    * This method first rotates the translation part 't' and then prepend the roll-rotation to the
    * rotation part 'q' of this transform.
    * </p>
    *
    * <pre>
    * t = q(roll) * t
    * q = q(roll) * q
    * </pre>
    *
    * where:
    *
    * <pre>
    *           / qx = sin(roll/2) \
    * q(roll) = | qy =      0      |
    *           | qz =      0      |
    *           \ qs = cos(roll/2) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, translationVector, translationVector);
      quaternion.prependRollRotation(roll);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      quaternion.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      quaternion.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      quaternion.transform(orientationOriginal, orientationTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      quaternion.transform(vectorOriginal, vectorTransformed);
      vectorTransformed.addX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.addY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.addZ(vectorTransformed.getS() * translationVector.getZ());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      quaternion.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      quaternion.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      pointTransformed.add(translationVector.getX(), translationVector.getY());
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      quaternion.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(AffineTransform original, AffineTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiply(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      quaternion.inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      quaternion.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      quaternion.inverseTransform(orientationOriginal, orientationTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.set(vectorOriginal);
      vectorTransformed.subX(vectorTransformed.getS() * translationVector.getX());
      vectorTransformed.subY(vectorTransformed.getS() * translationVector.getY());
      vectorTransformed.subZ(vectorTransformed.getS() * translationVector.getZ());
      quaternion.inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      quaternion.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      quaternion.inverseTransform(pointTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      quaternion.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(AffineTransform original, AffineTransform transformed)
   {
      transformed.set(original);
      transformed.preMultiplyInvertOther(this);
   }

   /**
    * Packs this quaternion-based transform in a column vector.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    *
    * @param matrixToPack the column vector in which this transform is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      quaternion.get(matrixToPack);
      translationVector.get(4, matrixToPack);
   }

   /**
    * Packs this quaternion-based transform in a column vector.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    *
    * @param startRow the first row index to start writing in {@code matrixToPack}.
    * @param column the column index to write in {@code matrixToPack}.
    * @param matrixToPack the column vector in which this transform is stored. Modified.
    */
   public void get(int startRow, int column, DenseMatrix64F matrixToPack)
   {
      quaternion.get(startRow, column, matrixToPack);
      translationVector.get(startRow + 4, column, matrixToPack);
   }

   /**
    * Packs this quaternion-based transform in an array.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    * 
    * 
    * @param transformArrayToPack the array in which this transform is packed. Modified.
    */
   public void get(double[] transformArrayToPack)
   {
      quaternion.get(transformArrayToPack);
      translationVector.get(4, transformArrayToPack);
   }

   /**
    * Packs the orientation and translation of this quaternion-based transform.
    *
    * @param orientationToPack the orientation in which this transform's quaternion is stored. Modified.
    * @param translationToPack the tuple in which this transform's translation is stored. Modified.
    */
   public void get(Orientation3DBasics orientationToPack, Tuple3DBasics translationToPack)
   {
      orientationToPack.set(quaternion);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the quaternion and translation of this quaternion-based transform.
    *
    * @param rotationMarixToPack the rotation matrix that is set to this transform's quaternion.
    *           Modified
    * @param translationToPack the tuple in which this transform's translation is stored. Modified.
    */
   public void get(RotationMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(quaternion);
      translationToPack.set(translationVector);
   }

   /**
    * Packs the quaternion and translation of this quaternion-based transform.
    *
    * @param rotationMarixToPack the rotation-scale matrix that is set to this transform's quaternion.
    *           The scale part is reset. Modified.
    * @param translationToPack the tuple in which this transform's translation is stored. Modified.
    */
   public void get(RotationScaleMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(quaternion, 1.0);
      translationToPack.set(translationVector);
   }

   /**
    * Get the read-only reference to the quaternion of this transform.
    *
    * @return the quaternion of this transform.
    */
   public QuaternionReadOnly getQuaternion()
   {
      return quaternion;
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *           Modified.
    */
   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(quaternion);
   }

   /**
    * Packs the rotation part of this affine transform.
    *
    * @param orientationToPack the orientation that is set to the rotation part of this transform.
    *           Modified.
    */
   public void getRotation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(quaternion);
   }

   /**
    * Packs the rotation part of this affine transform as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector that is set to the rotation part of this
    *           transform. Modified.
    */
   public void getRotation(Vector3DBasics rotationVectorToPack)
   {
      quaternion.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by the rotation part of this transform as the
    * yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    * @deprecated Use {@link YawPitchRoll} with {@link #getRotation(Orientation3DBasics)}.
    */
   public void getRotationYawPitchRoll(double[] yawPitchRollToPack)
   {
      quaternion.getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and packs the orientation described by the rotation part of this transform as the Euler
    * angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   public void getRotationEuler(Vector3DBasics eulerAnglesToPack)
   {
      quaternion.getEuler(eulerAnglesToPack);
   }

   /**
    * Gets the read-only reference of the translation part of this affine transform.
    *
    * @return the translation part of this transform.
    */
   public Vector3DReadOnly getTranslationVector()
   {
      return translationVector;
   }

   /**
    * Packs the translation part of this affine transform.
    *
    * @param translationToPack the tuple in which the translation part of this transform is stored.
    *           Modified.
    */
   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   /**
    * Gets the x-component of the translation part of this transform.
    *
    * @return the x-component of the translation part.
    */
   public double getTranslationX()
   {
      return translationVector.getX();
   }

   /**
    * Gets the y-component of the translation part of this transform.
    *
    * @return the y-component of the translation part.
    */
   public double getTranslationY()
   {
      return translationVector.getY();
   }

   /**
    * Gets the z-component of the translation part of this transform.
    *
    * @return the z-component of the translation part.
    */
   public double getTranslationZ()
   {
      return translationVector.getZ();
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other quaternion-based transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(QuaternionBasedTransform other, double epsilon)
   {
      return quaternion.epsilonEquals(other.quaternion, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(QuaternionBasedTransform)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof QuaternionBasedTransform)
         return equals((QuaternionBasedTransform) object);
      else
         return false;
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are exactly equal.
    * <p>
    * The method returns {@code false} if the given transform is {@code null}.
    * </p>
    *
    * @param other the other transform to compare against this. Not modified.
    * @return {@code true} if the two transforms are exactly equal, {@code false} otherwise.
    */
   public boolean equals(QuaternionBasedTransform other)
   {
      if (other == null)
         return false;
      else
         return quaternion.equals(other.quaternion) && translationVector.equals(other.translationVector);
   }

   /**
    * Two quaternion based transforms are considered geometrically equal if both the rotation-scale
    * matrices and translation vectors are equal.
    *
    * @param other the other quaternion based transform to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two quaternion based transforms are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(QuaternionBasedTransform other, double epsilon)
   {
      return other.quaternion.geometricallyEquals(quaternion, epsilon) && other.translationVector.geometricallyEquals(translationVector, epsilon);
   }

   /**
    * Provides a {@code String} representation of this transform as follows: <br>
    * Quaternion: (qx, qy, qz, qs) <br>
    * Translation: ( x, y, z)
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getQuaternionBasedTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(quaternion.hashCode(), translationVector.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}

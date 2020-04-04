package us.ihmc.euclid.tools;

import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.AxisAngle32;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * This class provides random generators to generate random geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreRandomTools
{
   private EuclidCoreRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates random a yaw-pitch-roll orientation.
    * <p>
    * <ul>
    * <li>yaw &in; [-<i>pi</i>; <i>pi</i>],
    * <li>pitch &in; [-<i>pi</i>/2.0; <i>pi</i>/2.0],
    * <li>roll &in; [-<i>pi</i>; <i>pi</i>],
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random yaw-pitch-roll orientation.
    */
   public static YawPitchRoll nextYawPitchRoll(Random random)
   {
      return nextYawPitchRoll(random, Math.PI, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE, Math.PI);
   }

   /**
    * Generates random a yaw-pitch-roll orientation.
    * <p>
    * <ul>
    * <li>yaw &in; [-{@code minMaxYaw}; {@code minMaxYaw}],
    * <li>pitch &in; [-{@code minMaxPitch}; {@code minMaxPitch}],
    * <li>roll &in; [-{@code minMaxRoll}; {@code minMaxRoll}],
    * </ul>
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxYaw   the maximum absolute angle for the generated yaw angle.
    * @param minMaxPitch the maximum absolute angle for the generated pitch angle.
    * @param minMaxRoll  the maximum absolute angle for the generated roll angle.
    * @return the random yaw-pitch-roll orientation.
    * @throws RuntimeException if {@code minMaxYaw < 0}, {@code minMaxPitch < 0},
    *                          {@code minMaxRoll < 0}.
    */
   public static YawPitchRoll nextYawPitchRoll(Random random, double minMaxYaw, double minMaxPitch, double minMaxRoll)
   {
      double yaw = nextDouble(random, minMaxYaw);
      double pitch = nextDouble(random, minMaxPitch);
      double roll = nextDouble(random, minMaxRoll);
      return new YawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Generates a random yaw-pitch-roll orientation uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated orientation is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated orientation.
    * @return the random yaw-pitch-roll orientation.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static YawPitchRoll nextYawPitchRollUniform(Random random, double minMaxAngle)
   {
      return new YawPitchRoll(nextAxisAngle(random, minMaxAngle));
   }

   /**
    * Generates a random rotation vector.
    * <p>
    * {@code rotationVector.length()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random rotation vector.
    */
   public static Vector3D nextRotationVector(Random random)
   {
      return nextRotationVector(random, Math.PI);
   }

   /**
    * Generates a random rotation vector.
    * <p>
    * {@code rotationVector.length()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum length of the generated rotation vector.
    * @return the random rotation vector.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Vector3D nextRotationVector(Random random, double minMaxAngle)
   {
      Vector3D rotationVector = new Vector3D();
      AxisAngle axisAngle = nextAxisAngle(random, minMaxAngle);
      axisAngle.getRotationVector(rotationVector);
      return rotationVector;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random axis-angle.
    */
   public static AxisAngle nextAxisAngle(Random random)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle value.
    * @return the random axis-angle.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static AxisAngle nextAxisAngle(Random random, double minMaxAngle)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, minMaxAngle, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random axis-angle.
    */
   public static AxisAngle32 nextAxisAngle32(Random random)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle value.
    * @return the random axis-angle.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static AxisAngle32 nextAxisAngle32(Random random, double minMaxAngle)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, minMaxAngle, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM11()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM22()} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random diagonal matrix.
    */
   public static Matrix3D nextDiagonalMatrix3D(Random random)
   {
      return nextDiagonalMatrix3D(random, 1.0);
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM11()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM22()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </ul>
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxValue the maximum absolute value for each diagonal element.
    * @return the random diagonal matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D nextDiagonalMatrix3D(Random random, double minMaxValue)
   {
      return nextDiagonalMatrix3D(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM11()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM22()} &in; [{@code minValue}; {@code maxValue}].
    * </ul>
    * </p>
    *
    * @param random   the random generator to use.
    * @param minValue the minimum value of each diagonal element.
    * @param maxValue the maximum value of each diagonal element.
    * @return the random diagonal matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static Matrix3D nextDiagonalMatrix3D(Random random, double minValue, double maxValue)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int i = 0; i < 3; i++)
         matrix3D.setElement(i, i, nextDouble(random, minValue, maxValue));
      return matrix3D;
   }

   /**
    * Generates a random double &in; [-1.0; 1.0].
    *
    * @param random the random generator to use.
    * @return the random double.
    */
   public static double nextDouble(Random random)
   {
      return nextDouble(random, 1.0);
   }

   /**
    * Generates a random double &in; [-{@code minMax}; {@code minMax}].
    *
    * @param random      the random generator to use.
    * @param minMaxValue the maximum absolute value of the generated double.
    * @return the random double.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static double nextDouble(Random random, double minMaxValue)
   {
      return nextDouble(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random double &in; [{@code minValue}; {@code maxValue}].
    *
    * @param random   the random generator to use.
    * @param minValue the minimum value of the generated double.
    * @param maxValue the maximum value of the generated double.
    * @return the random double.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static double nextDouble(Random random, double minValue, double maxValue)
   {
      if (minValue > maxValue)
         throw new RuntimeException("Min is greater than max: min = " + minValue + ", max = " + maxValue);

      return minValue + random.nextDouble() * (maxValue - minValue);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random matrix.
    */
   public static Matrix3D nextMatrix3D(Random random)
   {
      return nextMatrix3D(random, 1.0);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxValue the maximum absolute value for each element.
    * @return the random matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D nextMatrix3D(Random random, double minMaxValue)
   {
      return nextMatrix3D(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [{@code minValue}; {@code maxValue}].
    * </p>
    *
    * @param random   the random generator to use.
    * @param minValue the minimum value for each element.
    * @param maxValue the maximum value for each element.
    * @return the random matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static Matrix3D nextMatrix3D(Random random, double minValue, double maxValue)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrix3D.setElement(row, column, nextDouble(random, minValue, maxValue));
         }
      }
      return matrix3D;
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion.
    */
   public static Quaternion nextQuaternion(Random random)
   {
      return new Quaternion(nextAxisAngle(random));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated quaternion.
    * @return the random quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Quaternion nextQuaternion(Random random, double minMaxAngle)
   {
      return new Quaternion(nextAxisAngle(random, minMaxAngle));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion.
    */
   public static Quaternion32 nextQuaternion32(Random random)
   {
      return new Quaternion32(nextAxisAngle(random));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random           the random generator to use.
    * @param minMaxAngleRange the maximum absolute angle described by the generated quaternion.
    * @return the random quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Quaternion32 nextQuaternion32(Random random, double minMaxAngleRange)
   {
      return new Quaternion32(nextAxisAngle(random, minMaxAngleRange));
   }

   /**
    * Generates an orientation which both value and type are random.
    * <p>
    * The type can be either: axis-angle, quaternion, rotation matrix, or yaw-pitch-roll.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random orientation 3D.
    */
   public static Orientation3DBasics nextOrientation3D(Random random)
   {
      switch (random.nextInt(4))
      {
         case 0:
            return nextAxisAngle(random);
         case 1:
            return nextQuaternion(random);
         case 2:
            return nextRotationMatrix(random);
         default:
            return nextYawPitchRoll(random);
      }
   }

   /**
    * Generates a random rigid-body transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-<i>pi</i>; <i>pi</i>].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random rigid-body transform.
    */
   public static RigidBodyTransform nextRigidBodyTransform(Random random)
   {
      return new RigidBodyTransform(nextAxisAngle(random), nextVector3D(random));
   }

   /**
    * Generates a random rigid-body transform with the rotation part being a transform in the XY plane.
    *
    * @param random the random generator to use.
    * @return the random rigid-body transform.
    */
   public static RigidBodyTransform nextRigidBodyTransform2D(Random random)
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.getTranslation().set(nextPoint3D(random));
      rigidBodyTransform.getRotation().setToYawOrientation(nextDouble(random, Math.PI));
      return rigidBodyTransform;
   }

   /**
    * Generates a random quaternion-based transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-<i>pi</i>; <i>pi</i>].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion-based transform.
    */
   public static QuaternionBasedTransform nextQuaternionBasedTransform(Random random)
   {
      return new QuaternionBasedTransform(nextQuaternion(random), nextVector3D(random));
   }

   /**
    * Generates a random affine transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-<i>pi</i>; <i>pi</i>].
    * <li>Each scale factor is in ]0.0; 10.0].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random affine transform.
    */
   public static AffineTransform nextAffineTransform(Random random)
   {
      return new AffineTransform(nextRotationScaleMatrix(random, 10.0), nextVector3D(random));
   }

   /**
    * Generates a random rotation matrix uniformly distributed on the unit sphere and describes an
    * rotation angle in [-<i>pi</i>; <i>pi</i>].
    *
    * @param random the random generator to use.
    * @return the random rotation matrix.
    */
   public static RotationMatrix nextRotationMatrix(Random random)
   {
      return nextRotationMatrix(random, Math.PI);
   }

   /**
    * Generates a random rotation matrix uniformly distributed on the unit sphere and describes an
    * rotation angle in [-{@code minMaxAngle}; {@code minMaxAngle}].
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated rotation matrix.
    * @return the random rotation matrix.
    */
   public static RotationMatrix nextRotationMatrix(Random random, double minMaxAngle)
   {
      AxisAngle randomRotation = nextAxisAngle(random, minMaxAngle);
      return new RotationMatrix(randomRotation);
   }

   /**
    * Generates a random rotation-scale matrix.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-<i>pi</i>; <i>pi</i>].
    * <li>Each scale factor is in ]0.0; 10.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random rotation-scale matrix.
    * @throws RuntimeException if {@code maxScale < 0}.
    */
   public static RotationScaleMatrix nextRotationScaleMatrix(Random random)
   {
      return nextRotationScaleMatrix(random, 10.0);
   }

   /**
    * Generates a random rotation-scale matrix.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-<i>pi</i>; <i>pi</i>].
    * <li>Each scale factor is in ]0.0; {@code maxScale}].
    * </ul>
    * </p>
    *
    * @param random   the random generator to use.
    * @param maxScale the maximum scale value used for each scale factor.
    * @return the random rotation-scale matrix.
    * @throws RuntimeException if {@code maxScale < 0}.
    */
   public static RotationScaleMatrix nextRotationScaleMatrix(Random random, double maxScale)
   {
      return nextRotationScaleMatrix(random, Math.PI, maxScale);
   }

   /**
    * Generates a random rotation-scale matrix.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation angle
    * in [-{@code minMaxAngle}; {@code minMaxAngle}].
    * <li>Each scale factor is in ]0.0; {@code maxScale}].
    * </ul>
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxAngle the maximum absolute angle value that describes the generated rotation-scale
    *                    matrix.
    * @param maxScale    the maximum scale value used for each scale factor.
    * @return the random rotation-scale matrix.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    * @throws RuntimeException if {@code maxScale < 0}.
    */
   public static RotationScaleMatrix nextRotationScaleMatrix(Random random, double minMaxAngle, double maxScale)
   {
      AxisAngle randomRotation = nextAxisAngle(random, minMaxAngle);
      Vector3D randomScales = nextVector3D(random, new Vector3D(0.0, 0.0, 0.0), new Vector3D(maxScale, maxScale, maxScale));
      return new RotationScaleMatrix(randomRotation, randomScales);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random point.
    */
   public static Point3D nextPoint3D(Random random)
   {
      Point3D point = new Point3D();
      randomizeTuple3D(random, point);
      return point;
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [-minMax; minMax]. <br>
    * {@code point.y} &in; [-minMax; minMax]. <br>
    * {@code point.z} &in; [-minMax; minMax]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static Point3D nextPoint3D(Random random, double minMax)
   {
      double x = nextDouble(random, -minMax, minMax);
      double y = nextDouble(random, -minMax, minMax);
      double z = nextDouble(random, -minMax, minMax);

      return new Point3D(x, y, z);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [min; max]. <br>
    * {@code point.y} &in; [min; max]. <br>
    * {@code point.z} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param min    the minimum value for each coordinate.
    * @param max    the maximum value for each coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static Point3D nextPoint3D(Random random, double min, double max)
   {
      double x = nextDouble(random, min, max);
      double y = nextDouble(random, min, max);
      double z = nextDouble(random, min, max);

      return new Point3D(x, y, z);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code point.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code point.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    *
    * @param random       the random generator to use.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *                          {@code maxAbsoluteZ < 0}.
    */
   public static Point3D nextPoint3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = nextDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = nextDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = nextDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D(x, y, z);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [minX; maxX]. <br>
    * {@code point.y} &in; [minY; maxY]. <br>
    * {@code point.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param minX   the minimum value for the x-coordinate.
    * @param maxX   the maximum value for the x-coordinate.
    * @param minY   the minimum value for the y-coordinate.
    * @param maxY   the maximum value for the y-coordinate.
    * @param minZ   the minimum value for the z-coordinate.
    * @param maxZ   the maximum value for the z-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static Point3D nextPoint3D(Random random, double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
   {
      double x = nextDouble(random, minX, maxX);
      double y = nextDouble(random, minY, maxY);
      double z = nextDouble(random, minZ, maxZ);

      return new Point3D(x, y, z);
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random vector.
    */
   public static Vector3D nextVector3D(Random random)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, vector);
      return vector;
   }

   /**
    * Generates a random unit vector.
    * <p>
    * This generator uses {@link #nextVector3D(Random)}.
    * </p>
    * 
    * @param random the random generator to use.
    * @return the random unit vector.
    */
   public static UnitVector3D nextUnitVector3D(Random random)
   {
      return new UnitVector3D(nextVector3D(random));
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *               vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static Vector3D nextVector3D(Random random, Tuple3DReadOnly minMax)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, minMax, vector);
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min    tuple used as upper-bound for each component of the generated vector. Not modified.
    * @param max    tuple used as lower-bound for each component of the generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static Vector3D nextVector3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, min, max, vector);
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param min    upper-bound for each component of the generated vector. Not modified.
    * @param max    lower-bound for each component of the generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static Vector3D nextVector3D(Random random, double min, double max)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, new Point3D(min, min, min), new Point3D(max, max, max), vector);
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector.x} &in; [minX; maxX]. <br>
    * {@code vector.y} &in; [minY; maxY]. <br>
    * {@code vector.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param minX   the minimum value for the x-component.
    * @param maxX   the maximum value for the x-component.
    * @param minY   the minimum value for the y-component.
    * @param maxY   the maximum value for the y-component.
    * @param minZ   the minimum value for the z-component.
    * @param maxZ   the maximum value for the z-component.
    * @return the random vector.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static Vector3D nextVector3D(Random random, double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
   {
      double x = nextDouble(random, minX, maxX);
      double y = nextDouble(random, minY, maxY);
      double z = nextDouble(random, minZ, maxZ);

      return new Vector3D(x, y, z);
   }

   /**
    * Generates a random vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param length the length of the generated vector.
    * @return the random vector.
    */
   public static Vector3D nextVector3DWithFixedLength(Random random, double length)
   {
      Vector3D vector = nextVector3D(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   /**
    * Generates a random vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random                 the random generator to use.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize              whether to normalize the generated vector or not.
    * @return the random vector.
    */
   public static Vector3D nextOrthogonalVector3D(Random random, Vector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector3D v1 = new Vector3D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX(), 0.0);
      Vector3D v2 = new Vector3D(-vectorToBeOrthogonalTo.getZ(), 0.0, vectorToBeOrthogonalTo.getX());

      if (v1.lengthSquared() < 1.0e-12)
         v1.cross(vectorToBeOrthogonalTo, v2);
      if (v2.lengthSquared() < 1.0e-12)
         v2.cross(v1, vectorToBeOrthogonalTo);

      Vector3D randomPerpendicular = new Vector3D();
      double a = nextDouble(random, 1.0);
      double b = nextDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);
      randomPerpendicular.scaleAdd(b, v2, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   /**
    * Generates a random 2D point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D point.
    */
   public static Point2D nextPoint2D(Random random)
   {
      Point2D point = new Point2D();
      randomizeTuple2D(random, point);
      return point;
   }

   /**
    * Generates a random 2D point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-minMax; minMax].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random 2D point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static Point2D nextPoint2D(Random random, double minMax)
   {
      double x = nextDouble(random, -minMax, minMax);
      double y = nextDouble(random, -minMax, minMax);
      return new Point2D(x, y);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [min; max]. <br>
    * {@code point.y} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param min    the minimum value for each coordinate.
    * @param max    the maximum value for each coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static Point2D nextPoint2D(Random random, double min, double max)
   {
      double x = nextDouble(random, min, max);
      double y = nextDouble(random, min, max);

      return new Point2D(x, y);
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [minX; maxX]. <br>
    * {@code point.y} &in; [minY; maxY]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param minX   the minimum value for the x-coordinate.
    * @param maxX   the maximum value for the x-coordinate.
    * @param minY   the minimum value for the y-coordinate.
    * @param maxY   the maximum value for the y-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minX > maxX} or {@code minY > maxY}.
    */
   public static Point2D nextPoint2D(Random random, double minX, double maxX, double minY, double maxY)
   {
      double x = nextDouble(random, minX, maxX);
      double y = nextDouble(random, minY, maxY);

      return new Point2D(x, y);
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D vector.
    */
   public static Vector2D nextVector2D(Random random)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, vector);
      return vector;
   }

   /**
    * Generates a random unit vector.
    * <p>
    * This generator uses {@link #nextVector2D(Random)}.
    * </p>
    * 
    * @param random the random generator to use.
    * @return the random unit vector.
    */
   public static UnitVector2D nextUnitVector2D(Random random)
   {
      return new UnitVector2D(nextVector2D(random));
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param min    upper-bound for each component of the generated vector. Not modified.
    * @param max    lower-bound for each component of the generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static Vector2D nextVector2D(Random random, double min, double max)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, new Point2D(min, min), new Point2D(max, max), vector);
      return vector;
   }

   /**
    * Generates a random 2D vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param length the length of the generated 2D vector.
    * @return the random 2D vector.
    */
   public static Vector2D nextVector2DWithFixedLength(Random random, double length)
   {
      Vector2D vector = nextVector2D(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *               2D vector. Not modified.
    * @return the random 2D vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static Vector2D nextVector2D(Random random, Tuple2DReadOnly minMax)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, minMax, vector);
      return vector;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min    tuple used as upper-bound for each component of the generated 2D vector. Not
    *               modified.
    * @param max    tuple used as lower-bound for each component of the generated 2D vector. Not
    *               modified.
    * @return the random 2D vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static Vector2D nextVector2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, min, max, vector);
      return vector;
   }

   /**
    * Generates a random 4D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 4D vector.
    */
   public static Vector4D nextVector4D(Random random)
   {
      Vector4D vector = new Vector4D();
      for (int i = 0; i < 4; i++)
         vector.setElement(i, nextDouble(random));
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random vector.
    */
   public static Vector3D32 nextVector3D32(Random random)
   {
      Vector3D32 vector = new Vector3D32();
      randomizeTuple3D(random, vector);
      return vector;
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random point.
    */
   public static Point3D32 nextPoint3D32(Random random)
   {
      Point3D32 point = new Point3D32();
      randomizeTuple3D(random, point);
      return point;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D vector.
    */
   public static Vector2D32 nextVector2D32(Random random)
   {
      Vector2D32 vector = new Vector2D32();
      randomizeTuple2D(random, vector);
      return vector;
   }

   /**
    * Generates a random 2D point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D point.
    */
   public static Point2D32 nextPoint2D32(Random random)
   {
      Point2D32 point = new Point2D32();
      randomizeTuple2D(random, point);
      return point;
   }

   /**
    * Generates a random 4D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 4D vector.
    */
   public static Vector4D32 nextVector4D32(Random random)
   {
      Vector4D32 vector = new Vector4D32();
      for (int i = 0; i < 4; i++)
         vector.setElement(i, nextDouble(random));
      return vector;
   }

   /**
    * Randomizes the given axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random               the random generator to use.
    * @param axisAngleToRandomize the axis-angle to randomize. Modified.
    */
   public static void randomizeAxisAngle(Random random, AxisAngleBasics axisAngleToRandomize)
   {
      randomizeAxisAngle(random, Math.PI, axisAngleToRandomize);
   }

   /**
    * Randomizes the given axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random               the random generator to use.
    * @param minMaxAngle          the maximum absolute angle value.
    * @param axisAngleToRandomize the axis-angle to randomize. Modified.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static void randomizeAxisAngle(Random random, double minMaxAngle, AxisAngleBasics axisAngleToRandomize)
   {
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html )
      double height = 2.0 * random.nextDouble() - 1.0;
      double angle = nextDouble(random, minMaxAngle);
      double radius = EuclidCoreTools.squareRoot(1.0 - height * height);
      axisAngleToRandomize.set(radius * EuclidCoreTools.cos(angle), radius * EuclidCoreTools.sin(angle), height, angle);
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param tupleToRandomize the tuple to randomize. Modified.
    */
   public static void randomizeTuple3D(Random random, Tuple3DBasics tupleToRandomize)
   {
      randomizeTuple3D(random, new Point3D(1.0, 1.0, 1.0), tupleToRandomize);
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random           the random generator to use.
    * @param minMax           tuple used to bound the maximum absolute value of each component of the
    *                         generated vector. Not modified.
    * @param tupleToRandomize the tuple to randomize. Modified.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static void randomizeTuple3D(Random random, Tuple3DReadOnly minMax, Tuple3DBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.setElement(i, nextDouble(random, minMax.getElement(i)));
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random           the random generator to use.
    * @param min              tuple used as upper-bound for each component of the generated vector. Not
    *                         modified.
    * @param max              tuple used as lower-bound for each component of the generated vector. Not
    *                         modified.
    * @param tupleToRandomize the tuple to randomize. Modified.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static void randomizeTuple3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max, Tuple3DBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.setElement(i, nextDouble(random, min.getElement(i), max.getElement(i)));
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    */
   public static void randomizeTuple2D(Random random, Tuple2DBasics tupleToRandomize)
   {
      randomizeTuple2D(random, new Point2D(1.0, 1.0), tupleToRandomize);
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random           the random generator to use.
    * @param minMax           tuple used to bound the maximum absolute value of each component of the
    *                         generated vector. Not modified.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static void randomizeTuple2D(Random random, Tuple2DReadOnly minMax, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.setElement(i, nextDouble(random, minMax.getElement(i)));
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random           the random generator to use.
    * @param min              tuple used as upper-bound for each component of the generated vector. Not
    *                         modified.
    * @param max              tuple used as lower-bound for each component of the generated vector. Not
    *                         modified.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static void randomizeTuple2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.setElement(i, nextDouble(random, min.getElement(i), max.getElement(i)));
   }
}

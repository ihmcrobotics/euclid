package us.ihmc.euclid.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.getAxisAngleString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getMatrix3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getStringFormat;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple2DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple4DString;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * This class provides the tools to perform a variety of assertions on Euclid Core's types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreTestTools
{
   /**
    * Default format used with {@link EuclidCoreIOTools} to build comprehensible feedback when an
    * assertion is failing.
    */
   public static final String DEFAULT_FORMAT = getStringFormat(15, 12);

   private EuclidCoreTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected geometry. Not modified.
    * @param actual   the actual geometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two arguments are not equal or only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two arguments are not equal or only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two arguments are not equal or only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.epsilonEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected geometry. Not modified.
    * @param actual   the actual geometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two arguments do not represent the same geometry or only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two arguments do not represent the same geometry or only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two arguments do not represent the same geometry or only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.geometricallyEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two given angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param expected the expected angle.
    * @param actual   the actual angle.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two angles are not equal.
    */
   public static void assertAngleEquals(double expected, double actual, double epsilon)
   {
      assertAngleEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two given angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected angle.
    * @param actual        the actual angle.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two angles are not equal.
    */
   public static void assertAngleEquals(String messagePrefix, double expected, double actual, double epsilon)
   {
      double differenceAngle = Math.abs(expected - actual);
      differenceAngle = (differenceAngle + Math.PI) % (2.0 * Math.PI) - Math.PI;

      if (Math.abs(differenceAngle) > epsilon)
         throwNotEqualAssertionError(messagePrefix, Double.toString(expected), Double.toString(actual));
   }

   /**
    * Asserts the yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected yaw-pitch-roll orientation. Not modified.
    * @param actual   the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertYawPitchRollGeometricallyEquals(YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon)
   {
      assertOrientation3DGeometricallyEquals(expected, actual, epsilon);
   }

   /**
    * Asserts that the two rotation vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rotation vector. Not modified.
    * @param actual   the actual rotation vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationVectorGeometricallyEquals(Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon)
   {
      assertRotationVectorGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two rotation vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation vector. Not modified.
    * @param actual        the actual rotation vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationVectorGeometricallyEquals(String messagePrefix, Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon)
   {
      assertRotationVectorGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two rotation vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation vector. Not modified.
    * @param actual        the actual rotation vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationVectorGeometricallyEquals(String messagePrefix,
                                                              Vector3DReadOnly expected,
                                                              Vector3DReadOnly actual,
                                                              double epsilon,
                                                              String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      Quaternion expectedQuaternion = new Quaternion(expected);
      Quaternion actualQuaternion = new Quaternion(actual);

      try
      {
         assertOrientation3DGeometricallyEquals(expectedQuaternion, actualQuaternion, epsilon);
      }
      catch (AssertionError e)
      {
         double difference = expectedQuaternion.distance(actualQuaternion);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(difference), format);
      }
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected point. Not modified.
    * @param actual   the actual point. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint2DGeometricallyEquals(Point2DReadOnly expected, Point2DReadOnly actual, double epsilon)
   {
      assertPoint2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected point. Not modified.
    * @param actual        the actual point. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint2DGeometricallyEquals(String messagePrefix, Point2DReadOnly expected, Point2DReadOnly actual, double epsilon)
   {
      assertPoint2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected point. Not modified.
    * @param actual        the actual point. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint2DGeometricallyEquals(String messagePrefix, Point2DReadOnly expected, Point2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(expected.distance(actual)), format);
      }
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected vector. Not modified.
    * @param actual   the actual vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector2DGeometricallyEquals(Vector2DReadOnly expected, Vector2DReadOnly actual, double epsilon)
   {
      assertVector2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector2DGeometricallyEquals(String messagePrefix, Vector2DReadOnly expected, Vector2DReadOnly actual, double epsilon)
   {
      assertVector2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector2DGeometricallyEquals(String messagePrefix, Vector2DReadOnly expected, Vector2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         Vector2D difference = new Vector2D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(difference.norm()), format);
      }
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected point. Not modified.
    * @param actual   the actual point. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint3DGeometricallyEquals(Point3DReadOnly expected, Point3DReadOnly actual, double epsilon)
   {
      assertPoint3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected point. Not modified.
    * @param actual        the actual point. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint3DGeometricallyEquals(String messagePrefix, Point3DReadOnly expected, Point3DReadOnly actual, double epsilon)
   {
      assertPoint3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two points represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected point. Not modified.
    * @param actual        the actual point. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two points do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPoint3DGeometricallyEquals(String messagePrefix, Point3DReadOnly expected, Point3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(expected.distance(actual)), format);
      }
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected vector. Not modified.
    * @param actual   the actual vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector3DGeometricallyEquals(Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon)
   {
      assertVector3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector3DGeometricallyEquals(String messagePrefix, Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon)
   {
      assertVector3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector3DGeometricallyEquals(String messagePrefix, Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         Vector3D difference = new Vector3D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(difference.norm()), format);
      }
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected vector. Not modified.
    * @param actual   the actual vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector4DGeometricallyEquals(Vector4DReadOnly expected, Vector4DReadOnly actual, double epsilon)
   {
      assertVector4DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector4DGeometricallyEquals(String messagePrefix, Vector4DReadOnly expected, Vector4DReadOnly actual, double epsilon)
   {
      assertVector4DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two vectors represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected vector. Not modified.
    * @param actual        the actual vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two vectors do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertVector4DGeometricallyEquals(String messagePrefix, Vector4DReadOnly expected, Vector4DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         Vector4D difference = new Vector4D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(difference.norm()), format);
      }
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected matrix. Not modified.
    * @param actual   the actual matrix. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two matrices are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertMatrix3DEquals(Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      assertMatrix3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected matrix. Not modified.
    * @param actual        the actual matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two matrices are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertMatrix3DEquals(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      assertMatrix3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected matrix. Not modified.
    * @param actual        the actual matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two matrices are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertMatrix3DEquals(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!Matrix3DFeatures.epsilonEquals(expected, actual, epsilon))
      {
         Matrix3D difference = new Matrix3D();
         difference.sub(expected, actual);
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Max difference of: " + Double.toString(difference.maxAbsElement()), format);
      }
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not skew-symmetric. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertSkewSymmetric(Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertSkewSymmetric(null, matrixToAssert, epsilon);
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not skew-symmetric. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertSkewSymmetric(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertSkewSymmetric(messagePrefix, matrixToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @param format         the format to use for printing each component when an
    *                       {@code AssertionError} is thrown.
    * @throws AssertionError if the matrix is not skew-symmetric. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertSkewSymmetric(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon, String format)
   {
      if (matrixToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given matrix is null."));

      if (!matrixToAssert.isMatrixSkewSymmetric(epsilon))
      {
         String errorMessage = "The matrix is not skew-symmetric:\n" + getMatrix3DString(DEFAULT_FORMAT, matrixToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not a rotation matrix. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertRotationMatrix(Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertRotationMatrix(null, matrixToAssert, epsilon);
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not a rotation matrix. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertRotationMatrix(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertRotationMatrix(messagePrefix, matrixToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @param format         the format to use for printing each component when an
    *                       {@code AssertionError} is thrown.
    * @throws AssertionError if the matrix is not a rotation matrix. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertRotationMatrix(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon, String format)
   {
      if (matrixToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given matrix is null."));

      if (!matrixToAssert.isRotationMatrix(epsilon))
      {
         String errorMessage = "This is not a rotation matrix:\n" + getMatrix3DString(format, matrixToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an {@code epsilon}.
    *
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not identity. If the argument is equal to {@code null}.
    */
   public static void assertIdentity(Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertIdentity(null, matrixToAssert, epsilon);
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an {@code epsilon}.
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @throws AssertionError if the matrix is not identity. If the argument is equal to {@code null}.
    */
   public static void assertIdentity(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon)
   {
      assertIdentity(messagePrefix, matrixToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an {@code epsilon}.
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @param epsilon        the tolerance to use.
    * @param format         the format to use for printing each component when an
    *                       {@code AssertionError} is thrown.
    * @throws AssertionError if the matrix is not identity. If the argument is equal to {@code null}.
    */
   public static void assertIdentity(String messagePrefix, Matrix3DReadOnly matrixToAssert, double epsilon, String format)
   {
      if (matrixToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given matrix is null."));

      if (!matrixToAssert.isIdentity(epsilon))
      {
         String errorMessage = "The matrix is not identity:\n" + getMatrix3DString(DEFAULT_FORMAT, matrixToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the given matrix contains on {@link Double#NaN}.
    *
    * @param matrixToAssert the query. Not modified.
    * @throws AssertionError if the matrix does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertMatrix3DContainsOnlyNaN(Matrix3DReadOnly matrixToAssert)
   {
      assertMatrix3DContainsOnlyNaN(null, matrixToAssert);
   }

   /**
    * Asserts that the given matrix contains on {@link Double#NaN}.
    *
    * @param messagePrefix  prefix to add to the error message.
    * @param matrixToAssert the query. Not modified.
    * @throws AssertionError if the matrix does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertMatrix3DContainsOnlyNaN(String messagePrefix, Matrix3DReadOnly matrixToAssert)
   {
      if (matrixToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given matrix is null."));

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (!Double.isNaN(matrixToAssert.getElement(row, column)))
            {
               String errorMessage = "The matrix does not contain only NaN:\n" + getMatrix3DString(DEFAULT_FORMAT, matrixToAssert);
               throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
            }
         }
      }
   }

   /**
    * Check if the two Orientation3D's are geometrically equal. Does NOT indicate that the two are
    * 'equal'.
    *
    * @param expected the orientation3D to be used for comparison. Not modified.
    * @param actual   the orientation3D to be used for comparison. Not modified.
    * @param epsilon  the tolerance to be used in comparison.
    */
   public static void assertOrientation3DGeometricallyEquals(Orientation3DReadOnly expected, Orientation3DReadOnly actual, double epsilon)
   {
      assertOrientation3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Check if the two Orientation3D's are geometrically equal. Does NOT indicate that the two are
    * 'equal'.
    *
    * @param messagePrefix Prefix prefix to add to the error message.
    * @param expected      the orientation3D to be used for comparison. Not modified.
    * @param actual        the orientation3D to be used for comparison. Not modified.
    * @param epsilon       the tolerance to be used in comparison.
    */
   public static void assertOrientation3DGeometricallyEquals(String messagePrefix, Orientation3DReadOnly expected, Orientation3DReadOnly actual, double epsilon)
   {
      assertOrientation3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Check if the two Orientation3D's are geometrically equal. Does NOT indicate that the two are
    * 'equal'.
    *
    * @param messagePrefix Prefix prefix to add to the error message.
    * @param expected      the orientation3D to be used for comparison. Not modified.
    * @param actual        the orientation3D to be used for comparison. Not modified.
    * @param epsilon       the tolerance to be used in comparison.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two orientation 3Ds do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertOrientation3DGeometricallyEquals(String messagePrefix,
                                                             Orientation3DReadOnly expected,
                                                             Orientation3DReadOnly actual,
                                                             double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;
      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, "Difference of: " + Double.toString(expected.distance(actual)), format);
      }

   }

   /**
    * Asserts that the given axis-angle contains only {@link Double#NaN}.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle does not only contain {@link Double#NaN}. If the
    *                        argument is equal to {@code null}.
    */
   public static void assertAxisAngleContainsOnlyNaN(AxisAngleReadOnly axisAngleToAssert)
   {
      assertAxisAngleContainsOnlyNaN(null, axisAngleToAssert);
   }

   /**
    * Asserts that the given axis-angle contains only {@link Double#NaN}.
    *
    * @param messagePrefix     prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle does not only contain {@link Double#NaN}. If the
    *                        argument is equal to {@code null}.
    */
   public static void assertAxisAngleContainsOnlyNaN(String messagePrefix, AxisAngleReadOnly axisAngleToAssert)
   {
      if (axisAngleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given axis-angle is null."));

      for (int index = 0; index < 4; index++)
      {
         if (!Double.isNaN(axisAngleToAssert.getElement(index)))
         {
            String errorMessage = "The axis-angle does not contain only NaN:\n" + getAxisAngleString(DEFAULT_FORMAT, axisAngleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link AxisAngleBasics#setToZero()} has just been called on the given axis-angle.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertAxisAngleIsSetToZero(AxisAngleReadOnly axisAngleToAssert)
   {
      assertAxisAngleIsSetToZero(null, axisAngleToAssert);
   }

   /**
    * Assert that {@link AxisAngleBasics#setToZero()} has just been called on the given axis-angle.
    *
    * @param messagePrefix     prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertAxisAngleIsSetToZero(String messagePrefix, AxisAngleReadOnly axisAngleToAssert)
   {
      if (axisAngleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given axis-angle is null."));

      AxisAngle expected = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      if (!expected.equals(axisAngleToAssert))
      {
         String errorMessage = "The axis-angle has not been set to zero:\n" + getAxisAngleString(DEFAULT_FORMAT, axisAngleToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsSetToZero(QuaternionReadOnly quaternionToAssert)
   {
      assertQuaternionIsSetToZero(null, quaternionToAssert);
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsSetToZero(String messagePrefix, QuaternionReadOnly quaternionToAssert)
   {
      assertQuaternionIsSetToZero(messagePrefix, quaternionToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param format             the format to use for printing each component when an
    *                           {@code AssertionError} is thrown.
    * @throws AssertionError if the quaternion has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsSetToZero(String messagePrefix, QuaternionReadOnly quaternionToAssert, String format)
   {
      if (quaternionToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given quaternion is null."));

      Quaternion expected = new Quaternion(0.0, 0.0, 0.0, 1.0);

      if (!expected.equals(quaternionToAssert))
      {
         String errorMessage = "The axis-angle has not been set to zero:\n" + getTuple4DString(format, quaternionToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon            the tolerance to use.
    * @throws AssertionError if the quaternion is not a unit-quaternion. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsUnitary(QuaternionReadOnly quaternionToAssert, double epsilon)
   {
      assertQuaternionIsUnitary(null, quaternionToAssert, epsilon);
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon            the tolerance to use.
    * @throws AssertionError if the quaternion is not a unit-quaternion. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsUnitary(String messagePrefix, QuaternionReadOnly quaternionToAssert, double epsilon)
   {
      assertQuaternionIsUnitary(messagePrefix, quaternionToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon            the tolerance to use.
    * @param format             the format to use for printing each component when an
    *                           {@code AssertionError} is thrown.
    * @throws AssertionError if the quaternion is not a unit-quaternion. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertQuaternionIsUnitary(String messagePrefix, QuaternionReadOnly quaternionToAssert, double epsilon, String format)
   {
      if (quaternionToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given quaternion is null."));

      if (!quaternionToAssert.isUnitary(epsilon))
      {
         String errorMessage = "The quaternion is not unitary: " + getTuple4DString(format, quaternionToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertTuple2DContainsOnlyNaN(Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DContainsOnlyNaN(null, tupleToAssert);
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertTuple2DContainsOnlyNaN(String messagePrefix, Tuple2DReadOnly tupleToAssert)
   {
      if (tupleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given tuple is null."));

      for (int index = 0; index < 2; index++)
      {
         if (!Double.isNaN(tupleToAssert.getElement(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple2DString(DEFAULT_FORMAT, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DIsSetToZero(Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DIsSetToZero(null, tupleToAssert);
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DIsSetToZero(String messagePrefix, Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DIsSetToZero(messagePrefix, tupleToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DIsSetToZero(String messagePrefix, Tuple2DReadOnly tupleToAssert, String format)
   {
      if (tupleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given tuple is null."));

      for (int index = 0; index < 2; index++)
      {
         if (tupleToAssert.getElement(index) != 0.0)
         {
            String errorMessage = "The tuple has not been set to zero:\n " + getTuple2DString(format, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertTuple3DContainsOnlyNaN(Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DContainsOnlyNaN(null, tupleToAssert);
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}. If the argument is
    *                        equal to {@code null}.
    */
   public static void assertTuple3DContainsOnlyNaN(String messagePrefix, Tuple3DReadOnly tupleToAssert)
   {
      if (tupleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given tuple is null."));

      for (int index = 0; index < 3; index++)
      {
         if (!Double.isNaN(tupleToAssert.getElement(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple3DString(DEFAULT_FORMAT, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DIsSetToZero(Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DIsSetToZero(null, tupleToAssert);
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DIsSetToZero(String messagePrefix, Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DIsSetToZero(messagePrefix, tupleToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the tuple has not been set to zero. If the argument is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DIsSetToZero(String messagePrefix, Tuple3DReadOnly tupleToAssert, String format)
   {
      if (tupleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given tuple is null."));

      for (int index = 0; index < 3; index++)
      {
         if (tupleToAssert.getElement(index) != 0.0)
         {
            String errorMessage = "The tuple has not been set to zero:\n " + getTuple3DString(format, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the quaternion does not only contain {@link Double#NaN}. If the
    *                        argument is equal to {@code null}.
    */
   public static void assertTuple4DContainsOnlyNaN(Tuple4DReadOnly tupleToAssert)
   {
      assertTuple4DContainsOnlyNaN(null, tupleToAssert);
   }

   /**
    * Asserts that the given quaternion contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the quaternion does not only contain {@link Double#NaN}. If the
    *                        argument is equal to {@code null}.
    */
   public static void assertTuple4DContainsOnlyNaN(String messagePrefix, Tuple4DReadOnly tupleToAssert)
   {
      if (tupleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given tuple is null."));

      for (int index = 0; index < 4; index++)
      {
         if (!Double.isNaN(tupleToAssert.getElement(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple4DString(DEFAULT_FORMAT, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts that when executing the given runnable, an specific exception is thrown.
    *
    * @param runnable                 the code to be executed and to be throwing an exception.
    * @param acceptableExceptionTypes the different types of acceptable exception to the runnable to
    *                                 throw.
    * @throws AssertionError if the no exception is thrown, if the type of the thrown exception is not
    *                        contained in {@code accepetableExceptionType}, or if
    *                        {@code expectedMessageContent} is not {@code null} and the detail message
    *                        is different.
    */
   public static void assertExceptionIsThrown(Runnable runnable, Class<?>... acceptableExceptionTypes)
   {
      assertExceptionIsThrown(null, runnable, acceptableExceptionTypes);
   }

   /**
    * Asserts that when executing the given runnable, an specific exception is thrown.
    *
    * @param messagePrefix            prefix to add to the error message.
    * @param runnable                 the code to be executed and to be throwing an exception.
    * @param acceptableExceptionTypes the different types of acceptable exception to the runnable to
    *                                 throw.
    * @throws AssertionError if the no exception is thrown, if the type of the thrown exception is not
    *                        contained in {@code accepetableExceptionType}, or if
    *                        {@code expectedMessageContent} is not {@code null} and the detail message
    *                        is different.
    */
   public static void assertExceptionIsThrown(String messagePrefix, Runnable runnable, Class<?>... acceptableExceptionTypes)
   {
      assertExceptionIsThrown(messagePrefix, runnable, null, acceptableExceptionTypes);
   }

   /**
    * Asserts that when executing the given runnable, an specific exception is thrown.
    *
    * @param runnable                 the code to be executed and to be throwing an exception.
    * @param expectedMessageContent   the detail message the thrown should be carrying. The detail
    *                                 message is not tested when the argument is {@code null}.
    * @param acceptableExceptionTypes the different types of acceptable exception to the runnable to
    *                                 throw.
    * @throws AssertionError if the no exception is thrown, if the type of the thrown exception is not
    *                        contained in {@code accepetableExceptionType}, or if
    *                        {@code expectedMessageContent} is not {@code null} and the detail message
    *                        is different.
    */
   public static void assertExceptionIsThrown(Runnable runnable, String expectedMessageContent, Class<?>... acceptableExceptionTypes)
   {
      assertExceptionIsThrown(null, runnable, expectedMessageContent, acceptableExceptionTypes);
   }

   /**
    * Asserts that when executing the given runnable, an specific exception is thrown.
    *
    * @param messagePrefix            prefix to add to the error message.
    * @param runnable                 the code to be executed and to be throwing an exception.
    * @param expectedMessageContent   the detail message the thrown should be carrying. The detail
    *                                 message is not tested when the argument is {@code null}.
    * @param acceptableExceptionTypes the different types of acceptable exception to the runnable to
    *                                 throw.
    * @throws AssertionError if the no exception is thrown, if the type of the thrown exception is not
    *                        contained in {@code accepetableExceptionType}, or if
    *                        {@code expectedMessageContent} is not {@code null} and the detail message
    *                        is different.
    */
   public static void assertExceptionIsThrown(String messagePrefix, Runnable runnable, String expectedMessageContent, Class<?>... acceptableExceptionTypes)
   {
      Exception exceptionCaught = null;

      try
      {
         runnable.run();
      }
      catch (Exception e)
      {
         exceptionCaught = e;
      }
      finally
      {
         if (exceptionCaught == null)
            throw new AssertionError(addPrefixToMessage(messagePrefix, "The operation should have thrown an exception."));

         boolean isExceptionUnexpected = true;
         for (Class<?> expectedExceptionType : acceptableExceptionTypes)
         {
            if (exceptionCaught.getClass().equals(expectedExceptionType))
            {
               isExceptionUnexpected = false;
            }
         }

         if (isExceptionUnexpected)
         {
            List<String> expectedExceptionSimpleNames = Stream.of(acceptableExceptionTypes).map(e -> e.getSimpleName()).collect(Collectors.toList());
            throw new AssertionError(addPrefixToMessage(messagePrefix,
                                                        "Unexpected exception: expected any of " + expectedExceptionSimpleNames + ", actual = "
                                                              + exceptionCaught.getClass().getSimpleName()));
         }

         if (expectedMessageContent != null && !expectedMessageContent.equals(exceptionCaught.getMessage()))
         {
            throw new AssertionError(addPrefixToMessage(messagePrefix,
                                                        "Unexpected exception message: expected " + expectedMessageContent + ", actual = "
                                                              + exceptionCaught.getMessage()));
         }
      }
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expected.toString(format)
    * but was:
    * actual.toString(format)
    * </pre>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the result that was expected.
    * @param actual        the result that was obtained.
    * @param format        the format to use for each number.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, null, format);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expected.toString(format)
    * but was:
    * actual.toString(format)
    * Difference of: differenceAsString
    * </pre>
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param expected           the result that was expected.
    * @param actual             the result that was obtained.
    * @param differenceAsString a short comprehensible summary of the difference between the expected
    *                           and obtained results.
    * @param format             the format to use for each number.
    */
   public static void throwNotEqualAssertionError(String messagePrefix,
                                                  EuclidGeometry expected,
                                                  EuclidGeometry actual,
                                                  String differenceAsString,
                                                  String format)
   {
      String expectedAsString = expected == null ? "null" : expected.toString(format);
      String actualAsString = actual == null ? "null" : actual.toString(format);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * </pre>
    *
    * @param messagePrefix    prefix to add to the error message.
    * @param expectedAsString the result that was expected in a {@code String} form.
    * @param actualAsString   the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * Difference of: differenceAsString
    * </pre>
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param expectedAsString   the result that was expected in a {@code String} form.
    * @param actualAsString     the result that was obtained in a {@code String} form.
    * @param differenceAsString a short comprehensible summary of the difference between the expected
    *                           and obtained results.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\n" + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix errorMessage
    * </pre>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param errorMessage  the detail message of why the assertion is failing.
    */
   public static void throwAssertionError(String messagePrefix, String errorMessage)
   {
      throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
   }

   /**
    * Convenience method for prepending an optional prefix to a message.
    * <p>
    * In the case the given {@code prefix} is {@code null} or empty, the original {@code message} is
    * returned.
    * </p>
    *
    * @param prefix  the {@code String} to prepend to the message.
    * @param message the original message.
    * @return the message with the prefix.
    */
   public static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }
}

package us.ihmc.euclid.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual   the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal. If only one of the
    *                        arguments is equal to {@code null}. If at least one of the arguments has a
    *                        length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollEquals(YawPitchRollReadOnly, YawPitchRollReadOnly, double)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollEquals(double[] expected, double[] actual, double epsilon)
   {
      assertYawPitchRollEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual        the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal. If only one of the
    *                        arguments is equal to {@code null}. If at least one of the arguments has a
    *                        length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollEquals(String, YawPitchRollReadOnly, YawPitchRollReadOnly, double)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollEquals(String messagePrefix, double[] expected, double[] actual, double epsilon)
   {
      assertYawPitchRollEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual        the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal. If only one of the
    *                        arguments is equal to {@code null}. If at least one of the arguments has a
    *                        length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollEquals(String, YawPitchRollReadOnly, YawPitchRollReadOnly, double, String)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollEquals(String messagePrefix, double[] expected, double[] actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, Arrays.toString(expected), Arrays.toString(actual));

      if (expected.length != 3)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "Unexpected size for the expected argument: " + expected.length));

      if (actual.length != 3)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "Unexpected size for the actual argument: " + actual.length));

      try
      {
         assertAngleEquals(expected[0], actual[0], epsilon);
         assertAngleEquals(expected[1], actual[1], epsilon);
         assertAngleEquals(expected[2], actual[2], epsilon);
      }
      catch (AssertionError e)
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the yaw-pitch-roll orientations are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected yaw-pitch-roll orientation. Not modified.
    * @param actual   the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertYawPitchRollEquals(YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon)
   {
      assertYawPitchRollEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the yaw-pitch-roll orientations are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertYawPitchRollEquals(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon)
   {
      assertYawPitchRollEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the yaw-pitch-roll orientations are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertYawPitchRollEquals(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles represent the same
    * geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual   the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles do not represent the same
    *                        geometry. If only one of the arguments is equal to {@code null}. If at
    *                        least one of the arguments has a length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollGeometricallyEquals(YawPitchRollReadOnly, YawPitchRollReadOnly, double)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollGeometricallyEquals(double[] expected, double[] actual, double epsilon)
   {
      assertYawPitchRollGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles represent the same
    * geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual        the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles do not represent the same
    *                        geometry. If only one of the arguments is equal to {@code null}. If at
    *                        least one of the arguments has a length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollGeometricallyEquals(String, YawPitchRollReadOnly, YawPitchRollReadOnly, double)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollGeometricallyEquals(String messagePrefix, double[] expected, double[] actual, double epsilon)
   {
      assertYawPitchRollGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles represent the same
    * geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected set of yaw-pitch-roll angles. Not modified.
    * @param actual        the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles do not represent the same
    *                        geometry. If only one of the arguments is equal to {@code null}. If at
    *                        least one of the arguments has a length different than 3.
    * @deprecated Use
    *             {@link #assertYawPitchRollGeometricallyEquals(String, YawPitchRollReadOnly, YawPitchRollReadOnly, double, String)}
    *             instead.
    */
   @Deprecated
   public static void assertYawPitchRollGeometricallyEquals(String messagePrefix, double[] expected, double[] actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, Arrays.toString(expected), Arrays.toString(actual));

      if (expected.length != 3)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "Unexpected size for the expected argument: " + expected.length));

      if (actual.length != 3)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "Unexpected size for the actual argument: " + actual.length));

      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();
      expectedQuaternion.setYawPitchRoll(expected);
      actualQuaternion.setYawPitchRoll(actual);

      try
      {
         assertQuaternionGeometricallyEquals(messagePrefix, expectedQuaternion, actualQuaternion, epsilon, format);
      }
      catch (AssertionError e)
      {
         double difference = expectedQuaternion.distance(actualQuaternion);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
      }
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
      assertYawPitchRollGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertYawPitchRollGeometricallyEquals(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon)
   {
      assertYawPitchRollGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertYawPitchRollGeometricallyEquals(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double epsilon,
                                                            String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         double difference = expected.distance(actual);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
      }
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
   public static void assertRotationVectorGeometricallyEquals(String messagePrefix, Vector3DReadOnly expected, Vector3DReadOnly actual, double epsilon,
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
         assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, epsilon);
      }
      catch (AssertionError e)
      {
         double difference = expectedQuaternion.distance(actualQuaternion);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected tuple. Not modified.
    * @param actual   the actual tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DEquals(Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      assertTuple2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DEquals(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      assertTuple2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple2DEquals(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector2D difference = new Vector2D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, expected.distance(actual), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected tuple. Not modified.
    * @param actual   the actual tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DEquals(Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      assertTuple3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DEquals(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      assertTuple3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple3DEquals(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector3D difference = new Vector3D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, expected.distance(actual), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected tuple. Not modified.
    * @param actual   the actual tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple4DEquals(Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple4DEquals(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected tuple. Not modified.
    * @param actual        the actual tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tuples are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTuple4DEquals(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector4D difference = new Vector4D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.norm(), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.norm(), format);
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
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two rotation matrices represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rotation matrix. Not modified.
    * @param actual   the actual rotation matrix. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rotation matrices do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationMatrixGeometricallyEquals(RotationMatrixReadOnly expected, RotationMatrixReadOnly actual, double epsilon)
   {
      assertRotationMatrixGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two rotation matrices represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation matrix. Not modified.
    * @param actual        the actual rotation matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rotation matrices do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationMatrixGeometricallyEquals(String messagePrefix, RotationMatrixReadOnly expected, RotationMatrixReadOnly actual,
                                                              double epsilon)
   {
      assertRotationMatrixGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two rotation matrices represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation matrix. Not modified.
    * @param actual        the actual rotation matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two rotation matrices do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertRotationMatrixGeometricallyEquals(String messagePrefix, RotationMatrixReadOnly expected, RotationMatrixReadOnly actual,
                                                              double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, expected.distance(actual), format);
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
         String errorMessage = "The matrix is not skew-symmetric:\n" + getMatrixString(DEFAULT_FORMAT, matrixToAssert);
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
         String errorMessage = "This is not a rotation matrix:\n" + getMatrixString(format, matrixToAssert);
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
         String errorMessage = "The matrix is not identity:\n" + getMatrixString(DEFAULT_FORMAT, matrixToAssert);
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
               String errorMessage = "The matrix does not contain only NaN:\n" + getMatrixString(DEFAULT_FORMAT, matrixToAssert);
               throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
            }
         }
      }
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected quaternion. Not modified.
    * @param actual   the actual quaternion. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertQuaternionEquals(QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion. Not modified.
    * @param actual        the actual quaternion. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertQuaternionEquals(String messagePrefix, QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(messagePrefix, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion. Not modified.
    * @param actual        the actual quaternion. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two quaternions are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertQuaternionEquals(String messagePrefix, QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon, String format)
   {
      assertTuple4DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts that the two quaternions represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected quaternion. Not modified.
    * @param actual   the actual quaternion. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertQuaternionGeometricallyEquals(QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon)
   {
      assertQuaternionGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two quaternions represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion. Not modified.
    * @param actual        the actual quaternion. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertQuaternionGeometricallyEquals(String messagePrefix, QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon)
   {
      assertQuaternionGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two quaternions represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion. Not modified.
    * @param actual        the actual quaternion. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two quaternions do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertQuaternionGeometricallyEquals(String messagePrefix, QuaternionReadOnly expected, QuaternionReadOnly actual, double epsilon,
                                                          String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         double difference = expected.distance(actual);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
      }
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected axis-angle. Not modified.
    * @param actual   the actual axis-angle. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertAxisAngleEquals(AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon)
   {
      assertAxisAngleEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected axis-angle. Not modified.
    * @param actual        the actual axis-angle. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertAxisAngleEquals(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon)
   {
      assertAxisAngleEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected axis-angle. Not modified.
    * @param actual        the actual axis-angle. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two axis-angles are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertAxisAngleEquals(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts that the two axis-angles represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected axis-angle. Not modified.
    * @param actual   the actual axis-angle. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two axis-angles do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertAxisAngleGeometricallyEquals(AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon)
   {
      assertAxisAngleGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two axis-angles represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected axis-angle. Not modified.
    * @param actual        the actual axis-angle. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two axis-angles do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertAxisAngleGeometricallyEquals(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon)
   {
      assertAxisAngleGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two axis-angles represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected axis-angle. Not modified.
    * @param actual        the actual axis-angle. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two axis-angles do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertAxisAngleGeometricallyEquals(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, double epsilon,
                                                         String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         double difference = expected.distance(actual);
         difference = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(difference));
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
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
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon           the tolerance to use.
    * @throws AssertionError if the axis is not unitary. If the argument is equal to {@code null}.
    */
   public static void assertAxisUnitary(AxisAngleReadOnly axisAngleToAssert, double epsilon)
   {
      assertAxisUnitary(null, axisAngleToAssert, epsilon);
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix     prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon           the tolerance to use.
    * @throws AssertionError if the axis is not unitary. If the argument is equal to {@code null}.
    */
   public static void assertAxisUnitary(String messagePrefix, AxisAngleReadOnly axisAngleToAssert, double epsilon)
   {
      assertAxisUnitary(messagePrefix, axisAngleToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix     prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon           the tolerance to use.
    * @param format            the format to use for printing each component when an
    *                          {@code AssertionError} is thrown.
    * @throws AssertionError if the axis is not unitary. If the argument is equal to {@code null}.
    */
   public static void assertAxisUnitary(String messagePrefix, AxisAngleReadOnly axisAngleToAssert, double epsilon, String format)
   {
      if (axisAngleToAssert == null)
         throw new AssertionError(addPrefixToMessage(messagePrefix, "The given axis-angle is null."));

      if (!axisAngleToAssert.isAxisUnitary(epsilon))
      {
         String errorMessage = "The axis of the given axis-angle is not unitary: " + getAxisAngleString(format, axisAngleToAssert);
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
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual   the actual rigid-body transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rigid-body transform. Not modified.
    * @param actual        the actual rigid-body transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rigid-body transform. Not modified.
    * @param actual        the actual rigid-body transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two rigid-body transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon,
                                                     String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two given rigid-body transform represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual   the actual rigid-body transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms do not represent the same geometry. If
    *                        only one of the arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformGeometricallyEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two given rigid-body transform represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rigid-body transform. Not modified.
    * @param actual        the actual rigid-body transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms do not represent the same geometry. If
    *                        only one of the arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformGeometricallyEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two given rigid-body transform represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rigid-body transform. Not modified.
    * @param actual        the actual rigid-body transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two rigid-body transforms do not represent the same geometry. If
    *                        only one of the arguments is equal to {@code null}.
    */
   public static void assertRigidBodyTransformGeometricallyEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon,
                                                                  String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual   the actual quaternion-based transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformEquals(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      assertQuaternionBasedTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion-based transform. Not modified.
    * @param actual        the actual quaternion-based transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformEquals(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                           double epsilon)
   {
      assertQuaternionBasedTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion-based transform. Not modified.
    * @param actual        the actual quaternion-based transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two quaternion-based transforms are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformEquals(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                           double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two quaternion-based transforms represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual   the actual quaternion-based transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms do not represent the same geometry.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformGeometricallyEquals(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      assertQuaternionBasedTransformGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two quaternion-based transforms represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion-based transform. Not modified.
    * @param actual        the actual quaternion-based transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms do not represent the same geometry.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformGeometricallyEquals(String messagePrefix, QuaternionBasedTransform expected,
                                                                        QuaternionBasedTransform actual, double epsilon)
   {
      assertQuaternionBasedTransformGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two quaternion-based transforms represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected quaternion-based transform. Not modified.
    * @param actual        the actual quaternion-based transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two quaternion-based transforms do not represent the same geometry.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertQuaternionBasedTransformGeometricallyEquals(String messagePrefix, QuaternionBasedTransform expected,
                                                                        QuaternionBasedTransform actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected affine transform. Not modified.
    * @param actual   the actual affine transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two affine transforms are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertAffineTransformEquals(AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected affine transform. Not modified.
    * @param actual        the actual affine transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two affine transforms are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertAffineTransformEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected affine transform. Not modified.
    * @param actual        the actual affine transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two affine transforms are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertAffineTransformEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two given affine transforms represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected affine transform. Not modified.
    * @param actual   the actual affine transform. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two affine transforms do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertAffineTransformGeometricallyEquals(AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two given affine transforms represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected affine transform. Not modified.
    * @param actual        the actual affine transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two affine transforms do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertAffineTransformGeometricallyEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two given affine transforms represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected affine transform. Not modified.
    * @param actual        the actual affine transform. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two affine transforms do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertAffineTransformGeometricallyEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon,
                                                               String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
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

   private static void throwNotEqualAssertionError(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, String format)
   {
      String expectedAsString = getYawPitchRollString(format, expected);
      String actualAsString = getYawPitchRollString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, YawPitchRollReadOnly expected, YawPitchRollReadOnly actual, double difference,
                                                   String format)
   {
      String expectedAsString = getYawPitchRollString(format, expected);
      String actualAsString = getYawPitchRollString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, String format)
   {
      String expectedAsString = getTuple2DString(format, expected);
      String actualAsString = getTuple2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple2DString(format, expected);
      String actualAsString = getTuple2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, String format)
   {
      String expectedAsString = getTuple3DString(format, expected);
      String actualAsString = getTuple3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple3DString(format, expected);
      String actualAsString = getTuple3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, String format)
   {
      String expectedAsString = getTuple4DString(format, expected);
      String actualAsString = getTuple4DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple4DString(format, expected);
      String actualAsString = getTuple4DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, String format)
   {
      String expectedAsString = getAxisAngleString(format, expected);
      String actualAsString = getAxisAngleString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, double difference, String format)
   {
      String expectedAsString = getAxisAngleString(format, expected);
      String actualAsString = getAxisAngleString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, String format)
   {
      String expectedAsString = getMatrixString(format, expected);
      String actualAsString = getMatrixString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getMatrixString(format, expected);
      String actualAsString = getMatrixString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, AffineTransform expected, AffineTransform actual, String format)
   {
      String expectedAsString = getAffineTransformString(format, expected);
      String actualAsString = getAffineTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, String format)
   {
      String expectedAsString = getRigidBodyTransformString(format, expected);
      String actualAsString = getRigidBodyTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual, String format)
   {
      String expectedAsString = getQuaternionBasedTransformString(format, expected);
      String actualAsString = getQuaternionBasedTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, double[] expected, double[] actual, String format)
   {
      String expectedAsString = getStringOf("[", "]", ",", format, expected);
      String actualAsString = getStringOf("[", "]", ",", format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, double[] expected, double[] actual, double difference, String format)
   {
      String expectedAsString = getStringOf("[", "]", ",", format, expected);
      String actualAsString = getStringOf("[", "]", ",", format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
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
    * @param differenceAsString a short comprehensible summary of the difference between the expected
    *                           and obtained results.
    * @param actualAsString     the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\nDifference of: " + differenceAsString;

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

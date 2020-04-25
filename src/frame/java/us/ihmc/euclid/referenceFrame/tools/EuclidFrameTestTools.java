package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

/**
 * This class provides the tools to perform a variety of assertions on frame geometry types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidFrameTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two rotation frame vectors represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rotation frame vector. Not modified.
    * @param actual   the actual rotation frame vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertRotationFrameVectorGeometricallyEquals(FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon)
   {
      assertRotationFrameVectorGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two rotation frame vectors represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation frame vector. Not modified.
    * @param actual        the actual rotation frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertRotationFrameVectorGeometricallyEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual,
                                                                   double epsilon)
   {
      assertRotationFrameVectorGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two rotation frame vectors represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation frame vector. Not modified.
    * @param actual        the actual rotation frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two rotation vectors do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertRotationFrameVectorGeometricallyEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual,
                                                                   double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(messagePrefix, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the frame yaw-pitch-roll orientations are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual   the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal or not expressed in the same
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameYawPitchRollEquals(FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual, double epsilon)
   {
      assertFrameYawPitchRollEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the frame yaw-pitch-roll orientations are equal to an
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
    * @param expected      the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal or not expressed in the same
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameYawPitchRollEquals(String messagePrefix, FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual, double epsilon)
   {
      assertFrameYawPitchRollEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the frame yaw-pitch-roll orientations are equal to an
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
    * @param expected      the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two yaw-pitch-rolls are not equal or not expressed in the same
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameYawPitchRollEquals(String messagePrefix, FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual, double epsilon,
                                                    String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertYawPitchRollEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts the frame yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual   the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameYawPitchRollGeometricallyEquals(FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual, double epsilon)
   {
      assertFrameYawPitchRollGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the frame yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameYawPitchRollGeometricallyEquals(String messagePrefix, FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual,
                                                                 double epsilon)
   {
      assertFrameYawPitchRollGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the frame yaw-pitch-roll orientations are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame yaw-pitch-roll orientation. Not modified.
    * @param actual        the actual frame yaw-pitch-roll orientation. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two yaw-pitch-roll do not represent the same geometry or not
    *                        expressed in the same reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameYawPitchRollGeometricallyEquals(String messagePrefix, FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual,
                                                                 double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple. Not modified.
    * @param actual   the actual frame tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon)
   {
      assertFrameTuple2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon)
   {
      assertFrameTuple2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertTuple2DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame point. Not modified.
    * @param actual   the actual frame point. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint2DGeometricallyEquals(FramePoint2DReadOnly expected, FramePoint2DReadOnly actual, double epsilon)
   {
      assertFramePoint2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame point. Not modified.
    * @param actual        the actual frame point. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint2DGeometricallyEquals(String messagePrefix, FramePoint2DReadOnly expected, FramePoint2DReadOnly actual, double epsilon)
   {
      assertFramePoint2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame point. Not modified.
    * @param actual        the actual frame point. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint2DGeometricallyEquals(String messagePrefix, FramePoint2DReadOnly expected, FramePoint2DReadOnly actual, double epsilon,
                                                            String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame vector. Not modified.
    * @param actual   the actual frame vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector2DGeometricallyEquals(FrameVector2DReadOnly expected, FrameVector2DReadOnly actual, double epsilon)
   {
      assertFrameVector2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame vector. Not modified.
    * @param actual        the actual frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector2DGeometricallyEquals(String messagePrefix, FrameVector2DReadOnly expected, FrameVector2DReadOnly actual, double epsilon)
   {
      assertFrameVector2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame vector. Not modified.
    * @param actual        the actual frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector2DGeometricallyEquals(String messagePrefix, FrameVector2DReadOnly expected, FrameVector2DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertVector2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple. Not modified.
    * @param actual   the actual frame tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon)
   {
      assertFrameTuple3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon)
   {
      assertFrameTuple3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertTuple3DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame point. Not modified.
    * @param actual   the actual frame point. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint3DGeometricallyEquals(FramePoint3DReadOnly expected, FramePoint3DReadOnly actual, double epsilon)
   {
      assertFramePoint3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame point. Not modified.
    * @param actual        the actual frame point. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint3DGeometricallyEquals(String messagePrefix, FramePoint3DReadOnly expected, FramePoint3DReadOnly actual, double epsilon)
   {
      assertFramePoint3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame points represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame point. Not modified.
    * @param actual        the actual frame point. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame points do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFramePoint3DGeometricallyEquals(String messagePrefix, FramePoint3DReadOnly expected, FramePoint3DReadOnly actual, double epsilon,
                                                            String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame vector. Not modified.
    * @param actual   the actual frame vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector3DGeometricallyEquals(FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon)
   {
      assertFrameVector3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame vector. Not modified.
    * @param actual        the actual frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector3DGeometricallyEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon)
   {
      assertFrameVector3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected frame vector. Not modified.
    * @param actual        the actual frame vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame vectors do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector3DGeometricallyEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple. Not modified.
    * @param actual   the actual frame tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon)
   {
      assertFrameTuple4DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon)
   {
      assertFrameTuple4DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertTuple4DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple. Not modified.
    * @param actual   the actual frame tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry or not expressed in
    *                        the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector4DGeometricallyEquals(FrameVector4DReadOnly expected, FrameVector4DReadOnly actual, double epsilon)
   {
      assertFrameVector4DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two vectors do not represent the same geometry or not expressed in
    *                        the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector4DGeometricallyEquals(String messagePrefix, FrameVector4DReadOnly expected, FrameVector4DReadOnly actual, double epsilon)
   {
      assertFrameVector4DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame vectors represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two vectors do not represent the same geometry or not expressed in
    *                        the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameVector4DGeometricallyEquals(String messagePrefix, FrameVector4DReadOnly expected, FrameVector4DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertVector4DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame quaternions represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple. Not modified.
    * @param actual   the actual frame tuple. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same geometry or not expressed
    *                        in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameQuaternionGeometricallyEquals(FrameQuaternionReadOnly expected, FrameQuaternionReadOnly actual, double epsilon)
   {
      assertFrameQuaternionGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame quaternions represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same geometry or not expressed
    *                        in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameQuaternionGeometricallyEquals(String messagePrefix, FrameQuaternionReadOnly expected, FrameQuaternionReadOnly actual,
                                                               double epsilon)
   {
      assertFrameQuaternionGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame quaternions represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame tuple. Not modified.
    * @param actual        the actual frame tuple. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two quaternions do not represent the same geometry or not expressed
    *                        in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameQuaternionGeometricallyEquals(String messagePrefix, FrameQuaternionReadOnly expected, FrameQuaternionReadOnly actual,
                                                               double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame orientation 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected orientation 2D. Not modified.
    * @param actual   the actual orientation 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameOrientation2DEquals(FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual, double epsilon)
   {
      assertFrameOrientation2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame orientation 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameOrientation2DEquals(String messagePrefix, FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual,
                                                     double epsilon)
   {
      assertFrameOrientation2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame orientation 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two orientation 2Ds are not equal or not expressed in the reference
    *                        frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameOrientation2DEquals(String messagePrefix, FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual,
                                                     double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertOrientation2DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts that the two frame orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected orientation 2D. Not modified.
    * @param actual   the actual orientation 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameOrientation2DGeometricallyEquals(FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual, double epsilon)
   {
      assertFrameOrientation2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two frame orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameOrientation2DGeometricallyEquals(String messagePrefix, FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual,
                                                                  double epsilon)
   {
      assertFrameOrientation2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two frame orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameOrientation2DGeometricallyEquals(String messagePrefix, FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual,
                                                                  double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertOrientation2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame matrices are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected matrix. Not modified.
    * @param actual   the actual matrix. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two matrices are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameMatrix3DEquals(FrameMatrix3DReadOnly expected, FrameMatrix3DReadOnly actual, double epsilon)
   {
      assertFrameMatrix3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame matrices are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected matrix. Not modified.
    * @param actual        the actual matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two matrices are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameMatrix3DEquals(String messagePrefix, FrameMatrix3DReadOnly expected, FrameMatrix3DReadOnly actual, double epsilon)
   {
      assertFrameMatrix3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame matrices are equal to an {@code epsilon}.
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
    * @throws AssertionError if the two matrices are not equal or not expressed in the reference frame.
    *                        If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameMatrix3DEquals(String messagePrefix, FrameMatrix3DReadOnly expected, FrameMatrix3DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts that the two frame rotation matrices represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rotation matrix. Not modified.
    * @param actual   the actual rotation matrix. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two rotation matrices do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRotationMatrixGeometricallyEquals(FrameRotationMatrixReadOnly expected, FrameRotationMatrixReadOnly actual, double epsilon)
   {
      assertFrameRotationMatrixGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two frame rotation matrices represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected rotation matrix. Not modified.
    * @param actual        the actual rotation matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two rotation matrices do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRotationMatrixGeometricallyEquals(String messagePrefix, FrameRotationMatrixReadOnly expected,
                                                                   FrameRotationMatrixReadOnly actual, double epsilon)
   {
      assertFrameRotationMatrixGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two frame rotation matrices represent the same geometry to an {@code epsilon}.
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
    * @throws AssertionError if the two rotation matrices do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRotationMatrixGeometricallyEquals(String messagePrefix, FrameRotationMatrixReadOnly expected,
                                                                   FrameRotationMatrixReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 2D. Not modified.
    * @param actual   the actual bounding box 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox2DEquals(FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual, double epsilon)
   {
      assertFrameBoundingBox2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox2DEquals(String messagePrefix, FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual,
                                                     double epsilon)
   {
      assertFrameBoundingBox2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 2Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox2DEquals(String messagePrefix, FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual,
                                                     double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertBoundingBox2DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 2D. Not modified.
    * @param actual   the actual bounding box 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox2DGeometricallyEquals(FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual, double epsilon)
   {
      assertFrameBoundingBox2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox2DGeometricallyEquals(String messagePrefix, FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual,
                                                                  double epsilon)
   {
      assertFrameBoundingBox2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 2Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox2DGeometricallyEquals(String messagePrefix, FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual,
                                                                  double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertBoundingBox2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 3D. Not modified.
    * @param actual   the actual bounding box 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox3DEquals(FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual, double epsilon)
   {
      assertFrameBoundingBox3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox3DEquals(String messagePrefix, FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual,
                                                     double epsilon)
   {
      assertFrameBoundingBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 3Ds are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameBoundingBox3DEquals(String messagePrefix, FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual,
                                                     double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertBoundingBox3DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 3D. Not modified.
    * @param actual   the actual bounding box 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox3DGeometricallyEquals(FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual, double epsilon)
   {
      assertFrameBoundingBox3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox3DGeometricallyEquals(String messagePrefix, FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual,
                                                                  double epsilon)
   {
      assertFrameBoundingBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame bounding box 3Ds represent the same geometry
    * to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBoundingBox3DGeometricallyEquals(String messagePrefix, FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual,
                                                                  double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertBoundingBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame convex polygon 2D are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame convex polygon 2D. Not modified.
    * @param actual   the actual frame convex polygon 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two frame convex polygon 2D are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolygon2DEquals(FrameConvexPolygon2DReadOnly expected, FrameConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertFrameConvexPolygon2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame convex polygon 2D are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame convex polygon 2D. Not modified.
    * @param actual        the actual frame convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame convex polygon 2D are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolygon2DEquals(String messagePrefix, FrameConvexPolygon2DReadOnly expected, FrameConvexPolygon2DReadOnly actual,
                                                       double epsilon)
   {
      assertFrameConvexPolygon2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two convex polygon 2D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame convex polygon 2D. Not modified.
    * @param actual        the actual frame convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame convex polygon 2D are not equal or not expressed in the
    *                        reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolygon2DEquals(String messagePrefix, FrameConvexPolygon2DReadOnly expected, FrameConvexPolygon2DReadOnly actual,
                                                       double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts that the two frame convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame convex polygon 2D. Not modified.
    * @param actual   the actual frame convex polygon 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two convex polygon 2D do not represent the same geometry or not
    *                        expressed in the reference frame. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameConvexPolygon2DGeometricallyEquals(FrameConvexPolygon2DReadOnly expected, FrameConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertFrameConvexPolygon2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two frame convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame convex polygon 2D. Not modified.
    * @param actual        the actual frame convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two frame convex polygon 2D do not represent the same geometry or
    *                        not expressed in the reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameConvexPolygon2DGeometricallyEquals(String messagePrefix, FrameConvexPolygon2DReadOnly expected,
                                                                    FrameConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertFrameConvexPolygon2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two frame convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected frame convex polygon 2D. Not modified.
    * @param actual        the actual frame convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two frame convex polygon 2D do not represent the same geometry or
    *                        not expressed in the reference frame. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameConvexPolygon2DGeometricallyEquals(String messagePrefix, FrameConvexPolygon2DReadOnly expected,
                                                                    FrameConvexPolygon2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (expected != null ^ actual != null)
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameTuple2DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameTuple2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameTuple3DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameTuple3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameTuple4DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameTuple4DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameOrientation2DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameOrientation2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameYawPitchRollReadOnly expected, FrameYawPitchRollReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameYawPitchRollString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameYawPitchRollString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameMatrix3DReadOnly expected, FrameMatrix3DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameMatrix3DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameMatrix3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameBoundingBox2DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameBoundingBox2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual, String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameBoundingBox3DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameBoundingBox3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameConvexPolygon2DReadOnly expected, FrameConvexPolygon2DReadOnly actual,
                                                   String format)
   {
      String expectedAsString = EuclidFrameIOTools.getFrameConvexPolygon2DString(format, expected);
      String actualAsString = EuclidFrameIOTools.getFrameConvexPolygon2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\nDifference of: " + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   private static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }
}

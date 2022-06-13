package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class provides the tools to perform a variety of assertions on frame shape types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameShapeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidFrameShapeTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected collision result. Not modified.
    * @param actual   the actual collision result. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultEquals(EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                    double epsilon)
   {
      assertEuclidFrameShape3DCollisionResultEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultEquals(String messagePrefix,
                                                                    EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                    double epsilon)
   {
      assertEuclidFrameShape3DCollisionResultEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultEquals(String messagePrefix,
                                                                    EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                    double epsilon,
                                                                    String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);
         }
         else
         {
            Vector3D differenceNormalOnA = new Vector3D();
            differenceNormalOnA.sub(expected.getNormalOnA(), actual.getNormalOnA());
            Vector3D differenceNormalOnB = new Vector3D();
            differenceNormalOnB.sub(expected.getNormalOnB(), actual.getNormalOnB());

            String difference = "[";
            difference += "distance: " + Math.abs(expected.getSignedDistance() - actual.getSignedDistance());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
         }
      }
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected collision result. Not modified.
    * @param actual   the actual collision result. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double epsilon)
   {
      assertEuclidFrameShape3DCollisionResultGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double distanceEpsilon,
                                                                                 double pointTangentialEpsilon,
                                                                                 double normalEpsilon)
   {
      assertEuclidFrameShape3DCollisionResultGeometricallyEquals(null,
                                                                 expected,
                                                                 actual,
                                                                 distanceEpsilon,
                                                                 pointTangentialEpsilon,
                                                                 normalEpsilon,
                                                                 DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double epsilon)
   {
      assertEuclidFrameShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix          prefix to add to the automated message.
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double distanceEpsilon,
                                                                                 double pointTangentialEpsilon,
                                                                                 double normalEpsilon)
   {
      assertEuclidFrameShape3DCollisionResultGeometricallyEquals(messagePrefix,
                                                                 expected,
                                                                 actual,
                                                                 distanceEpsilon,
                                                                 pointTangentialEpsilon,
                                                                 normalEpsilon,
                                                                 DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double epsilon,
                                                                                 String format)
   {
      assertEuclidFrameShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, epsilon, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix          prefix to add to the automated message.
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @param format                 the format to use for printing each component when an
    *                               {@code AssertionError} is thrown.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual,
                                                                                 double distanceEpsilon,
                                                                                 double pointTangentialEpsilon,
                                                                                 double normalEpsilon,
                                                                                 String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, distanceEpsilon, pointTangentialEpsilon, normalEpsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);
         }
         else
         {
            Vector3D differenceNormalOnA = new Vector3D();
            differenceNormalOnA.sub(expected.getNormalOnA(), actual.getNormalOnA());
            Vector3D differenceNormalOnB = new Vector3D();
            differenceNormalOnB.sub(expected.getNormalOnB(), actual.getNormalOnB());

            String difference = "[";
            difference += "distance: " + Math.abs(expected.getSignedDistance() - actual.getSignedDistance());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, difference, format);
         }
      }
   }
}

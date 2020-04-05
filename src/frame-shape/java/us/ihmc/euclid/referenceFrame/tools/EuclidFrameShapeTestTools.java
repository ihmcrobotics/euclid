package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getEuclidFrameShape3DCollisionResultString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameBox3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameCapsule3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameConvexPolytope3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameCylinder3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameEllipsoid3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameFace3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFramePointShape3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameRamp3DString;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools.getFrameSphere3DString;

import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
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
    * Asserts on a per component basis that the two boxes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected box. Not modified.
    * @param actual   the actual box. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two boxes are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBox3DEquals(FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon)
   {
      assertFrameBox3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two boxes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected box. Not modified.
    * @param actual        the actual box. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two boxes are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBox3DEquals(String messagePrefix, FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon)
   {
      assertFrameBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two boxes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected box. Not modified.
    * @param actual        the actual box. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two boxes are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameBox3DEquals(String messagePrefix, FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two boxes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected box. Not modified.
    * @param actual   the actual box. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two boxes do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameBox3DGeometricallyEquals(FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon)
   {
      assertFrameBox3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two boxes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected box. Not modified.
    * @param actual        the actual box. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two boxes do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameBox3DGeometricallyEquals(String messagePrefix, FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon)
   {
      assertFrameBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two boxes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected box. Not modified.
    * @param actual        the actual box. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two boxes do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameBox3DGeometricallyEquals(String messagePrefix, FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, double epsilon,
                                                          String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two capsules are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected capsule. Not modified.
    * @param actual   the actual capsule. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two capsules are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCapsule3DEquals(FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual, double epsilon)
   {
      assertFrameCapsule3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two capsules are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected capsule. Not modified.
    * @param actual        the actual capsule. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two capsules are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCapsule3DEquals(String messagePrefix, FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual, double epsilon)
   {
      assertFrameCapsule3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two capsules are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected capsule. Not modified.
    * @param actual        the actual capsule. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two capsules are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCapsule3DEquals(String messagePrefix, FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual, double epsilon,
                                                 String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two capsules represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected capsule. Not modified.
    * @param actual   the actual capsule. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two capsules do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameCapsule3DGeometricallyEquals(FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual, double epsilon)
   {
      assertFrameCapsule3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two capsules represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected capsule. Not modified.
    * @param actual        the actual capsule. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two capsules do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameCapsule3DGeometricallyEquals(String messagePrefix, FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual,
                                                              double epsilon)
   {
      assertFrameCapsule3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two capsules represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected capsule. Not modified.
    * @param actual        the actual capsule. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two capsules do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameCapsule3DGeometricallyEquals(String messagePrefix, FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual,
                                                              double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two cylinders are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected cylinder. Not modified.
    * @param actual   the actual cylinder. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two cylinders are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCylinder3DEquals(FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual, double epsilon)
   {
      assertFrameCylinder3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two cylinders are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected cylinder. Not modified.
    * @param actual        the actual cylinder. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two cylinders are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCylinder3DEquals(String messagePrefix, FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual, double epsilon)
   {
      assertFrameCylinder3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two cylinders are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected cylinder. Not modified.
    * @param actual        the actual cylinder. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two cylinders are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameCylinder3DEquals(String messagePrefix, FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual, double epsilon,
                                                  String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two cylinders represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected cylinder. Not modified.
    * @param actual   the actual cylinder. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two cylinders do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFrameCylinder3DGeometricallyEquals(FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual, double epsilon)
   {
      assertFrameCylinder3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two cylinders represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected cylinder. Not modified.
    * @param actual        the actual cylinder. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two cylinders do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFrameCylinder3DGeometricallyEquals(String messagePrefix, FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual,
                                                               double epsilon)
   {
      assertFrameCylinder3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two cylinders represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected cylinder. Not modified.
    * @param actual        the actual cylinder. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two cylinders do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFrameCylinder3DGeometricallyEquals(String messagePrefix, FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual,
                                                               double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected ellipsoid. Not modified.
    * @param actual   the actual ellipsoid. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two ellipsoids are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameEllipsoid3DEquals(FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual, double epsilon)
   {
      assertFrameEllipsoid3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ellipsoid. Not modified.
    * @param actual        the actual ellipsoid. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two ellipsoids are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameEllipsoid3DEquals(String messagePrefix, FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual, double epsilon)
   {
      assertFrameEllipsoid3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ellipsoid. Not modified.
    * @param actual        the actual ellipsoid. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two ellipsoids are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertFrameEllipsoid3DEquals(String messagePrefix, FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual, double epsilon,
                                                   String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected ellipsoid. Not modified.
    * @param actual   the actual ellipsoid. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two ellipsoids do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFrameEllipsoid3DGeometricallyEquals(FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ellipsoid. Not modified.
    * @param actual        the actual ellipsoid. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two ellipsoids do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual,
                                                           double epsilon)
   {
      assertFrameEllipsoid3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two ellipsoids represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ellipsoid. Not modified.
    * @param actual        the actual ellipsoid. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two ellipsoids do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFrameEllipsoid3DGeometricallyEquals(String messagePrefix, FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual,
                                                                double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two point shapes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected point shape. Not modified.
    * @param actual   the actual point shape. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two point shapes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFramePointShape3DEquals(FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual, double epsilon)
   {
      assertFramePointShape3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two point shapes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected point shape. Not modified.
    * @param actual        the actual point shape. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two point shapes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFramePointShape3DEquals(String messagePrefix, FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual, double epsilon)
   {
      assertFramePointShape3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two point shapes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected point shape. Not modified.
    * @param actual        the actual point shape. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two point shapes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFramePointShape3DEquals(String messagePrefix, FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual, double epsilon,
                                                    String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two point shapes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected point shape. Not modified.
    * @param actual   the actual point shape. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two point shapes do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFramePointShape3DGeometricallyEquals(FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual, double epsilon)
   {
      assertFramePointShape3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two point shapes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected point shape. Not modified.
    * @param actual        the actual point shape. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two point shapes do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFramePointShape3DGeometricallyEquals(String messagePrefix, FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual,
                                                                 double epsilon)
   {
      assertFramePointShape3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two point shapes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected point shape. Not modified.
    * @param actual        the actual point shape. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two point shapes do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertFramePointShape3DGeometricallyEquals(String messagePrefix, FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual,
                                                                 double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two ramps are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected ramp. Not modified.
    * @param actual   the actual ramp. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two ramps are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRamp3DEquals(FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon)
   {
      assertFrameRamp3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two ramps are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ramp. Not modified.
    * @param actual        the actual ramp. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two ramps are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRamp3DEquals(String messagePrefix, FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon)
   {
      assertFrameRamp3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two ramps are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ramp. Not modified.
    * @param actual        the actual ramp. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two ramps are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameRamp3DEquals(String messagePrefix, FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two ramps represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected ramp. Not modified.
    * @param actual   the actual ramp. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two ramps do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameRamp3DGeometricallyEquals(FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon)
   {
      assertFrameRamp3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two ramps represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ramp. Not modified.
    * @param actual        the actual ramp. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two ramps do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameRamp3DGeometricallyEquals(String messagePrefix, FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon)
   {
      assertFrameRamp3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two ramps represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected ramp. Not modified.
    * @param actual        the actual ramp. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two ramps do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameRamp3DGeometricallyEquals(String messagePrefix, FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, double epsilon,
                                                           String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two spheres are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected sphere. Not modified.
    * @param actual   the actual sphere. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spheres are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameSphere3DEquals(FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon)
   {
      assertFrameSphere3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spheres are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected sphere. Not modified.
    * @param actual        the actual sphere. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spheres are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameSphere3DEquals(String messagePrefix, FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon)
   {
      assertFrameSphere3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spheres are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected sphere. Not modified.
    * @param actual        the actual sphere. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spheres are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameSphere3DEquals(String messagePrefix, FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two spheres represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected sphere. Not modified.
    * @param actual   the actual sphere. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spheres do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameSphere3DGeometricallyEquals(FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon)
   {
      assertFrameSphere3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spheres represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected sphere. Not modified.
    * @param actual        the actual sphere. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spheres do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameSphere3DGeometricallyEquals(String messagePrefix, FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon)
   {
      assertFrameSphere3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spheres represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected sphere. Not modified.
    * @param actual        the actual sphere. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spheres do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameSphere3DGeometricallyEquals(String messagePrefix, FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
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
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon)
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
   public static void assertEuclidFrameShape3DCollisionResultEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon)
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
   public static void assertEuclidFrameShape3DCollisionResultEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                    EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            throwNotEqualAssertionError(messagePrefix, expected, actual, format);
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
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
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
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon)
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
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double distanceEpsilon,
                                                                                 double pointTangentialEpsilon, double normalEpsilon)
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
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon)
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
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double distanceEpsilon,
                                                                                 double pointTangentialEpsilon, double normalEpsilon)
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
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double epsilon,
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
   public static void assertEuclidFrameShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                                                 EuclidFrameShape3DCollisionResultReadOnly actual, double distanceEpsilon,
                                                                                 double pointTangentialEpsilon, double normalEpsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, distanceEpsilon, pointTangentialEpsilon, normalEpsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            throwNotEqualAssertionError(messagePrefix, expected, actual, format);
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
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
         }
      }
   }

   /**
    * Asserts on a per component basis that the two faces are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected face. Not modified.
    * @param actual   the actual face. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two faces are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameFace3DEquals(FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon)
   {
      assertFrameFace3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two faces are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected face. Not modified.
    * @param actual        the actual face. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two faces are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameFace3DEquals(String messagePrefix, FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon)
   {
      assertFrameFace3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two faces are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected face. Not modified.
    * @param actual        the actual face. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two faces are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertFrameFace3DEquals(String messagePrefix, FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two faces represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected face. Not modified.
    * @param actual   the actual face. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two faces do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameFace3DGeometricallyEquals(FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon)
   {
      assertFrameFace3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two faces represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected face. Not modified.
    * @param actual        the actual face. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two faces do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameFace3DGeometricallyEquals(String messagePrefix, FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon)
   {
      assertFrameFace3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two faces represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected face. Not modified.
    * @param actual        the actual face. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two faces do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertFrameFace3DGeometricallyEquals(String messagePrefix, FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, double epsilon,
                                                           String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected convex polytope. Not modified.
    * @param actual   the actual convex polytope. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two convex polytopes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DEquals(FrameConvexPolytope3DReadOnly expected, FrameConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertFrameConvexPolytope3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected convex polytope. Not modified.
    * @param actual        the actual convex polytope. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two convex polytopes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DEquals(String messagePrefix, FrameConvexPolytope3DReadOnly expected, FrameConvexPolytope3DReadOnly actual,
                                                        double epsilon)
   {
      assertFrameConvexPolytope3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected convex polytope. Not modified.
    * @param actual        the actual convex polytope. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two convex polytopes are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DEquals(String messagePrefix, FrameConvexPolytope3DReadOnly expected, FrameConvexPolytope3DReadOnly actual,
                                                        double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected convex polytope. Not modified.
    * @param actual   the actual convex polytope. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two convex polytopes do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DGeometricallyEquals(FrameConvexPolytope3DReadOnly expected, FrameConvexPolytope3DReadOnly actual,
                                                                     double epsilon)
   {
      assertFrameConvexPolytope3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected convex polytope. Not modified.
    * @param actual        the actual convex polytope. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two convex polytopes do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DGeometricallyEquals(String messagePrefix, FrameConvexPolytope3DReadOnly expected,
                                                                     FrameConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertFrameConvexPolytope3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two convex polytopes represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected convex polytope. Not modified.
    * @param actual        the actual convex polytope. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two convex polytopes do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameConvexPolytope3DGeometricallyEquals(String messagePrefix, FrameConvexPolytope3DReadOnly expected,
                                                                     FrameConvexPolytope3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameBox3DReadOnly expected, FrameBox3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameBox3DString(format, expected);
      String actualAsString = getFrameBox3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameCapsule3DReadOnly expected, FrameCapsule3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameCapsule3DString(format, expected);
      String actualAsString = getFrameCapsule3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameCylinder3DReadOnly expected, FrameCylinder3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameCylinder3DString(format, expected);
      String actualAsString = getFrameCylinder3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameEllipsoid3DReadOnly expected, FrameEllipsoid3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameEllipsoid3DString(format, expected);
      String actualAsString = getFrameEllipsoid3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FramePointShape3DReadOnly expected, FramePointShape3DReadOnly actual, String format)
   {
      String expectedAsString = getFramePointShape3DString(format, expected);
      String actualAsString = getFramePointShape3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameRamp3DReadOnly expected, FrameRamp3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameRamp3DString(format, expected);
      String actualAsString = getFrameRamp3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameSphere3DReadOnly expected, FrameSphere3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameSphere3DString(format, expected);
      String actualAsString = getFrameSphere3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                   EuclidFrameShape3DCollisionResultReadOnly actual, String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, format, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidFrameShape3DCollisionResultReadOnly expected,
                                                   EuclidFrameShape3DCollisionResultReadOnly actual, String format, String differenceAsString)
   {
      String expectedAsString = getEuclidFrameShape3DCollisionResultString(format, expected);
      String actualAsString = getEuclidFrameShape3DCollisionResultString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameFace3DReadOnly expected, FrameFace3DReadOnly actual, String format)
   {
      String expectedAsString = getFrameFace3DString(format, expected);
      String actualAsString = getFrameFace3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameConvexPolytope3DReadOnly expected, FrameConvexPolytope3DReadOnly actual,
                                                   String format)
   {
      String expectedAsString = getFrameConvexPolytope3DString(format, expected);
      String actualAsString = getFrameConvexPolytope3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }
}

package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.*;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides the tools to perform a variety of assertions on shape types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidShapeTestTools()
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
   public static void assertBox3DEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(null, expected, actual, epsilon);
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
   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
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
   public static void assertBox3DGeometricallyEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
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
   public static void assertCapsule3DEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(null, expected, actual, epsilon);
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
   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon, String format)
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
   public static void assertCapsule3DGeometricallyEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertCapsule3DGeometricallyEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertCapsule3DGeometricallyEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon,
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
   public static void assertCylinder3DEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(null, expected, actual, epsilon);
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
   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon, String format)
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
   public static void assertCylinder3DGeometricallyEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertCylinder3DGeometricallyEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertCylinder3DGeometricallyEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon,
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
   public static void assertEllipsoid3DEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(null, expected, actual, epsilon);
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
   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon, String format)
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
   public static void assertEllipsoid3DGeometricallyEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
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
   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon,
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
   public static void assertPointShape3DEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(null, expected, actual, epsilon);
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
   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon, String format)
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
   public static void assertPointShape3DGeometricallyEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertPointShape3DGeometricallyEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertPointShape3DGeometricallyEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon,
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
   public static void assertRamp3DEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(null, expected, actual, epsilon);
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
   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
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
   public static void assertRamp3DGeometricallyEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
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
   public static void assertSphere3DEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(null, expected, actual, epsilon);
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
   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
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
   public static void assertSphere3DGeometricallyEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two tori are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected torus. Not modified.
    * @param actual   the actual torus. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tori are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTorus3DEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tori are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected torus. Not modified.
    * @param actual        the actual torus. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tori are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tori are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected torus. Not modified.
    * @param actual        the actual torus. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tori are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two tori represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected torus. Not modified.
    * @param actual   the actual torus. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two tori do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertTorus3DGeometricallyEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tori represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected torus. Not modified.
    * @param actual        the actual torus. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two tori do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tori represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected torus. Not modified.
    * @param actual        the actual torus. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two tori do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
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
   public static void assertEuclidShape3DCollisionResultEquals(EuclidShape3DCollisionResultReadOnly expected, EuclidShape3DCollisionResultReadOnly actual,
                                                               double epsilon)
   {
      assertEuclidShape3DCollisionResultEquals(null, expected, actual, epsilon);
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
   public static void assertEuclidShape3DCollisionResultEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                               EuclidShape3DCollisionResultReadOnly actual, double epsilon)
   {
      assertEuclidShape3DCollisionResultEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertEuclidShape3DCollisionResultEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                               EuclidShape3DCollisionResultReadOnly actual, double epsilon, String format)
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double epsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double distanceEpsilon,
                                                                            double pointTangentialEpsilon, double normalEpsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(null, expected, actual, distanceEpsilon, pointTangentialEpsilon, normalEpsilon, DEFAULT_FORMAT);
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double epsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double distanceEpsilon,
                                                                            double pointTangentialEpsilon, double normalEpsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix,
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double epsilon, String format)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, epsilon, epsilon, format);
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
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual, double distanceEpsilon,
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
   public static void assertFace3DEquals(Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DEquals(null, expected, actual, epsilon);
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
   public static void assertFace3DEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertFace3DEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon, String format)
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
   public static void assertFace3DGeometricallyEquals(Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertFace3DGeometricallyEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertFace3DGeometricallyEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon, String format)
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
   public static void assertConvexPolytope3DEquals(ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DEquals(null, expected, actual, epsilon);
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
   public static void assertConvexPolytope3DEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertConvexPolytope3DEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon,
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
   public static void assertConvexPolytope3DGeometricallyEquals(ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertConvexPolytope3DGeometricallyEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual,
                                                                double epsilon)
   {
      assertConvexPolytope3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertConvexPolytope3DGeometricallyEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual,
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
    * Asserts the integrity of a convex polytope.
    * <p>
    * Properties asserted include notably:
    * <ul>
    * <li>vertices, edges, and faces are properly linked to each other.
    * <li>convexity between neighbor faces.
    * <li>convexity and clockwise winding for each face.
    * <li>each face's normal points outside.
    * </ul>
    * </p>
    *
    * @param convexPolytope3D the convex polytope to run assertions on. Not modified.
    * @throws AssertionError if the convex polytope is corrupted.
    */
   public static void assertConvexPolytope3DGeneralIntegrity(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      assertConvexPolytope3DGeneralIntegrity(null, convexPolytope3D);
   }

   /**
    * Asserts the integrity of a convex polytope.
    * <p>
    * Properties asserted include notably:
    * <ul>
    * <li>vertices, edges, and faces are properly linked to each other.
    * <li>convexity between neighbor faces.
    * <li>convexity and clockwise winding for each face.
    * <li>each face's normal points outside.
    * </ul>
    * </p>
    *
    * @param messagePrefix    prefix to add to the automated message.
    * @param convexPolytope3D the convex polytope to run assertions on. Not modified.
    * @throws AssertionError if the convex polytope is corrupted.
    */
   public static void assertConvexPolytope3DGeneralIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D.getCentroid().containsNaN())
         EuclidCoreTestTools.throwAssertionError(messagePrefix, "The polytope's centroid contains NaN.");

      if (convexPolytope3D.getNumberOfFaces() > 1)
      {
         int verticesSize = convexPolytope3D.getVertices().size();
         int halfEdgesSize = convexPolytope3D.getHalfEdges().size();
         int facesSize = convexPolytope3D.getFaces().size();

         int expectedNumberOfVertices = EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(facesSize, halfEdgesSize / 2);
         if (verticesSize != expectedNumberOfVertices)
            EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                    "Inconsistent data size, expected " + expectedNumberOfVertices + " vertices but was " + verticesSize);
      }

      assertConvexPolytope3DFacesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DHalfEdgesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DVerticesIntegrity(messagePrefix, convexPolytope3D);
   }

   private static void assertConvexPolytope3DFacesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Face3DReadOnly> faceSet = new HashSet<>();

      for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      { // We assert the uniqueness of the faces.
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         if (!faceSet.add(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face is a duplicate.");
      }

      Vector3D toOrigin = new Vector3D();
      Vector3D toDestination = new Vector3D();
      Vector3D toCentroid = new Vector3D();
      Vector3D cross = new Vector3D();

      if (convexPolytope3D.getNumberOfFaces() == 1)
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);

               if (edge.getFace() != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != (edgeIndex + 1) % edges.size())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");
            }

            if (edges.size() >= 3)
            {
               for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
               {
                  HalfEdge3DReadOnly edge = edges.get(edgeIndex);

                  if (edge.getFace() != face)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
                  if (!edges.contains(edge.getNext()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
                  if (!edges.contains(edge.getPrevious()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
                  if (edges.indexOf(edge.getNext()) != ((edgeIndex + 1) % edges.size()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

                  // Verifying the edges are all clockwise ordered.
                  toOrigin.sub(edge.getOrigin(), centroid);
                  toDestination.sub(edge.getDestination(), centroid);
                  cross.cross(toDestination, toOrigin);

                  if (cross.dot(normal) < 0.0)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");
               }
            }
         }
      }
      else
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            // Verifying the normal is pointing towards the outside of the polytope.
            toCentroid.sub(centroid, convexPolytope3D.getCentroid());

            if (toCentroid.dot(normal) < 0.0)
            {
               if (convexPolytope3D.getVolume() < convexPolytope3D.getConstructionEpsilon())
                  System.out.println("WARNING: "
                        + EuclidCoreTestTools.addPrefixToMessage(messagePrefix,
                                                                 faceIndex + "th face's normal might be pointing towards the inside of the polytope."));
               else
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face's normal is pointing towards the inside of the polytope.");
            }

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);
               Face3DReadOnly neighbor = edge.getTwin().getFace();

               if (edge.getFace() != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != ((edgeIndex + 1) % edges.size()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

               // Verifying the edges are all clockwise ordered.
               toOrigin.sub(edge.getOrigin(), centroid);
               toDestination.sub(edge.getDestination(), centroid);
               cross.cross(toDestination, toOrigin);

               if (cross.dot(normal) < 0.0)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");

               if (EuclidGeometryTools.isPoint3DAbovePlane3D(neighbor.getCentroid(), face.getCentroid(), face.getNormal()))
               {
                  if (face.signedDistanceFromSupportPlane(neighbor.getCentroid()) < convexPolytope3D.getConstructionEpsilon())
                     System.out.println("WARNING: "
                           + EuclidCoreTestTools.addPrefixToMessage(messagePrefix,
                                                                    faceIndex + "th face might be concave with respect to a neighor, the "
                                                                          + convexPolytope3D.getFaces().indexOf(neighbor) + "th face."));
                  else
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face is concave with respect to a neighor, the "
                                                                   + convexPolytope3D.getFaces().indexOf(neighbor) + "th face.");
               }
            }
         }
      }
   }

   private static void assertConvexPolytope3DHalfEdgesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<HalfEdge3DReadOnly> halfEdgeSet = new HashSet<>();

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      { // We assert the uniqueness of the half-edges.
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);
         if (!halfEdgeSet.add(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is a duplicate.");
      }

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      {
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);

         Vertex3DReadOnly origin = halfEdge.getOrigin();
         Vertex3DReadOnly destination = halfEdge.getDestination();
         HalfEdge3DReadOnly twin = halfEdge.getTwin();
         HalfEdge3DReadOnly next = halfEdge.getNext();
         HalfEdge3DReadOnly previous = halfEdge.getPrevious();
         Face3DReadOnly face = halfEdge.getFace();

         if (convexPolytope3D.getNumberOfFaces() > 1 && twin == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is null.");
         if (next == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is null.");
         if (previous == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is null.");
         if (face == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is null.");

         if (twin != null && (origin != twin.getDestination() || destination != twin.getOrigin()))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is inconsistent with its twin.");
         if (origin != previous.getDestination())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its previous.");
         if (destination != next.getOrigin())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its next.");
         if (halfEdge.getFace() != previous.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its previous.");
         if (halfEdge.getFace() != next.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its next.");
         if (!origin.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not associated to its origin.");
         if (origin != destination && destination.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge should not be associated to its origin.");
         if (!halfEdge.getFace().getEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face does not declare it as one of its edges.");

         if (!convexPolytope3D.getFaces().contains(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is not registered as a polytope face.");
         if (twin != null && !convexPolytope3D.getHalfEdges().contains(twin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(next))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(previous))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getVertices().contains(origin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's origin is not registered as a polytope vertex.");
         if (!convexPolytope3D.getVertices().contains(destination))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's destination is not registered as a polytope vertex.");
      }
   }

   private static void assertConvexPolytope3DVerticesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Vertex3DReadOnly> vertexSet = new HashSet<>();

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      { // We assert the uniqueness of the vertices.
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         if (!vertexSet.add(vertex))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is a duplicate.");
      }

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      {
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         Collection<? extends HalfEdge3DReadOnly> associatedEdges = vertex.getAssociatedEdges();

         // Used to assert uniqueness of the faces obtained from the associated-edges.
         Set<Face3DReadOnly> connectedFaces = new HashSet<>();

         for (HalfEdge3DReadOnly associatedEdge : associatedEdges)
         {
            if (vertex != associatedEdge.getOrigin())
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is not the origin of an associated edge.");

            if (!connectedFaces.add(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, "The connected faces to " + vertexIndex + "th vertex are not unique.");

            if (convexPolytope3D.getNumberOfFaces() > 1)
            {
               if (associatedEdge.getDestination().getEdgeTo(vertex) == null)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination does not have an edge going back to it.");
               if (associatedEdge.getDestination().getEdgeTo(vertex) != associatedEdge.getTwin())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination's return edge is not the twin.");
            }
            if (!associatedEdge.getFace().getVertices().contains(vertex))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which face does not delare it.");

            if (!convexPolytope3D.getFaces().contains(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge's face which is not registered as a polytope face.");
            if (!convexPolytope3D.getHalfEdges().contains(associatedEdge))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which is not registered as a polytope edge.");
            if (!convexPolytope3D.getVertices().contains(associatedEdge.getDestination()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                       vertexIndex + "th vertex has an edge's destination which is not registered as a polytope vertex.");
         }
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, String format)
   {
      String expectedAsString = getBox3DString(format, expected);
      String actualAsString = getBox3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, String format)
   {
      String expectedAsString = getCapsule3DString(format, expected);
      String actualAsString = getCapsule3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, String format)
   {
      String expectedAsString = getCylinder3DString(format, expected);
      String actualAsString = getCylinder3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, String format)
   {
      String expectedAsString = getEllipsoid3DString(format, expected);
      String actualAsString = getEllipsoid3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, String format)
   {
      String expectedAsString = getPointShape3DString(format, expected);
      String actualAsString = getPointShape3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, String format)
   {
      String expectedAsString = getRamp3DString(format, expected);
      String actualAsString = getRamp3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, String format)
   {
      String expectedAsString = getSphere3DString(format, expected);
      String actualAsString = getSphere3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, String format)
   {
      String expectedAsString = getTorus3DString(format, expected);
      String actualAsString = getTorus3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                   EuclidShape3DCollisionResultReadOnly actual, String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, format, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidShape3DCollisionResultReadOnly expected,
                                                   EuclidShape3DCollisionResultReadOnly actual, String format, String differenceAsString)
   {
      String expectedAsString = getEuclidShape3DCollisionResultString(format, expected);
      String actualAsString = getEuclidShape3DCollisionResultString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, String format)
   {
      String expectedAsString = getFace3DString(format, expected);
      String actualAsString = getFace3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, String format)
   {
      String expectedAsString = getConvexPolytope3DString(format, expected);
      String actualAsString = getConvexPolytope3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }
}

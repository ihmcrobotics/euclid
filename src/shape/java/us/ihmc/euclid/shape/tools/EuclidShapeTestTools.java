package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.*;

import us.ihmc.euclid.shape.collision.CollisionTestResult;
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

public class EuclidShapeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   public static void assertBox3DEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(null, expected, actual, epsilon);
   }

   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertBox3DGeometricallyEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCapsule3DEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(null, expected, actual, epsilon);
   }

   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCapsule3DGeometricallyEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCapsule3DGeometricallyEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

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

   public static void assertCylinder3DEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(null, expected, actual, epsilon);
   }

   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCylinder3DGeometricallyEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCylinder3DGeometricallyEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

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

   public static void assertEllipsoid3DEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(null, expected, actual, epsilon);
   }

   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertEllipsoid3DGeometricallyEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

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

   public static void assertPointShape3DEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(null, expected, actual, epsilon);
   }

   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertPointShape3DGeometricallyEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertPointShape3DGeometricallyEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

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

   public static void assertRamp3DEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(null, expected, actual, epsilon);
   }

   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertRamp3DGeometricallyEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertSphere3DEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(null, expected, actual, epsilon);
   }

   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertSphere3DGeometricallyEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertTorus3DEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(null, expected, actual, epsilon);
   }

   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertTorus3DGeometricallyEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCollisionTestResultEquals(CollisionTestResult expected, CollisionTestResult actual, double epsilon)
   {
      assertCollisionTestResultEquals(null, expected, actual, epsilon);
   }

   public static void assertCollisionTestResultEquals(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual, double epsilon)
   {
      assertCollisionTestResultEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCollisionTestResultEquals(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual, double epsilon,
                                                      String format)
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
            difference += "depth: " + Math.abs(expected.getDepth() - actual.getDepth());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
         }
      }
   }

   public static void assertCollisionTestResultGeometricallyEquals(CollisionTestResult expected, CollisionTestResult actual, double epsilon)
   {
      assertCollisionTestResultGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCollisionTestResultGeometricallyEquals(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual,
                                                                   double epsilon)
   {
      assertCollisionTestResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCollisionTestResultGeometricallyEquals(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual,
                                                                   double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
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

   private static void throwNotEqualAssertionError(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual, String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, format, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, CollisionTestResult expected, CollisionTestResult actual, String format,
                                                   String differenceAsString)
   {
      String expectedAsString = getCollisionTestResultString(format, expected);
      String actualAsString = getCollisionTestResultString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }
}

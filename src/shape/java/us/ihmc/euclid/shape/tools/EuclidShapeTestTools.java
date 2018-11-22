package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.*;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import us.ihmc.euclid.shape.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class EuclidShapeTestTools
{
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
}

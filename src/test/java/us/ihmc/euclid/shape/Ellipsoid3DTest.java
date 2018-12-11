package us.ihmc.euclid.shape;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Ellipsoid3DTest
{
   private static final double EPSILON = 1.0e-10; // This epsilon is meant small changes in coordinates. Use Ellipsoid3d's DEFAULT_EPSILON for error handling.

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double xRadius = EuclidCoreRandomTools.nextDouble(random, 0.02, 10.0);
         double yRadius = EuclidCoreRandomTools.nextDouble(random, 0.02, 10.0);
         double zRadius = EuclidCoreRandomTools.nextDouble(random, 0.02, 10.0);
         Ellipsoid3D ellipsoid3d = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);
         testHelper.runSimpleTests(ellipsoid3d, random, numberOfPoints);
      }
   }

   @Test
   public void testSimpleWithNoTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius - EPSILON, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius - EPSILON, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius - EPSILON)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xRadius + EPSILON, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yRadius + EPSILON, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, -zRadius + EPSILON)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius + EPSILON, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius + EPSILON, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius + EPSILON)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-xRadius - EPSILON, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yRadius - EPSILON, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, -zRadius - EPSILON)));

      Point3D closestPointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();
      boolean isInside = ellipsoid.checkIfInside(new Point3D(xRadius - EPSILON, 0.0, 0.0), closestPointToPack, normalToPack);
      assertTrue(isInside);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.0, 0.0, 0.0), normalToPack, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(xRadius, 0.0, 0.0), closestPointToPack, 1e-7);

      isInside = ellipsoid.checkIfInside(new Point3D(3.0, 7.0, -12.0), closestPointToPack, normalToPack);
      assertFalse(isInside);
      double xScaled = closestPointToPack.getX() / xRadius;
      double yScaled = closestPointToPack.getY() / yRadius;
      double zScaled = closestPointToPack.getZ() / zRadius;
      double sumSquared = xScaled * xScaled + yScaled * yScaled + zScaled * zScaled;
      assertEqualsDelta(1.0, sumSquared, 1e-7);
   }

   @Test
   public void testSimpleWithTranslationalTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      double xTranslation = 0.5;
      double yTranslation = -0.3;
      double zTranslation = 1.1;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3D(xTranslation, yTranslation, zTranslation));

      Ellipsoid3D ellipsoid = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation + xRadius - EPSILON, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation + yRadius - EPSILON, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation + zRadius - EPSILON)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation - xRadius + EPSILON, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation - yRadius + EPSILON, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation - zRadius + EPSILON)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation + xRadius + EPSILON, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation + yRadius + EPSILON, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation + zRadius + EPSILON)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation - xRadius - EPSILON, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation - yRadius - EPSILON, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation - zRadius - EPSILON)));
   }

   @Test
   public void testExampleUsage()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 2.0);
      transform.setTranslation(new Vector3D(0.0, 5.0, 0.0));

      Ellipsoid3D ellipsoid = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);

      assertTrue(ellipsoid.isInsideEpsilon(new Point3D(0.0, 8.0, 0.0), 0.001));
      assertTrue(ellipsoid.isInsideEpsilon(new Point3D(0.0, 7.9, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideEpsilon(new Point3D(0.0, 8.2, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideEpsilon(new Point3D(0.2, 8.2, 0.2), 0.001));
   }

   @Test
   public void testSet()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

      double[] zeroes = new double[] {0.0, 0.0, 0.0};

      double[] radii = new double[] {xRadius, yRadius, zRadius};

      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, zeroes, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, new double[] {-EPSILON, -EPSILON, -EPSILON}, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, zeroes, new double[] {EPSILON, EPSILON, EPSILON}, ellipsoid);
   }

   private void assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(boolean invertResult, double[] radii, double[] translations, double[] offsets,
                                                                   Ellipsoid3D ellipsoid)
   {
      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0] + radii[0] + offsets[0], translations[1], translations[2])));
      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0] - (radii[0] + offsets[0]), translations[1], translations[2])));

      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1] + radii[1] + offsets[1], translations[2])));
      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1] - (radii[1] + offsets[1]), translations[2])));

      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1], translations[2] + radii[2] + offsets[2])));
      assertEquals((Object) !invertResult, (Object) ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1], translations[2] - (radii[2] + offsets[2]))));
   }

   @Test
   public void testTranslation()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] radii = new double[] {xRadius, yRadius, zRadius};

         double xTranslation = random.nextDouble();
         double yTranslation = random.nextDouble();
         double zTranslation = random.nextDouble();

         double[] translations = new double[] {xTranslation, yTranslation, zTranslation};

         double[] zeroes = new double[] {0.0, 0.0, 0.0};

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(new Vector3D(xTranslation, yTranslation, zTranslation));

         Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
         ellipsoid.setPose(transform);

         assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation)));

         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, zeroes, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, new double[] {-EPSILON, -EPSILON, -EPSILON}, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, translations, new double[] {EPSILON, EPSILON, EPSILON}, ellipsoid);
      }
   }

   @Test
   public void testSimpleRotations()
   {
      for (int n = 0; n < ITERATIONS; n++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] angles = new double[] {Math.PI / 3.0, Math.PI / 4.0, Math.PI / 6.0};

         for (double angle : angles)
         {
            for (int i = 0; i < 3; i++)
            {
               RigidBodyTransform transform = new RigidBodyTransform();

               if (i == 0)
                  transform.setRotationRollAndZeroTranslation(angle);
               if (i == 1)
                  transform.setRotationPitchAndZeroTranslation(angle);
               if (i == 2)
                  transform.setRotationYawAndZeroTranslation(angle);

               Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
               ellipsoid.setPose(transform);

               assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

               if (i == 0)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius, 0.0, 0.0)));

                  double yCoord = yRadius * Math.cos(angle);
                  double zCoord = yRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord + EPSILON, zCoord + EPSILON)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -(yCoord + EPSILON), -(zCoord + EPSILON))));

                  yCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord - EPSILON, zCoord + EPSILON)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -(yCoord - EPSILON), -(zCoord + EPSILON))));
               }

               if (i == 1)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius, 0.0)));

                  double xCoord = xRadius * Math.cos(angle);
                  double zCoord = -xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord + EPSILON, 0.0, zCoord - EPSILON)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord + EPSILON), 0.0, -(zCoord - EPSILON))));

                  xCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = -zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord - EPSILON, 0.0, zCoord - EPSILON)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord - EPSILON), 0.0, -(zCoord - EPSILON))));
               }

               if (i == 2)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius)));

                  double xCoord = xRadius * Math.cos(angle);
                  double yCoord = xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord + EPSILON, yCoord + EPSILON, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord + EPSILON), -(yCoord + EPSILON), 0.0)));

                  xCoord = yRadius * Math.cos(angle + Math.PI / 2.0);
                  yCoord = yRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord - EPSILON, yCoord + EPSILON, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord - EPSILON), -(yCoord + EPSILON), 0.0)));
               }
            }
         }
      }
   }

   @Test
   public void testMethodsForRandomEllipsoids()
   {
      Random random = new Random(1865L);
      Ellipsoid3D ellipsoid = new Ellipsoid3D(1.0, 2.0, 3.0);
      double xRad, yRad, zRad;
      RigidBodyTransform transform = randomTransform(random);

      for (int n = 0; n < ITERATIONS; n++)
      {
         xRad = random.nextDouble();
         yRad = random.nextDouble();
         zRad = random.nextDouble();

         ellipsoid.setRadiusX(xRad);
         ellipsoid.setRadiusY(yRad);
         ellipsoid.setRadiusZ(zRad);
         ellipsoid.setPose(transform);

         Ellipsoid3D ellipsoidCopy = new Ellipsoid3D(ellipsoid);

         assertEqualsDelta(ellipsoid.getRadiusX(), ellipsoidCopy.getRadiusX(), 1e-10);
         assertEqualsDelta(ellipsoid.getRadiusY(), ellipsoidCopy.getRadiusY(), 1e-10);
         assertEqualsDelta(ellipsoid.getRadiusZ(), ellipsoidCopy.getRadiusZ(), 1e-10);

         Point3D center = new Point3D();
         ellipsoid.getPosition(center);

         Point3D centerCopy = new Point3D();
         ellipsoidCopy.getPosition(centerCopy);

         EuclidCoreTestTools.assertTuple3DEquals(center, centerCopy, 1e-10);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89725L);

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test with identical ellipsoids
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(firstEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         firstEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(firstEllipsoid.geometricallyEquals(firstEllipsoid, EPSILON), "Iteration: " + i);
         assertTrue(secondEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 ellipsoids with same pose and different radii - Method 1
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.1);
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         // Generating random radii while making sure we don't generate spheres
         double radiusX1 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double radiusY1 = EuclidCoreRandomTools.nextDouble(random, radiusX1 + 2.0 * epsilon, radiusX1 + 1.0);
         double radiusZ1 = EuclidCoreRandomTools.nextDouble(random, radiusY1 + 2.0 * epsilon, radiusY1 + 1.0);
         double radiusX2 = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double radiusY2 = EuclidCoreRandomTools.nextDouble(random, radiusX2 + 2.0 * epsilon, radiusX2 + 1.0);
         double radiusZ2 = EuclidCoreRandomTools.nextDouble(random, radiusY2 + 2.0 * epsilon, radiusY2 + 1.0);
         Vector3D radii1 = new Vector3D(radiusX1, radiusY1, radiusZ1);
         Vector3D radii2 = new Vector3D(radiusX2, radiusY2, radiusZ2);
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX1, radiusY1, radiusZ1);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX2, radiusY2, radiusZ2);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON) == radii1.geometricallyEquals(radii2, EPSILON), "Iteration: " + i);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 2 ellipsoids with same pose and different radii - Method 2
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.1);
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         // Generating random radii while making sure we don't generate spheres
         double radiusX1 = EuclidCoreRandomTools.nextDouble(random, 2.0 * epsilon, 10.0);
         double radiusY1 = EuclidCoreRandomTools.nextDouble(random, radiusX1 + 2.0 * epsilon, radiusX1 + 1.0);
         double radiusZ1 = EuclidCoreRandomTools.nextDouble(random, radiusY1 + 2.0 * epsilon, radiusY1 + 1.0);
         Vector3D radii1 = new Vector3D(radiusX1, radiusY1, radiusZ1);
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radii1.getX(), radii1.getY(), radii1.getZ());

         Vector3D radii2 = new Vector3D();
         radii2.add(radii1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon));
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radii2.getX(), radii2.getY(), radii2.getZ());
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);

         radii2.add(radii1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon));
         ellipsoid2 = new Ellipsoid3D(pose, radii2.getX(), radii2.getY(), radii2.getZ());
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, epsilon) == radii1.geometricallyEquals(radii2, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Ellipsoids are equal if translations are equal within +- epsilon and are otherwise the same
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON);
         secondEllipsoid.appendTranslation(translation);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON);
         secondEllipsoid.appendTranslation(translation);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Ellipsoids are equal if translations are equal within +- epsilon and are otherwise the same
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();

         Ellipsoid3D firstEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         Ellipsoid3D secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON);
         secondEllipsoid.appendTranslation(translation);

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);

         secondEllipsoid = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON);
         secondEllipsoid.appendTranslation(translation);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if the three radii are equal, the rotation does not matter as we are dealing with spheres.
         double radius = random.nextDouble();
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);
         RigidBodyTransform pose2 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radius, radius, radius);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radius, radius, radius);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 0.99 * EPSILON, radius + 0.99 * EPSILON, radius + 0.99 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.19 * EPSILON, radius + 0.99 * EPSILON, radius + 0.79 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 0.99 * EPSILON, radius + 1.19 * EPSILON, radius + 0.79 * EPSILON);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.01 * EPSILON, radius + 1.01 * EPSILON, radius + 1.01 * EPSILON);
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.setRadii(radius + 1.11 * EPSILON, radius + 1.01 * EPSILON, radius + 0.91 * EPSILON);
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // The first ellipsoid is a sphere, ensuring that the second is tested before assuming it is also a sphere.
         double radius = random.nextDouble();
         Point3D position = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         RigidBodyTransform pose1 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);
         RigidBodyTransform pose2 = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), position);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radius, radius, radius);

         Vector3D radii = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         radii.absolute();
         double average = (radii.getX() + radii.getY() + radii.getZ()) / 3.0;
         radii.scale(average / radii.length());

         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radii.getX(), radii.getY(), radii.getZ());

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         radii = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         radii.setX(radii.getY());
         radii.absolute();
         average = (radii.getX() + radii.getY() + radii.getZ()) / 3.0;
         radii.scale(average / radii.length());

         ellipsoid2 = new Ellipsoid3D(pose2, radii.getX(), radii.getY(), radii.getZ());

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii X and Y are equal, rotations around Z and 180 degree flip around any axis orthogonal to Z do not affect the method.
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));
         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii X and Z are equal, rotations around Y and 180 degree flip around any axis orthogonal to Y do not affect the method.
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Show that if two of the radii are equal, the rotation around the third axis is negligible.
        // Radii Y and Z are equal, rotations around X and 180 degree flip around any axis orthogonal to X do not affect the method.
         double radiusX = random.nextDouble();
         double radiusYZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test 180 degree flips around the x, y, or z axis
         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusX, radiusY, radiusZ);

         int axis = random.nextInt(3);
         switch (axis)
         {
         case 0:
            ellipsoid2.appendRollRotation(Math.PI);
            break;
         case 1:
            ellipsoid2.appendPitchRotation(Math.PI);
            break;
         case 2:
            ellipsoid2.appendYawRotation(Math.PI);
            break;
         default:
            throw new RuntimeException("Unexpected axis value: " + axis);
         }

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         double angle = EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI / 2.0);
         switch (axis)
         {
         case 0:
            ellipsoid2.appendRollRotation(angle);
            break;
         case 1:
            ellipsoid2.appendPitchRotation(angle);
            break;
         case 2:
            ellipsoid2.appendYawRotation(angle);
            break;
         default:
            throw new RuntimeException("Unexpected axis value: " + axis);
         }

         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping X <-> Y and adding a 90-degree rotation around Z
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double heightZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, heightZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusY, radiusX, heightZ);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping X <-> Z and adding a 90-degree rotation around Y
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusZ, radiusY, radiusX);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Swapping two of the radii components and adding a 90-degree rotation such that the two ellipsoids should represent the same geometry
        // Swapping Y <-> Z and adding a 90-degree rotation around X
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendRollRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusX, radiusZ, radiusY);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three radii components and adding a 90-degree rotations such that the two ellipsoids should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendPitchRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusZ, radiusX, radiusY);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Permuting the three radii components and adding a 90-degree rotations such that the two ellipsoids should represent the same geometry
         RigidBodyTransform pose1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pose2 = new RigidBodyTransform(pose1);
         pose2.appendYawRotation(Math.PI / 2.0);
         pose2.appendRollRotation(Math.PI / 2.0);

         double radiusX = random.nextDouble();
         double radiusY = random.nextDouble();
         double radiusZ = random.nextDouble();
         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose1, radiusX, radiusY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose2, radiusY, radiusZ, radiusX);

         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Y are equal.
        // 2- Swap X <-> Z
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusZ, radiusXY, radiusXY);
         ellipsoid2.appendPitchRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Y are equal.
        // 2- Swap Y <-> Z
         double radiusXY = random.nextDouble();
         double radiusZ = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXY, radiusXY, radiusZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXY, radiusZ, radiusXY);
         ellipsoid2.appendRollRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Z are equal.
        // 2- Swap X <-> Y
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusY, radiusXZ, radiusXZ);
         ellipsoid2.appendYawRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendRollRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(1.0, 0.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii X and Z are equal.
        // 2- Swap Y <-> Z
         double radiusXZ = random.nextDouble();
         double radiusY = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusXZ, radiusY, radiusXZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusXZ, radiusXZ, radiusY);
         ellipsoid2.appendRollRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii Y and Z are equal.
        // 2- Swap X <-> Y
         double radiusYZ = random.nextDouble();
         double radiusX = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusYZ, radiusX, radiusYZ);
         ellipsoid2.appendYawRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendPitchRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 1.0, 0.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Combining scenarios: two radii equal and swapping radii components.
        // 1- Radii Y and Z are equal.
        // 2- Swap X <-> Z
         double radiusYZ = random.nextDouble();
         double radiusX = random.nextDouble();

         RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Ellipsoid3D ellipsoid1 = new Ellipsoid3D(pose, radiusX, radiusYZ, radiusYZ);
         Ellipsoid3D ellipsoid2 = new Ellipsoid3D(pose, radiusYZ, radiusYZ, radiusX);
         ellipsoid2.appendPitchRotation(Math.PI / 2.0);
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         ellipsoid2.appendYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0.0, 0.0, 1.0), true);
         AxisAngle axisAngle = new AxisAngle(rotationAxis, Math.PI);

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertTrue(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);

         axisAngle = new AxisAngle(rotationAxis, EuclidCoreRandomTools.nextDouble(random, 0.1, Math.PI - 0.1));

         ellipsoid2.appendTransform(new RigidBodyTransform(axisAngle, new Vector3D()));
         assertFalse(ellipsoid1.geometricallyEquals(ellipsoid2, EPSILON), "Iteration: " + i);
      }
   }

   private RigidBodyTransform randomTransform(Random random)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(2 * Math.PI * random.nextDouble());
      tempTransform.setRotationPitchAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);
      tempTransform.setRotationYawAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);

      double[] matrix = new double[16];
      transform.get(matrix);
      matrix[3] = random.nextDouble();
      matrix[7] = random.nextDouble();
      matrix[11] = random.nextDouble();

      return transform;
   }
}

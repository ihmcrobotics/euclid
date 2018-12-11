package us.ihmc.euclid.shape;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Torus3DTest
{
   private static final double MIN_THICKNESS = 0.005;
   private static final double EPSILON = 0.0001;

   @Test
   public void testExapleUsage()
   {
      double radius = 1.0;
      double thickness = 0.1;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 2.0);
      transform.setTranslation(new Vector3D(2.0, 0.0, 3.0));

      Torus3D torus3d = new Torus3D(transform, radius, thickness);
      Point3D pointToCheck = new Point3D(2.0, 0.0, 4.0);

      assertTrue(torus3d.isInsideOrOnSurface(pointToCheck));
   }

   @Test
   public void testSimplePointOnOrInside()
   {
      double radius = 1.0;
      double thickness = 0.1;

      Torus3D torus3d = new Torus3D(radius, thickness);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, 0.0);
   }

   @Test
   public void testTranslatedPointsOnOrInside()
   {
      double radius = 1.0;
      double thickness = 0.1;

      testPointsInsideWhenTranslated(radius, thickness);
   }

   @Test
   public void testPointsInsideRandomSizesAndThicknesses()
   {
      Random random = new Random(1888L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] radiusAndThickness = getRandomRadiusAndThickness(random);

         //         System.out.println("Torus3dTest:testRandomSizes: radius=" + radius + ", thickness=" + thickness);

         double radius = radiusAndThickness[0];
         double thickness = radiusAndThickness[1];

         Torus3D torus3d = new Torus3D(radius, thickness);
         testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, 0.0);

         testTranslatedPointsOnOrInside();
         testPointsInsideWhenTranslated(radius, thickness);
      }
   }

   @Test
   public void testOrthogonalProjection()
   {
      double radius = 1.0;
      double thickness = 0.1;

      Torus3D torus3d = new Torus3D(radius, thickness);

      Point3D testPoint = new Point3D(1.0, 0.0, 0.0);
      Point3D projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);

      EuclidCoreTestTools.assertTuple3DEquals(testPoint, projectedPoint, 1e-7);

      testPoint = new Point3D(-1.09, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);

      EuclidCoreTestTools.assertTuple3DEquals(testPoint, projectedPoint, 1e-7);

      testPoint = new Point3D(radius + 0.2, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      Point3D expectedProjectedPoint = new Point3D(radius + thickness, 0.0, 0.0);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-7);

      double amountPastCenter = 1.7;
      testPoint = new Point3D(radius + amountPastCenter, 0.0, amountPastCenter);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      expectedProjectedPoint = new Point3D(radius + thickness * Math.sqrt(2.0) / 2.0, 0.0, thickness * Math.sqrt(2.0) / 2.0);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-7);

      // Middle of torus should project to anywhere on the inside ring.
      testPoint = new Point3D(0.0, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);

      assertEqualsDelta(radius - thickness, testPoint.distance(projectedPoint), 1e-7);
      assertEqualsDelta(0.0, projectedPoint.getZ(), 1e-7);
   }

   @Test
   public void testClosestPointAndNormalAt()
   {
      // also tests orthogonalProjection() and surfaceNormalAt()
      double radius = 1.0;
      double thickness = 0.1;
      Torus3D torus3d = new Torus3D(radius, thickness);
      Point3D closestPointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();

      // any value on the axis of the origin of the torus is indeterminate, but will be treated as being at (R, 0, 0).
      //X
      // on the outside edge
      Point3D pointInWorldToCheck = new Point3D(radius + thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius + thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(radius + thickness + EPSILON, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius + thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      // on the outside edge
      pointInWorldToCheck = new Point3D(-(radius + thickness), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-(radius + thickness), 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(-(radius + thickness + EPSILON), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-(radius + thickness), 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // on the inner edge
      pointInWorldToCheck = new Point3D(radius - thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius - thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // beyond the inner edge
      pointInWorldToCheck = new Point3D(radius - (thickness + EPSILON), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius - thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // on the inner edge
      pointInWorldToCheck = new Point3D(-radius + thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius + thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      // beyond the inner edge
      pointInWorldToCheck = new Point3D(-radius + (thickness + EPSILON), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius + thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      //Y
      // on the outside edge
      pointInWorldToCheck = new Point3D(0.0, radius + thickness, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, radius + thickness, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 1.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(0.0, radius + thickness + EPSILON, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, radius + thickness, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 1.0, 0.0), 10e-7));

      // on the outside edge
      pointInWorldToCheck = new Point3D(0.0, -(radius + thickness), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, -(radius + thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, -1.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(0.0, -(radius + thickness + EPSILON), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, -(radius + thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, -1.0, 0.0), 10e-7));

      // Z at X=radius
      pointInWorldToCheck = new Point3D(radius, 0.0, thickness);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius, 0.0, thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, 1.0), 10e-7));

      pointInWorldToCheck = new Point3D(radius, 0.0, thickness + EPSILON);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius, 0.0, thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, 1.0), 10e-7));

      pointInWorldToCheck = new Point3D(-radius, 0.0, -thickness);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius, 0.0, -thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, -1.0), 10e-7));

      pointInWorldToCheck = new Point3D(-radius, 0.0, -(thickness + EPSILON));
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius, 0.0, -thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, -1.0), 10e-7));
   }

   @Test
   public void test90DegRotation()
   {
      Random random = new Random(1972L);

      for (int n = 0; n < ITERATIONS; n++)
      {
         double[] radiusAndThickness = getRandomRadiusAndThickness(random);
         double radius = radiusAndThickness[0];
         double thickness = radiusAndThickness[1];

         Torus3D torus3d = new Torus3D(radius, thickness);

         // center point should always be false
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

         RigidBodyTransform transform = new RigidBodyTransform();
         RotationMatrix rotation = new RotationMatrix();

         // test rotation about x-axis of pi/2
         rotation.setToRollMatrix(Math.PI / 2);
         transform.setRotation(rotation);
         torus3d.setPose(transform);

         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(radius, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(radius, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, radius)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, -radius)));

         // test rotation about y-axis of pi/2
         rotation.setToPitchMatrix(Math.PI / 2);
         transform.setRotation(rotation);
         torus3d.setPose(transform);

         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, radius, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, radius, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, radius)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, -radius)));

      }
   }

   @Test
   public void testSimpleRotations()
   {
      Random random = new Random(1984L);

      double radius = 1.0;
      double thickness = 0.1;

      for (int n = 0; n < ITERATIONS; n++)
      {
         while (true)
         {
            double[] radiusAndThickness = getRandomRadiusAndThickness(random);
            radius = radiusAndThickness[0];
            thickness = radiusAndThickness[1];

            thickness = thickness * Math.abs(Math.cos(Math.PI / 3)) - 0.01; // limit thickness of torus so that edges do not accidentally intersect non-rotated axes
            if (thickness >= MIN_THICKNESS && radius > MIN_THICKNESS + EPSILON)
               break;
         }

         Torus3D torus3d = new Torus3D(radius, thickness);

         // center point should always be false
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

         RigidBodyTransform transform = new RigidBodyTransform();
         RotationMatrix rotation = new RotationMatrix();

         // loop to test rotations of pi/3, pi/4, and pi/6 about x and y-axes respectively
         double[] angles = new double[] {Math.PI / 3, Math.PI / 4, Math.PI / 6, 2 * Math.PI / 3, 3 * Math.PI / 4, 5 * Math.PI / 6};

         for (double angle : angles)
         {
            //System.out.println("Radius = " + radius + " thickness = " + thickness);

            torus3d.set(new Torus3D(radius, thickness));

            assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

            for (int i = 0; i < 2; i++)
            {
               if (i == 0)
               {
                  rotation.setToRollMatrix(angle);
                  //                  System.out.println("Rotating " + angle + " rads about X");
               }
               if (i == 1)
               {
                  rotation.setToPitchMatrix(angle);
                  //                  System.out.println("Rotating " + angle + " rads about Y");
               }

               transform.setRotation(rotation);
               torus3d.setPose(transform);

               for (int j = -1; j < 2; j++)
               {
                  double[] xyzToTest = new double[] {0.0, 0.0, 0.0};
                  xyzToTest[i] = radius + j * thickness; // testing radius - thickness, radius, and radius + thickness

                  //                  for (double point : xyzToTest)
                  //                     System.out.print(point + " ");
                  //                  System.out.print("\n");

                  // only points along rotating axis should not move
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[0], xyzToTest[1], xyzToTest[2])));
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[0], -xyzToTest[1], -xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[1], xyzToTest[0], xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[1], -xyzToTest[0], -xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[1], xyzToTest[2], xyzToTest[0])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[1], -xyzToTest[2], -xyzToTest[0])));

                  xyzToTest[i] = 0.0;

                  if (i == 0)
                     xyzToTest[1] = (radius + j * thickness) * Math.cos(angle);
                  if (i == 1)
                     xyzToTest[0] = (radius + j * thickness) * Math.cos(angle);

                  xyzToTest[2] = (radius + j * thickness) * Math.sin(angle) * (i == 1 ? -1 : 1);

                  //                  for (double point : xyzToTest)
                  //                     System.out.print(point + " ");
                  //                  System.out.print("\n");

                  // points along non-rotated axis should be displaced in 2 dimensions
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[0], xyzToTest[1], xyzToTest[2])));
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[0], -xyzToTest[1], -xyzToTest[2])));
               }
            }
         }
      }
   }

   private double[] getRandomRadiusAndThickness(Random random)
   {
      double radius = random.nextDouble() * 10.0;
      while (radius <= MIN_THICKNESS + EPSILON)
      {
         radius = random.nextDouble() * 10.0;
      }

      double thickness = random.nextDouble() * radius;
      while (thickness < MIN_THICKNESS || radius - thickness <= EPSILON)
      {
         thickness = random.nextDouble() * radius;
      }

      return new double[] {radius, thickness};
   }

   public void testPointsInsideWhenTranslated(double radius, double thickness)
   {
      Torus3D torus3d = new Torus3D(radius, thickness);

      Random random = new Random(1892L);
      double translation = (random.nextDouble() - 0.5) * 100.0;

      RigidBodyTransform transform = new RigidBodyTransform();

      transform.setTranslation(new Vector3D(translation, 0.0, 0.0));
      torus3d.setPose(transform);
      testPointsInsideWhenOffsetBy(torus3d, translation, 0.0, 0.0);

      translation = (random.nextDouble() - 0.5) * 10.0;
      transform.setTranslation(new Vector3D(0.0, translation, 0.0));
      torus3d.setPose(transform);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, translation, 0.0);

      translation = (random.nextDouble() - 0.5) * 10.0;
      //      System.out.println("Torus3dTest:testTranslatedPointOnOrInside:" + "0,0," + translation);
      transform.setTranslation(new Vector3D(0.0, 0.0, translation));
      torus3d.setPose(transform);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, translation);

      translation = (random.nextDouble() - 0.5) * 10.0;
      double translationY = (random.nextDouble() - 0.5) * 10.0;
      double translationZ = (random.nextDouble() - 0.5) * 10.0;
      //      System.out.println("Torus3dTest:testTranslatedPointOnOrInside:" + translation + "," + translationY + "," + translationZ);
      transform.setTranslation(new Vector3D(translation, translationY, translationZ));
      torus3d.setPose(transform);
      testPointsInsideWhenOffsetBy(torus3d, translation, translationY, translationZ);
   }

   public void testPointsInsideWhenOffsetBy(Torus3D torus3d, double tx, double ty, double tz)
   {
      double radius = torus3d.getRadius();
      double thickness = torus3d.getTubeRadius();

      //      System.out.println("Testing points for offsets " + tx + "," + ty + "," + tz + " for torus : " + torus3d);
      // center point is false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz)));

      // left, right, top and bottom at the radius are true
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx + radius, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty + radius, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx - radius, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty - radius, tz)));

      // Z direction at the radius is false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz + radius)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz - radius)));

      // on the left, right, top, and bottom outside surface edges
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx + radius + thickness, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty + radius + thickness, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx - (radius + thickness), ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty - (radius + thickness), tz)));

      // Z direction at the center is always false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz + radius + thickness)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz - (radius + thickness))));

      // At radius in X or Y and thickness in Z should be true
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx + radius, ty, tz + thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx - radius, ty, tz + thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx + radius, ty, tz - thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx - radius, ty, tz - thickness)));

      // left, right, top, bottom just beyond the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx + radius + thickness + EPSILON, ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty + radius + thickness + EPSILON, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx - (radius + thickness + EPSILON), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty - (radius + thickness + EPSILON), tz)));

      // at radius, but Z just beyond the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx + radius, ty, tz + thickness + EPSILON)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx + radius, ty, tz - (thickness + EPSILON))));

      // left, right, top, bottom just inside the inner ring of the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx + radius - (thickness + EPSILON), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty + radius - (thickness + EPSILON), tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx - (radius - (thickness + EPSILON)), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty - (radius - (thickness + EPSILON)), tz)));
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89725L);
      Torus3D firstTorus, secondTorus;
      RigidBodyTransform pose;
      double radius, tubeRadius;
      double epsilon = 1e-7;

      pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      radius = 1.0 + random.nextDouble();
      tubeRadius = random.nextDouble();

      firstTorus = new Torus3D(pose, radius, tubeRadius);
      secondTorus = new Torus3D(pose, radius, tubeRadius);

      assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      assertTrue(secondTorus.geometricallyEquals(firstTorus, epsilon));
      assertTrue(firstTorus.geometricallyEquals(firstTorus, epsilon));
      assertTrue(secondTorus.geometricallyEquals(secondTorus, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii do not represent the same geometry object
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(pose, radius, tubeRadius);
         secondTorus = new Torus3D(pose, radius, tubeRadius);

         secondTorus.appendTransform(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii within +- epsilon are equal
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(pose, radius, tubeRadius);

         secondTorus = new Torus3D(pose, radius + 0.99 * epsilon, tubeRadius);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));

         secondTorus = new Torus3D(pose, radius, tubeRadius + 0.99 * epsilon);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii outside of +- epsilon are not equal
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(pose, radius, tubeRadius);

         secondTorus = new Torus3D(pose, radius + 1.01 * epsilon, tubeRadius);

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));

         secondTorus = new Torus3D(pose, radius, tubeRadius + 1.01 * epsilon);

         assertFalse(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii are equal if in the exact same position, but one is upside-down (w.r.t. reference frame)
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(pose, radius, tubeRadius);
         secondTorus = new Torus3D(pose, radius, tubeRadius);

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, Axis.Z, true);
         secondTorus.appendTransform(new RigidBodyTransform(new AxisAngle(rotationAxis, Math.PI), new Vector3D()));

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Torii are equal if yaw differs, but they otherwise represent the same geometry object
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         radius = 1.0 + random.nextDouble();
         tubeRadius = random.nextDouble();

         firstTorus = new Torus3D(pose, radius, tubeRadius);
         secondTorus = new Torus3D(pose, radius, tubeRadius);

         secondTorus.appendYawRotation(2.0 * Math.PI * random.nextDouble());

         assertTrue(firstTorus.geometricallyEquals(secondTorus, epsilon));
      }
   }

   @Test
   public void testIndependenceOfCopiedTransforms()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 6);
      Torus3D torus = new Torus3D(transform, 7.0, 2.0);

      Torus3D torusCopy = new Torus3D(torus);
      RigidBodyTransform transformAppliedOnlyToCopy = new RigidBodyTransform();
      transformAppliedOnlyToCopy.setRotationPitchAndZeroTranslation(Math.PI / 4);
      torusCopy.applyTransform(transformAppliedOnlyToCopy);
      assertFalse(torusCopy.equals(torus));

      Torus3D torusCopyBySet = new Torus3D(5.0, 1.0);
      torusCopyBySet.set(torus);
      RigidBodyTransform transformAppliedOnlyToCopyBySet = new RigidBodyTransform();
      transformAppliedOnlyToCopyBySet.setRotationYawAndZeroTranslation(Math.PI / 5);
      torusCopyBySet.applyTransform(transformAppliedOnlyToCopyBySet);
      assertFalse(torusCopyBySet.equals(torus));
   }
}

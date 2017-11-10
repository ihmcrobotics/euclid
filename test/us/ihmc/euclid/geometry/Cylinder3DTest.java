package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Cylinder3DTest
{
   double eps = 1e-14;
   int iterations = 100;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double height = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);

         testHelper.runSimpleTests(cylinder3d, random, numberOfPoints);
      }
   }

   @Test
   public void testCommonShape3dFunctionality_2()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double height = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);
         Cylinder3D cylinder3d2 = new Cylinder3D(cylinder3d);
         testHelper.runSimpleTests(cylinder3d2, random, numberOfPoints);
      }
   }

   @Test
   public void testGettersAndSetters()
   {
      Random random = new Random(1776L);

      int numberOfShapes = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double height = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = EuclidCoreRandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);

         assertEquals(cylinder3d.getRadius(), radius, 1e-7);
         assertEquals(cylinder3d.getHeight(), height, 1e-7);

         cylinder3d.setRadius(5.0);
         cylinder3d.setHeight(10.0);

         assertEquals(cylinder3d.getRadius(), 5.0, 1e-7);
         assertEquals(cylinder3d.getHeight(), 10.0, 1e-7);

         RigidBodyTransform rbt = new RigidBodyTransform();
         cylinder3d.getPose(rbt);

         Point3D point = new Point3D();
         rbt.getTranslation(point);

         Point3D point1 = new Point3D();
         transform.getTranslation(point1);

         assertEquals(point.getX(), point1.getX(), 1e-7);
         assertEquals(point.getY(), point1.getY(), 1e-7);
         assertEquals(point.getZ(), point1.getZ(), 1e-7);

         Quaternion quat1 = new Quaternion();
         rbt.getRotation(quat1);

         Quaternion quat2 = new Quaternion();
         transform.getRotation(quat2);

         assertEquals(quat1.getX(), quat2.getX(), 1e-7);
         assertEquals(quat1.getY(), quat2.getY(), 1e-7);
         assertEquals(quat1.getZ(), quat2.getZ(), 1e-7);
         assertEquals(quat1.getS(), quat2.getS(), 1e-7);
      }
   }

   @Test
   public void testWithNoTransform()
   {
      double height = 2.0;
      double radius = 0.2;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);

      Point3D pointToCheck = new Point3D(0.0, 0.0, 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(0.0, 0.0, -0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(0.0, 0.0, (height / 2.0) + 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(0.0, 0.0, -(height / 2.0) + 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(radius + 0.001, 0.0, height / 4.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(radius - 0.001, 0.0, height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(0.0, radius + 0.001, height / 4.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(0.0, radius - 0.001, height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(radius / 2.0, radius / 2.0, height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));
   }

   @Test
   public void testWithTransform()
   {
      // TODO: More tests, rotations.

      double height = 2.0;
      double radius = 0.2;
      RigidBodyTransform transform = new RigidBodyTransform();
      double translateX = 1.1;
      double translateY = 1.3;
      double translateZ = 1.4;
      transform.setTranslation(new Vector3D(translateX, translateY, translateZ));

      Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);

      Point3D pointToCheck = new Point3D(translateX, translateY, translateZ - (height / 2.0) + 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ - (height / 2.0) - 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ + (height / 2.0) + 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ + (height / 2.0) - 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX + radius + 0.001, translateY, translateZ + height / 4.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX + radius - 0.001, translateY, translateZ + height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY + radius + 0.001, translateZ + height / 4.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY + radius - 0.001, translateZ + height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX + radius / 2.0, translateY + radius / 2.0, translateZ + height / 4.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));
   }

   @Test
   public void testOrthogonalProjectionSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Point3D pointToCheckAndPack = new Point3D(1, 1, 1);

      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      double xy = Math.sqrt(radius * radius / 2);
      Point3D expectedProjection = new Point3D(xy, xy, 1);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   @Test
   public void testOrthogonalProjectionTop()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Point3D pointToCheckAndPack = new Point3D(0.5, 0.25, 1.5);

      Point3D expectedProjection = new Point3D(pointToCheckAndPack.getX(), pointToCheckAndPack.getY(), (height / 2.0));
      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   @Test
   public void testOrthogonalProjectionBottom()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Point3D pointToCheckAndPack = new Point3D(0.5, 0.25, -height);

      Point3D expectedProjection = new Point3D(pointToCheckAndPack.getX(), pointToCheckAndPack.getY(), -1.0);
      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   private void assertPointEquals(Point3D expectedPoint, Point3D actualPoint)
   {
      String failMessage = "Expected <(" + expectedPoint.getX() + "," + expectedPoint.getY() + "," + expectedPoint.getZ() + ")>, but was <("
            + actualPoint.getX() + "," + actualPoint.getY() + "," + actualPoint.getZ() + ")>";
      assertEquals(failMessage, expectedPoint.getX(), actualPoint.getX(), eps);
      assertEquals(failMessage, expectedPoint.getY(), actualPoint.getY(), eps);
      assertEquals(failMessage, expectedPoint.getZ(), actualPoint.getZ(), eps);
   }

   @Test
   public void testSurfaceNormalAt_OnSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(1, 0, 1);
      Vector3D expectedNormal = new Vector3D(1, 0, 0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_inSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(0.5, 0, 0.5);
      Vector3D expectedNormal = new Vector3D(1, 0, 0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_outSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(1.5, 0, 1);
      Vector3D expectedNormal = new Vector3D(1, 0, 0);
      assertFalse(cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack));
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_above()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(0, 0, 3.5);
      Vector3D expectedNormal = new Vector3D(0, 0, 1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_below()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(0, 0, -1);
      Vector3D expectedNormal = new Vector3D(0, 0, -1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_upAndToSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(0.0, 2.0, 2.0);
      Vector3D expectedNormal = new Vector3D(0.0, 1.0, 1.0);
      expectedNormal.normalize();
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testSurfaceNormalAt_above_translated()
   {
      double height = 2.0;
      double radius = 1.0;
      RigidBodyTransform transform = new RigidBodyTransform();
      double tx = 1.5, ty = 1, tz = 2;
      Vector3D translation = new Vector3D(tx, ty, tz);
      transform.setTranslationAndIdentityRotation(translation);
      Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(tx, ty, tz + 3.0);
      Vector3D expectedNormal = new Vector3D(0, 0, 1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @Test
   public void testCheckInside()
   {
      double height = 2.0;
      double radius = 1.0;
      RigidBodyTransform transform = new RigidBodyTransform();
      double tx = 1.5, ty = 1, tz = 2;
      Vector3D translation = new Vector3D(tx, ty, tz);
      transform.setTranslationAndIdentityRotation(translation);
      Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);
      Vector3D normalToPack = new Vector3D();
      Point3D closestPointToPack = new Point3D();

      Point3D pointToCheck = new Point3D(tx, ty, tz + 3.0);
      Vector3D expectedNormal = new Vector3D(0, 0, 1.0);
      boolean isInside = cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, normalToPack, 1e-7);
      assertFalse(isInside);

      pointToCheck.set(tx, ty, tz + (height / 2.0) - 0.011);
      isInside = cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertTrue(isInside);
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, normalToPack, 1e-7);
   }
   
   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(12653L);
      double epsilon = 1e-7;
      Cylinder3D firstCylinder, secondCylinder;
      RigidBodyTransform rbt;
      double height, radius;
      
      for (int i = 0; i < iterations; ++i) {
         height = random.nextDouble();
         radius = random.nextDouble();
         rbt = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         
         firstCylinder = new Cylinder3D(rbt, height, radius);
         secondCylinder = new Cylinder3D(rbt, height, radius);
         
         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
         
         secondCylinder.setHeight(height + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setHeight(height + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }
      
      for (int i = 0; i < iterations; ++i) {
         height = random.nextDouble();
         radius = random.nextDouble();
         rbt = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         
         firstCylinder = new Cylinder3D(rbt, height, radius);
         secondCylinder = new Cylinder3D(rbt, height, radius);
         
         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
         
         secondCylinder.setRadius(radius + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setRadius(radius  + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }
      
      for (int i = 0; i < iterations; ++i) {
         height = random.nextDouble();
         radius = random.nextDouble();
         //rbt = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         rbt = new RigidBodyTransform();
         
         firstCylinder = new Cylinder3D(rbt, height, radius);
         secondCylinder = new Cylinder3D(rbt, height, radius);
         
         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
         
         secondCylinder.appendTransform(new RigidBodyTransform(new AxisAngle(1.0, 0.0, 0.0, Math.PI), new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
         
         secondCylinder = new Cylinder3D(rbt, height, radius);

         secondCylinder.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI), new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
         
         secondCylinder = new Cylinder3D(rbt, height, radius);

         secondCylinder.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 0.0, 1.0, Math.PI), new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      height = random.nextDouble();
      radius = random.nextDouble();
      rbt = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      firstCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder = new Cylinder3D(rbt, height, radius);
      
      assertTrue(firstCylinder.geometricallyEquals(firstCylinder, epsilon));
      assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,0.5 * epsilon,0,1,0,0,0,0,1,0));
      assertTrue(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      secondCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,0,0,1,0,0.5 * epsilon,0,0,1,0));
      assertTrue(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      secondCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,0,0,1,0,0,0,0,1,0.5 * epsilon));
      assertTrue(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      
      secondCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,1.5 * epsilon,0,1,0,0,0,0,1,0));
      assertFalse(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      secondCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,0,0,1,0,1.5 * epsilon,0,0,1,0));
      assertFalse(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      secondCylinder = new Cylinder3D(rbt, height, radius);
      secondCylinder.applyTransform(new RigidBodyTransform(1,0,0,0,0,1,0,0,0,0,1,1.5 * epsilon));
      assertFalse(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
   }
}

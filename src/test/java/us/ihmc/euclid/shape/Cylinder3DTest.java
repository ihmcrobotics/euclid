package us.ihmc.euclid.shape;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Cylinder3DTest
{
   private static final double EPSILON = 1e-12;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         testHelper.runSimpleTests(EuclidShapeRandomTools.nextCylinder3D(random), random, numberOfPoints);
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
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double height = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
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
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double height = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double radius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         Cylinder3D cylinder3d = new Cylinder3D(transform, height, radius);

         assertEquals(cylinder3d.getRadius(), radius, 1e-7);
         assertEquals(cylinder3d.getLength(), height, 1e-7);

         cylinder3d.setRadius(5.0);
         cylinder3d.setLength(10.0);

         assertEquals(cylinder3d.getRadius(), 5.0, 1e-7);
         assertEquals(cylinder3d.getLength(), 10.0, 1e-7);

         RigidBodyTransform rbt = new RigidBodyTransform();
         cylinder3d.getPose(rbt);

         Point3D point = new Point3D();
         rbt.getTranslation(point);

         Point3D point1 = new Point3D();
         transform.getTranslation(point1);

         EuclidCoreTestTools.assertTuple3DEquals(point, point1, 1e-7);

         Quaternion quat1 = new Quaternion();
         rbt.getRotation(quat1);

         Quaternion quat2 = new Quaternion();
         transform.getRotation(quat2);

         EuclidCoreTestTools.assertQuaternionEquals(quat1, quat2, 1e-7);
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

      pointToCheck = new Point3D(0.0, 0.0, height / 2.0 + 0.0001);
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

      Point3D pointToCheck = new Point3D(translateX, translateY, translateZ - height / 2.0 + 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ - height / 2.0 - 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ + height / 2.0 + 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3D(translateX, translateY, translateZ + height / 2.0 - 0.0001);
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
      Point3D pointToCheckAndPack = new Point3D(5, 5, 1);

      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      double xy = Math.sqrt(radius * radius / 2);
      Point3D expectedProjection = new Point3D(xy, xy, 1);
      EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, pointToCheckAndPack, EPSILON);
   }

   @Test
   public void testOrthogonalProjectionTop()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3D cylinder3d = new Cylinder3D(height, radius);
      Point3D pointToCheckAndPack = new Point3D(0.5, 0.25, 1.5);

      Point3D expectedProjection = new Point3D(pointToCheckAndPack.getX(), pointToCheckAndPack.getY(), height / 2.0);
      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, pointToCheckAndPack, EPSILON);
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
      EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, pointToCheckAndPack, EPSILON);
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      assertFalse(cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack));
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
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
      boolean isInside = cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, normalToPack, 1e-7);
      assertFalse(isInside);

      pointToCheck.set(tx, ty, tz + height / 2.0 - 0.011);
      isInside = cylinder3d.doPoint3DCollisionTest(pointToCheck, closestPointToPack, normalToPack);
      assertTrue(isInside);
      EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, normalToPack, 1e-7);
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(12653L);
      double epsilon = 1e-7;
      Cylinder3D firstCylinder, secondCylinder;
      RigidBodyTransform pose;
      double height, radius;
      Vector3D translation;

      height = random.nextDouble();
      radius = random.nextDouble();
      pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      firstCylinder = new Cylinder3D(pose, height, radius);
      secondCylinder = new Cylinder3D(pose, height, radius);

      assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      assertTrue(secondCylinder.geometricallyEquals(firstCylinder, epsilon));
      assertTrue(firstCylinder.geometricallyEquals(firstCylinder, epsilon));
      assertTrue(secondCylinder.geometricallyEquals(secondCylinder, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if heights are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         firstCylinder = new Cylinder3D(pose, height, radius);
         secondCylinder = new Cylinder3D(pose, height, radius);

         secondCylinder.setLength(height + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setLength(height + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if radii are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         firstCylinder = new Cylinder3D(pose, height, radius);
         secondCylinder = new Cylinder3D(pose, height, radius);

         secondCylinder.setRadius(radius + 0.99 * epsilon);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder.setRadius(radius + 1.01 * epsilon);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if aligned opposite on the same axis and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         pose = new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), new Vector3D());

         firstCylinder = new Cylinder3D(pose, height, radius);
         secondCylinder = new Cylinder3D(pose, height, radius);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, new Vector3D(0, 0, 1), true);

         secondCylinder.getPose().multiply(new RigidBodyTransform(new AxisAngle(orthogonalToAxis, Math.PI), new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder = new Cylinder3D(pose, height, radius);

         secondCylinder.getPose().multiply(new RigidBodyTransform(new AxisAngle(new Vector3D(0, 0, 1), EuclidCoreRandomTools.nextDouble(random, Math.PI)),
         new Vector3D()));

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Cylinders are equal if translations are equal within +- epsilon and are otherwise the same
         height = random.nextDouble();
         radius = random.nextDouble();
         pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         firstCylinder = new Cylinder3D(pose, height, radius);
         secondCylinder = new Cylinder3D(pose, height, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);

         secondCylinder.getPose().appendTranslation(translation);

         assertTrue(firstCylinder.geometricallyEquals(secondCylinder, epsilon));

         secondCylinder = new Cylinder3D(pose, height, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);

         secondCylinder.getPose().appendTranslation(translation);

         assertFalse(firstCylinder.geometricallyEquals(secondCylinder, epsilon));
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Cylinder3D cylinder = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = cylinder.getSupportingVertex(supportDirection);
         assertTrue(cylinder.isInsideOrOnSurface(supportingVertex));

         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         assertFalse(cylinder.isInsideOrOnSurface(supportingVertexTranslated));

         Vector3D actualNormal = new Vector3D();
         cylinder.doPoint3DCollisionTest(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(supportDirection, actualNormal, EPSILON);
      }
   }
}

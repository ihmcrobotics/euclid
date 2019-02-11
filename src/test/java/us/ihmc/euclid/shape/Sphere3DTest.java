package us.ihmc.euclid.shape;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Sphere3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         testHelper.runSimpleTests(EuclidShapeRandomTools.nextSphere3D(random), random, numberOfPoints);
      }
   }

   @Test
   public void testSimpleConstructor()
   {
      Sphere3D sphere3d = new Sphere3D();

      double epsilon = 1e-14;
      assertEquals(sphere3d.getRadius(), 1.0, epsilon);
      Point3D centerCheck = new Point3D();
      sphere3d.getPosition(centerCheck);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(), centerCheck, epsilon);
   }

   @Test
   public void testIsInside()
   {
      Sphere3D sphere3d = new Sphere3D();

      double justInside = 0.999;
      double justOutside = 1.001;
      assertTrue(sphere3d.isInsideOrOnSurface(new Point3D(justInside, 0.0, 0.0)));
      assertTrue(sphere3d.isInsideOrOnSurface(new Point3D(0.0, justInside, 0.0)));
      assertTrue(sphere3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, justInside)));

      assertFalse(sphere3d.isInsideOrOnSurface(new Point3D(justOutside, 0.0, 0.0)));
      assertFalse(sphere3d.isInsideOrOnSurface(new Point3D(0.0, justOutside, 0.0)));
      assertFalse(sphere3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, justOutside)));

      assertTrue(sphere3d.isInsideOrOnSurface(new Point3D(Math.sqrt(1.999) / 2.0, Math.sqrt(1.999) / 2.0, 0.0)));
      assertFalse(sphere3d.isInsideOrOnSurface(new Point3D(Math.sqrt(2.001) / 2.0, Math.sqrt(2.001) / 2.0, 0.0)));

   }

   @Test
   public void testOrthogonalProjection()
   {
      Point3D center = new Point3D(1.0, 2.1, 3.2);
      double radius = 0.7634;

      Sphere3D sphere3d = new Sphere3D(center, radius);

      Point3D randomPoint = new Point3D(17.3, 19.2, 11.4);
      Point3D orthogonalProjection = new Point3D(randomPoint);
      sphere3d.orthogonalProjection(orthogonalProjection);

      assertEquals(radius, orthogonalProjection.distance(center), 1e-7);
      Vector3D vector1 = new Vector3D(randomPoint);
      vector1.sub(center);

      Vector3D vector2 = new Vector3D(orthogonalProjection);
      vector2.sub(center);

      vector1.normalize();
      vector2.normalize();

      EuclidCoreTestTools.assertTuple3DEquals(vector1, vector2, 1e-7);
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(34201L);
      Sphere3D firstSphere, secondSphere;
      Point3D center;
      double radius;
      double epsilon = 1e-7;

      center = EuclidCoreRandomTools.nextPoint3D(random);
      radius = random.nextDouble();

      firstSphere = new Sphere3D(center.getX(), center.getY(), center.getZ(), radius);
      secondSphere = new Sphere3D(center.getX(), center.getY(), center.getZ(), radius);

      assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      assertTrue(secondSphere.geometricallyEquals(firstSphere, epsilon));
      assertTrue(firstSphere.geometricallyEquals(firstSphere, epsilon));
      assertTrue(secondSphere.geometricallyEquals(secondSphere, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are equal if radii are equal within +- epsilon and are otherwise the same
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);

         secondSphere = new Sphere3D(center, radius + epsilon * 0.99);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius - epsilon * 0.99);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are not equal if radii outside of +- epsilon
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);

         secondSphere = new Sphere3D(center, radius + epsilon * 1.01);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius - epsilon * 1.01);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are still equal if center, radius, and location are equal but orientations are different
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);
         secondSphere = new Sphere3D(firstSphere);

         secondSphere.appendTransform(new RigidBodyTransform(EuclidCoreRandomTools.nextAxisAngle(random), new Vector3D()));

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Spheres are equal only if translations equal within +- epsilon and otherwise the same
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);
         secondSphere = new Sphere3D(center, radius);

         Vector3D translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);

         secondSphere.appendTranslation(translation);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));

         secondSphere = new Sphere3D(center, radius);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);

         secondSphere.appendTranslation(translation);

         assertFalse(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Rotations should not affect the assertion
         center = EuclidCoreRandomTools.nextPoint3D(random);
         radius = random.nextDouble();

         firstSphere = new Sphere3D(center, radius);
         secondSphere = new Sphere3D(center, radius);

         RigidBodyTransform rotationOnly = new RigidBodyTransform();
         rotationOnly.setRotation(EuclidCoreRandomTools.nextQuaternion(random));

         secondSphere.appendTransform(rotationOnly);

         assertTrue(firstSphere.geometricallyEquals(secondSphere, epsilon));
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D sphere = EuclidShapeRandomTools.nextSphere3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = sphere.getSupportingVertex(supportDirection);
         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         assertTrue(sphere.isInsideOrOnSurface(supportingVertex));
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(sphere.isInsideOrOnSurface(supportingVertexTranslated));
         supportingVertexTranslated.scaleAdd(1.0e-2, supportDirection, supportingVertex);
         Vector3D expectedNormal = new Vector3D();
         expectedNormal.sub(supportingVertexTranslated, supportingVertex);
         expectedNormal.normalize();

         Vector3D actualNormal = new Vector3D();
         sphere.doPoint3DCollisionTest(supportingVertexTranslated, null, actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedNormal, actualNormal, EPSILON);
      }
   }
}

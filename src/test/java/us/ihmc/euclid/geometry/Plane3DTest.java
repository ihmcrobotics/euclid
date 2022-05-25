package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Plane3DTest
{
   @Test
   public void test()
   {
      Point3D point = new Point3D();
      Vector3D normal = new Vector3D(1.0, 2.0, 3.0);
      Plane3D plane = new Plane3D(point, normal);
      assertTrue(point.equals(plane.getPointCopy()));
      assertFalse(point == plane.getPointCopy());
      normal.normalize();
      assertTrue(normal.equals(plane.getNormalCopy()));
      Plane3D plane2 = new Plane3D(point, normal);
      assertTrue(plane2.epsilonEquals(plane, 1e-17));
   }

   @Test
   public void testConstructor()
   {
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(plane.getNormalCopy().getX() == 0.0);
      assertTrue(plane.getNormalCopy().getY() == 0.0);
      assertTrue(plane.getNormalCopy().getZ() == 1.0);
      assertTrue(plane.getPointCopy().getX() == 0.0);
      assertTrue(plane.getPointCopy().getY() == 0.0);
      assertTrue(plane.getPointCopy().getZ() == 0.0);
   }

   @Test
   public void testIsOnOrAbove()
   {
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      Point3D Q = new Point3D(0.0, 0.0, 2.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3D(0.0, 0.0, 0.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3D(0.0, 0.0, -2.0);
      assertFalse(plane.isOnOrAbove(Q));
   }

   @Test
   public void testIsOnOrBelow()
   {
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      Point3D Q = new Point3D(0.0, 0.0, -2.0);
      assertTrue(plane.isOnOrBelow(Q));
      Q = new Point3D(0.0, 0.0, 2.0);
      assertFalse(plane.isOnOrBelow(Q));
   }

   @Test
   public void testOrthogonalProjection()
   {
      Point3D q = new Point3D(1.0, 2.0, -3.0);
      Point3D v = new Point3D(1.0, 2.0, 0.0);
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(v.equals(plane.orthogonalProjectionCopy(q)));

      q.set(3.0, 3.0, -4.0);
      plane.setPoint(1.0, 1.0, 1.0);
      plane.setNormal(2.0, 0.0, 0.0);
      v.set(1.0, 3.0, -4.0);
      assertTrue(v.equals(plane.orthogonalProjectionCopy(q)));
   }

   @Test
   public void testGetZOnPlane()
   {
      Point3D point = new Point3D(1.0, 2.0, -3.0);
      Vector3D normal = new Vector3D(0.2, 1.7, 0.4);
      Plane3D plane = new Plane3D(point, normal);

      double x = 2.33;
      double y = 1.97;

      double z = plane.getZOnPlane(x, y);

      Point3D testPoint = new Point3D(x, y, z);
      assertTrue(plane.distance(testPoint) < 1e-10);

      normal = new Vector3D(0.2, 1.7, 0.0);
      plane = new Plane3D(point, normal);

      z = plane.getZOnPlane(x, y);
      assertTrue(Double.isNaN(z));
   }

   @Test
   public void testDistance()
   {
      Point3D q = new Point3D(1.0, 1.0, 1.0);
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      assertEquals(1.0, plane.distance(q), 1e-14);
   }

   @Test
   public void testApplyTransform()
   {
      RigidBodyTransform transformation = new RigidBodyTransform();
      transformation.setRotationYawAndZeroTranslation(2.3);
      Plane3D plane = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      plane.applyTransform(transformation);
      EuclidCoreTestTools.assertTuple3DEquals(plane.getNormalCopy(), new Vector3D(0.0, 0.0, 1.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane.getPointCopy(), new Point3D(0.0, 0.0, 0.0), 1e-14);

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      Plane3D plane2 = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      plane2.applyTransform(transformation2);
      EuclidCoreTestTools.assertTuple3DEquals(plane2.getNormalCopy(), new Vector3D(0.0, 0.0, 1.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane2.getPointCopy(), new Point3D(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      Plane3D plane3 = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      plane3.applyTransform(transformation3);
      EuclidCoreTestTools.assertTuple3DEquals(plane3.getNormalCopy(), new Vector3D(1.0, 0.0, 0.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane3.getPointCopy(), new Point3D(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation4 = new RigidBodyTransform();
      transformation4.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation4.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(9608123L);
      Plane3D firstPlane, secondPlane;
      Vector3D normal;
      Vector3D translation = new Vector3D();
      double epsilon = 1e-5;

      firstPlane = EuclidGeometryRandomTools.nextPlane3D(random);
      secondPlane = new Plane3D(firstPlane);

      assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon));
      assertTrue(secondPlane.geometricallyEquals(firstPlane, epsilon));
      assertTrue(firstPlane.geometricallyEquals(firstPlane, epsilon));
      assertTrue(secondPlane.geometricallyEquals(secondPlane, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Planes are equal if orientations equal within +- epsilon and are otherwise the same
         firstPlane = EuclidGeometryRandomTools.nextPlane3D(random);
         secondPlane = new Plane3D(firstPlane);

         normal = firstPlane.getNormalCopy();
         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         double rotationAngle = 0.99 * epsilon;
         normal.applyTransform(new RigidBodyTransform(new AxisAngle(rotationAxis, rotationAngle), new Vector3D()));
         secondPlane.setNormal(normal);
         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon));

         normal = firstPlane.getNormalCopy();
         rotationAngle = 1.01 * epsilon;
         normal.applyTransform(new RigidBodyTransform(new AxisAngle(rotationAxis, rotationAngle), new Vector3D()));
         secondPlane.setNormal(normal);
         assertFalse(firstPlane.geometricallyEquals(secondPlane, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Planes are equal if locations equal within +- epsilon and are otherwise the same
         AxisAngle zeroRotation = new AxisAngle();

         firstPlane = EuclidGeometryRandomTools.nextPlane3D(random);

         secondPlane = new Plane3D(firstPlane);
         translation.setAndScale(0.99 * epsilon, firstPlane.getNormal());
         secondPlane.applyTransform(new RigidBodyTransform(zeroRotation, translation));
         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon));

         secondPlane = new Plane3D(firstPlane);
         translation.setAndScale(1.01 * epsilon, firstPlane.getNormal());
         secondPlane.applyTransform(new RigidBodyTransform(zeroRotation, translation));
         assertFalse(firstPlane.geometricallyEquals(secondPlane, epsilon));

         secondPlane = new Plane3D(firstPlane);
         translation.setAndScale(0.99 * epsilon, firstPlane.getNormal());
         secondPlane.applyTransform(new RigidBodyTransform(zeroRotation, translation));
         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon));

         secondPlane = new Plane3D(firstPlane);
         translation.setAndScale(-1.01 * epsilon, firstPlane.getNormal());
         secondPlane.applyTransform(new RigidBodyTransform(zeroRotation, translation));
         assertFalse(firstPlane.geometricallyEquals(secondPlane, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Flip the second plane normal
         firstPlane = EuclidGeometryRandomTools.nextPlane3D(random);
         secondPlane = new Plane3D(firstPlane);
         normal = secondPlane.getNormalCopy();
         normal.negate();
         secondPlane.setNormal(normal);

         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon), "Iteration: " + i);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Translate the point of the second plane orthogonally to the normal
         firstPlane = EuclidGeometryRandomTools.nextPlane3D(random);
         secondPlane = new Plane3D(firstPlane);

         translation = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstPlane.getNormal(), true);
         translation.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon), "Iteration: " + i);

         normal = secondPlane.getNormalCopy();
         normal.negate();
         secondPlane.setNormal(normal);
         assertTrue(firstPlane.geometricallyEquals(secondPlane, epsilon), "Iteration: " + i);
      }
   }
}

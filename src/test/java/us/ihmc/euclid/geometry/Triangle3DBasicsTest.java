package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import java.util.Random;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.interfaces.Triangle3DBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public abstract class Triangle3DBasicsTest<T extends Triangle3DBasics>
{
   public abstract T newEmptyTriangle3D();

   public abstract T newRandomTriangle3D(Random random);

   public abstract T newTriangle3D(Point3DReadOnly A, Point3DReadOnly B, Point3DReadOnly C);

   public abstract double getEpsilon();

   @Test
   public void testSetter()
   {
      Random random = new Random(24323);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T firstTriangle = newRandomTriangle3D(random);
         T secondTriangle = newRandomTriangle3D(random);
         T thirdTriangle = newEmptyTriangle3D();

         //Test set by Triangle3DReadOnly
         assertFalse(firstTriangle.equals(secondTriangle));
         firstTriangle.set(secondTriangle);
         assertEquals(firstTriangle, secondTriangle);

         //Test set by 3 points
         assertFalse(firstTriangle.equals(thirdTriangle));
         thirdTriangle.set(firstTriangle.getA(), firstTriangle.getB(), firstTriangle.getC());
         assertEquals(firstTriangle, thirdTriangle);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Point3D pointA = new Point3D(1.0, 2.0, 3.0);
      Point3D pointB = new Point3D(4.0, 5.0, 6.0);
      Point3D pointC = new Point3D(7.0, 8.0, 9.0);

      T triangle = newTriangle3D(pointA, pointB, pointC);

      //Test setToZero method
      triangle.setToZero();
      assertTrue(0 == triangle.getA().getX());
      assertTrue(0 == triangle.getA().getY());
      assertTrue(0 == triangle.getA().getZ());
      assertTrue(0 == triangle.getB().getX());
      assertTrue(0 == triangle.getB().getY());
      assertTrue(0 == triangle.getB().getZ());
      assertTrue(0 == triangle.getC().getX());
      assertTrue(0 == triangle.getC().getY());
      assertTrue(0 == triangle.getC().getZ());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(32423L);

      //Test setToNaN method
      T triangle = newRandomTriangle3D(random);
      assertFalse(triangle.containsNaN());
      triangle.setToNaN();
      assertTrue(triangle.containsNaN());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(triangle.getA());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(triangle.getB());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(triangle.getC());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Point3D pointA = new Point3D(1.0, 2.0, 3.0);
      Point3D pointB = new Point3D(4.0, 5.0, 6.0);
      Point3D pointC = new Point3D(7.0, 8.0, 9.0);

      T triangle = newEmptyTriangle3D();
      assertFalse(triangle.containsNaN());
      triangle.set(pointA, pointB, pointC);
      assertFalse(triangle.containsNaN());

      //Test containsNan method
      pointA.setX(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointA.setX(0.0);
      pointA.setY(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointA.setY(0.0);
      pointA.setZ(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointA.setZ(1.0);
      pointB.setX(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointB.setX(0.0);
      pointB.setY(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointB.setY(0.0);
      pointB.setZ(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointB.setZ(1.0);
      pointC.setX(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointC.setX(0.0);
      pointC.setY(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointC.setY(0.0);
      pointC.setZ(Double.NaN);
      triangle.set(pointA, pointB, pointC);
      assertTrue(triangle.containsNaN());

      pointC.setZ(1.0);
      triangle.set(pointA, pointB, pointC);
      assertFalse(triangle.containsNaN());
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(25613L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(expected, original);

         transform.transform(expected.getA());
         transform.transform(expected.getB());
         transform.transform(expected.getC());
         assertFalse(expected.equals(original));

         original.applyTransform(transform);
         assertEquals(expected, original);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(expected, original);

         transform.transform(expected.getA());
         transform.transform(expected.getB());
         transform.transform(expected.getC());
         assertFalse(expected.equals(original));

         original.applyTransform(transform);
         assertEquals(expected, original);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(expected, original);

         transform.transform(expected.getA());
         transform.transform(expected.getB());
         transform.transform(expected.getC());
         assertFalse(expected.equals(original));

         original.applyTransform(transform);
         assertEquals(expected, original);
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(original, expected);

         expected.applyTransform(transform);
         assertFalse(expected.equals(original));
         expected.applyInverseTransform(transform);
         assertTrue(expected.epsilonEquals(original, getEpsilon()));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(original, expected);

         expected.applyTransform(transform);
         assertFalse(expected.equals(original));
         expected.applyInverseTransform(transform);
         assertTrue(expected.epsilonEquals(original, getEpsilon()));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = newRandomTriangle3D(random);
         T expected = newEmptyTriangle3D();

         expected.set(original);
         assertEquals(original, expected);

         expected.applyTransform(transform);
         assertFalse(expected.equals(original));
         expected.applyInverseTransform(transform);
         assertTrue(expected.epsilonEquals(original, getEpsilon()));
      }
   }

}

package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Triangle3DTest extends Triangle3DBasicsTest<Triangle3D>
{

   @Override
   public Triangle3D newEmptyTriangle3D()
   {
      // Return new empty Triangle3D()
      return new Triangle3D();
   }

   @Override
   public Triangle3D newRandomTriangle3D(Random random)
   {
      // Return new Random Triangle3D
      Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

      return new Triangle3D(pointA, pointB, pointC);
   }

   @Override
   public Triangle3D newTriangle3D(Point3DReadOnly A, Point3DReadOnly B, Point3DReadOnly C)
   {
      // Return new Triangle3D with points
      return new Triangle3D(A, B, C);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-5;
   }

   @Test
   public void testConstructors()
   {
      Random random = new Random(1000L);

      // Test Triangle3D()
      Triangle3D triangle = new Triangle3D();

      assertTrue(0 == triangle.getA().getX());
      assertTrue(0 == triangle.getA().getY());
      assertTrue(0 == triangle.getA().getZ());
      assertTrue(0 == triangle.getB().getX());
      assertTrue(0 == triangle.getB().getY());
      assertTrue(0 == triangle.getB().getZ());
      assertTrue(0 == triangle.getC().getX());
      assertTrue(0 == triangle.getC().getY());
      assertTrue(0 == triangle.getC().getZ());

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

         // Test Triangle3D(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
         Triangle3D triangle3DbyPointPointPoint = new Triangle3D(pointA, pointB, pointC);

         assertEquals(triangle3DbyPointPointPoint.getA(), pointA);
         assertEquals(triangle3DbyPointPointPoint.getB(), pointB);
         assertEquals(triangle3DbyPointPointPoint.getC(), pointC);

         // Test Triangle3D(Triangle3DReadOnly other) - copy of triangle3DbyPointPointPoint
         Triangle3D triangle3DbyCopy = new Triangle3D(triangle3DbyPointPointPoint);
         assertFalse(triangle3DbyCopy == triangle3DbyPointPointPoint);
         assertEquals(triangle3DbyCopy.getA(), triangle3DbyPointPointPoint.getA());
         assertEquals(triangle3DbyCopy.getB(), triangle3DbyPointPointPoint.getB());
         assertEquals(triangle3DbyCopy.getC(), triangle3DbyPointPointPoint.getC());

      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(1000L);
      Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

      Triangle3D original = new Triangle3D(pointA, pointB, pointC);
      Triangle3D same = new Triangle3D(original);
      assertTrue(original.epsilonEquals(same, getEpsilon()));

      for (int i = 0; i < ITERATIONS; i++)
      {
         pointA = EuclidCoreRandomTools.nextPoint3D(random);
         pointB = EuclidCoreRandomTools.nextPoint3D(random);
         pointC = EuclidCoreRandomTools.nextPoint3D(random);

         original = new Triangle3D(pointA, pointB, pointC);

         Point3D pointAChanged = new Point3D(pointA);
         Point3D pointBChanged = new Point3D(pointB);
         Point3D pointCChanged = new Point3D(pointC);

         //Point A above epsilon
         pointAChanged.setElement(i % 3, pointAChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedA = new Triangle3D(pointAChanged, pointB, pointC);
         assertFalse(changedA.epsilonEquals(original, getEpsilon()));

         //Point A below epsilon
         pointAChanged.set(pointA);
         pointAChanged.setElement(i % 3, pointAChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedA = new Triangle3D(pointAChanged, pointB, pointC);
         assertTrue(changedA.epsilonEquals(original, getEpsilon()));

         //Point B above epsilon
         pointBChanged.setElement(i % 3, pointBChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedB = new Triangle3D(pointA, pointBChanged, pointC);
         assertFalse(changedB.epsilonEquals(original, getEpsilon()));

         //Point B below epsilon
         pointBChanged.set(pointB);
         pointBChanged.setElement(i % 3, pointBChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedB = new Triangle3D(pointA, pointBChanged, pointC);
         assertTrue(changedB.epsilonEquals(original, getEpsilon()));

         //Point C above epsilon
         pointCChanged.setElement(i % 3, pointCChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedC = new Triangle3D(pointA, pointB, pointCChanged);
         assertFalse(changedC.epsilonEquals(original, getEpsilon()));

         //Point C below epsilon
         pointCChanged.set(pointC);
         pointCChanged.setElement(i % 3, pointCChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedC = new Triangle3D(pointA, pointB, pointCChanged);
         assertTrue(changedC.epsilonEquals(original, getEpsilon()));
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(87452L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

         Triangle3D original = new Triangle3D(pointA, pointB, pointC);
         Triangle3D expected = new Triangle3D(original);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointC, pointB);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointA, pointC);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointC, pointA);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointB, pointA);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointA, pointB);
         assertTrue(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointA, pointA);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointB, pointB);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointC, pointC);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointA, pointB);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointA, pointC);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointB, pointA);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointB, pointC);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointC, pointA);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointC, pointB);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointB, pointB);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointA, pointC, pointC);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointA, pointA);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointB, pointC, pointC);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointA, pointA);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));

         expected = new Triangle3D(pointC, pointB, pointB);
         assertFalse(original.geometricallyEquals(expected, getEpsilon()));
      }

      //Test Epsilon is working correctly for geometricallyEquals method
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

         Triangle3D original = new Triangle3D(pointA, pointB, pointC);

         Point3D pointAChanged = new Point3D(pointA);
         Point3D pointBChanged = new Point3D(pointB);
         Point3D pointCChanged = new Point3D(pointC);

         //Point A above epsilon
         pointAChanged.setElement(i % 3, pointAChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedA = new Triangle3D(pointAChanged, pointB, pointC);
         assertFalse(changedA.geometricallyEquals(original, getEpsilon()));

         //Point A below epsilon
         pointAChanged.set(pointA);
         pointAChanged.setElement(i % 3, pointAChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedA = new Triangle3D(pointAChanged, pointB, pointC);
         assertTrue(changedA.geometricallyEquals(original, getEpsilon()));

         //Point B above epsilon
         pointBChanged.setElement(i % 3, pointBChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedB = new Triangle3D(pointA, pointBChanged, pointC);
         assertFalse(changedB.geometricallyEquals(original, getEpsilon()));

         //Point B below epsilon
         pointBChanged.set(pointB);
         pointBChanged.setElement(i % 3, pointBChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedB = new Triangle3D(pointA, pointBChanged, pointC);
         assertTrue(changedB.geometricallyEquals(original, getEpsilon()));

         //Point C above epsilon
         pointCChanged.setElement(i % 3, pointCChanged.getElement(i % 3) - 1.01 * getEpsilon());
         Triangle3D changedC = new Triangle3D(pointA, pointB, pointCChanged);
         assertFalse(changedC.geometricallyEquals(original, getEpsilon()));

         //Point C below epsilon
         pointCChanged.set(pointC);
         pointCChanged.setElement(i % 3, pointCChanged.getElement(i % 3) - 0.99 * getEpsilon());
         changedC = new Triangle3D(pointA, pointB, pointCChanged);
         assertTrue(changedC.geometricallyEquals(original, getEpsilon()));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(62691L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

         Point3D pointAChanged = new Point3D(pointA);
         Point3D pointBChanged = new Point3D(pointB);
         Point3D pointCChanged = new Point3D(pointC);

         //Verify hash of the points is equal to hash of the Triangle
         int hashOfPointAPointBPointC = EuclidHashCodeTools.toIntHashCode(pointA, pointB, pointC);
         Triangle3D original = new Triangle3D(pointA, pointB, pointC);

         int hashOfTriangle = original.hashCode();
         assertEquals(hashOfPointAPointBPointC, hashOfTriangle);

         //Change one of the points in PointA to verify the hash is no longer equal
         pointAChanged.setElement(i % 3, random.nextDouble());
         Triangle3D changed = new Triangle3D(pointAChanged, pointB, pointC);
         hashOfTriangle = changed.hashCode();
         assertFalse(hashOfPointAPointBPointC == hashOfTriangle);

         //Change one of the points in PointB to verify the hash is no longer equal
         pointBChanged.setElement(i % 3, random.nextDouble());
         changed = new Triangle3D(pointA, pointBChanged, pointC);
         hashOfTriangle = changed.hashCode();
         assertFalse(hashOfPointAPointBPointC == hashOfTriangle);

         //Change one of the points in PointC to verify the hash is no longer equal
         pointCChanged.setElement(i % 3, random.nextDouble());
         changed = new Triangle3D(pointA, pointB, pointCChanged);
         hashOfTriangle = changed.hashCode();
         assertFalse(hashOfPointAPointBPointC == hashOfTriangle);

      }
   }

   @Test
   public void testToString() throws Exception
   {
      Random random = new Random(62691L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointB = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointC = EuclidCoreRandomTools.nextPoint3D(random);

         Triangle3D triangle = new Triangle3D(pointA, pointB, pointC);

         assertEquals(triangle.toString(), EuclidGeometryIOTools.getTriangle3DString(triangle));
      }

   }
}
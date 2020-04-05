package us.ihmc.euclid.tuple2D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class Point2DTest extends Point2DBasicsTest<Point2D>
{
   @Test
   public void testPoint2D()
   {
      Random random = new Random(621541L);
      Point2D point = new Point2D();

      { // Test Point2D()
         assertTrue(0 == point.getX());
         assertTrue(0 == point.getY());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D(double x, double y)
         double newX = random.nextDouble();
         double newY = random.nextDouble();

         point = new Point2D(newX, newY);

         assertTrue(newX == point.getX());
         assertTrue(newY == point.getY());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D(double[] pointArray)
         double[] randomPoint2DArray = {random.nextDouble(), random.nextDouble()};
         Point2D pointArray = new Point2D(randomPoint2DArray);

         assertTrue(randomPoint2DArray[0] == pointArray.getX());
         assertTrue(randomPoint2DArray[1] == pointArray.getY());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D(Tuple2DReadOnly tuple)
         Point2D point2 = createRandomTuple(random);
         point = new Point2D(point2);

         assertTrue(point.getX() == point2.getX());
         assertTrue(point.getY() == point2.getY());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D(Tuple3DReadOnly tuple)
         Point3D point2 = EuclidCoreRandomTools.nextPoint3D(random);
         point = new Point2D(point2);

         assertTrue(point.getX() == point2.getX());
         assertTrue(point.getY() == point2.getY());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple1.setElement(i % 2, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Point2D pointA;
      Point2D pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         pointA = EuclidCoreRandomTools.nextPoint2D(random);
         pointB = EuclidCoreRandomTools.nextPoint2D(random);

         if (((Point2DReadOnly) pointA).geometricallyEquals(pointB, getEpsilon()))
         {
            assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));
         }
         else
         {
            assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
         }
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-14;
   }

   @Override
   public Point2D createEmptyTuple()
   {
      return new Point2D();
   }

   @Override
   public Point2D createTuple(double x, double y)
   {
      return new Point2D(x, y);
   }

   @Override
   public Point2D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint2D(random);
   }
}

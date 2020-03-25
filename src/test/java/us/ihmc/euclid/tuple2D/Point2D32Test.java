package us.ihmc.euclid.tuple2D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class Point2D32Test extends Point2DBasicsTest<Point2D32>
{
   @Test
   public void testPoint2D32()
   {
      Random random = new Random(621541L);
      Point2D32 point = new Point2D32();

      { // Test Point2D32()
         assertTrue(0f == point.getX32());
         assertTrue(0f == point.getY32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D32(float x, float y)
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         point = new Point2D32(newX, newY);

         assertTrue(newX == point.getX32());
         assertTrue(newY == point.getY32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D32(float[] pointArray)
         float[] randomPoint2D32Array = {random.nextFloat(), random.nextFloat()};
         float[] copyRandomPoint2D32Array = new float[2];
         copyRandomPoint2D32Array[0] = randomPoint2D32Array[0];
         copyRandomPoint2D32Array[1] = randomPoint2D32Array[1];

         Point2D32 pointArray = new Point2D32(randomPoint2D32Array);

         assertTrue(randomPoint2D32Array[0] == pointArray.getX32());
         assertTrue(randomPoint2D32Array[1] == pointArray.getY32());

         assertTrue(copyRandomPoint2D32Array[0] == randomPoint2D32Array[0]);
         assertTrue(copyRandomPoint2D32Array[1] == randomPoint2D32Array[1]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point2D32(TupleBasics tuple)
         Point2D32 point2 = EuclidCoreRandomTools.nextPoint2D32(random);

         point = new Point2D32(point2);

         assertTrue(point.getX32() == point2.getX32());
         assertTrue(point.getY32() == point2.getY32());
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Point2D32 tuple1 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setX(float x)
         float x = random.nextFloat();
         tuple1.setX(x);
         assertEquals(tuple1.getX32(), x, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setY(float y)
         float y = random.nextFloat();
         tuple1.setY(y);
         assertEquals(tuple1.getY32(), y, getEpsilon());
      }

   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D32 point = EuclidCoreRandomTools.nextPoint2D32(random);

      int newHashCode, previousHashCode;
      newHashCode = point.hashCode();
      assertEquals(newHashCode, point.hashCode());

      previousHashCode = point.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         point.setElement(i % 2, random.nextFloat());
         newHashCode = point.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Point2D32 pointA;
      Point2D32 pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         pointA = EuclidCoreRandomTools.nextPoint2D32(random);
         pointB = EuclidCoreRandomTools.nextPoint2D32(random);

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
      return 1.0e-6;
   }

   @Override
   public Point2D32 createEmptyTuple()
   {
      return new Point2D32();
   }

   @Override
   public Point2D32 createTuple(double x, double y)
   {
      return new Point2D32((float) x, (float) y);
   }

   @Override
   public Point2D32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint2D32(random);
   }
}

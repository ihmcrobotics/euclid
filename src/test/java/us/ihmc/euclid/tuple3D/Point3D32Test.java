package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Point3D32Test extends Point3DBasicsTest<Point3D32>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testPoint3D32()
   {
      Random random = new Random(621541L);

      { // Test Point32()
         Point3D32 point = new Point3D32();

         assertTrue(0 == point.getX());
         assertTrue(0 == point.getY());
         assertTrue(0 == point.getZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point32(float x, float y, float z)
         Point3D32 point;

         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         point = new Point3D32(newX, newY, newZ);

         assertTrue(newX == point.getX32());
         assertTrue(newY == point.getY32());
         assertTrue(newZ == point.getZ32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point32(float[] pointArray)
         float[] randomPoint32Array = {random.nextFloat(), random.nextFloat(), random.nextFloat()};

         Point3D32 point = new Point3D32(randomPoint32Array);

         assertTrue(randomPoint32Array[0] == point.getX32());
         assertTrue(randomPoint32Array[1] == point.getY32());
         assertTrue(randomPoint32Array[2] == point.getZ32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Point32(TupleBasics tuple)
         Point3D32 point;
         Point3D32 point2 = EuclidCoreRandomTools.nextPoint3D32(random);
         point = new Point3D32(point2);
         EuclidCoreTestTools.assertEquals(point, point2, EPS);
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Point3D32 tuple1 = createEmptyTuple();

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

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setZ(float z)
         float z = random.nextFloat();
         tuple1.setZ(z);
         assertEquals(tuple1.getZ32(), z, getEpsilon());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point3D32 tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple1.setElement(i % 3, random.nextFloat());
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

      Point3D32 pointA;
      Point3D32 pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         pointA = EuclidCoreRandomTools.nextPoint3D32(random);
         pointB = EuclidCoreRandomTools.nextPoint3D32(random);

         if (((Point3DReadOnly) pointA).geometricallyEquals(pointB, getEpsilon()))
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
   public Point3D32 createEmptyTuple()
   {
      return new Point3D32();
   }

   @Override
   public Point3D32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint3D32(random);
   }

   @Override
   public Point3D32 createTuple(double x, double y, double z)
   {
      return new Point3D32((float) x, (float) y, (float) z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}

package us.ihmc.euclid.tuple3D;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Point3DTest extends Point3DBasicsTest<Point3D>
{
   @Test
   public void testPoint3D()
   {
      Random random = new Random(621541L);
      Point3D point = new Point3D();

      { // Test Point()
         Assert.assertTrue(0 == point.getX());
         Assert.assertTrue(0 == point.getY());
         Assert.assertTrue(0 == point.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         point = new Point3D(newX, newY, newZ);

         Assert.assertTrue(newX == point.getX());
         Assert.assertTrue(newY == point.getY());
         Assert.assertTrue(newZ == point.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(double[] pointArray)
         double[] randomPointArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] copyRandomPointArray = new double[3];
         copyRandomPointArray[0] = randomPointArray[0];
         copyRandomPointArray[1] = randomPointArray[1];
         copyRandomPointArray[2] = randomPointArray[2];

         Point3D pointArray = new Point3D(randomPointArray);

         Assert.assertTrue(randomPointArray[0] == pointArray.getX());
         Assert.assertTrue(randomPointArray[1] == pointArray.getY());
         Assert.assertTrue(randomPointArray[2] == pointArray.getZ());

         Assert.assertTrue(copyRandomPointArray[0] == randomPointArray[0]);
         Assert.assertTrue(copyRandomPointArray[1] == randomPointArray[1]);
         Assert.assertTrue(copyRandomPointArray[2] == randomPointArray[2]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point(TupleBasics tuple)
         Point3D point2 = new Point3D();
         point2.setX(random.nextDouble());
         point2.setY(random.nextDouble());
         point2.setZ(random.nextDouble());

         point = new Point3D(point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
         Assert.assertTrue(point.getZ() == point2.getZ());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point3D tuple1 = createEmptyTuple();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.setElement(i % 3, random.nextDouble());
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

      Point3D pointA;
      Point3D pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         pointA = EuclidCoreRandomTools.nextPoint3D(random);
         pointB = EuclidCoreRandomTools.nextPoint3D(random);

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
   public Point3D createEmptyTuple()
   {
      return new Point3D();
   }

   @Override
   public Point3D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint3D(random);
   }

   @Override
   public Point3D createTuple(double x, double y, double z)
   {
      return new Point3D(x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}

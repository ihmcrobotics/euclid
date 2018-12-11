package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Vector3DTest extends Vector3DBasicsTest<Vector3D>
{
   @Test
   public void testVector()
   {
      Random random = new Random(621541L);
      Vector3D vector = new Vector3D();

      { // Test Vector()
         assertTrue(0 == vector.getX());
         assertTrue(0 == vector.getY());
         assertTrue(0 == vector.getZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         vector = new Vector3D(newX, newY, newZ);

         assertTrue(newX == vector.getX());
         assertTrue(newY == vector.getY());
         assertTrue(newZ == vector.getZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector(double[] vectorArray)
         double[] randomVectorArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector3D vectorArray = new Vector3D(randomVectorArray);

         assertTrue(randomVectorArray[0] == vectorArray.getX());
         assertTrue(randomVectorArray[1] == vectorArray.getY());
         assertTrue(randomVectorArray[2] == vectorArray.getZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector(TupleBasics tuple)
         Vector3D vector2 = new Vector3D();
         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         vector = new Vector3D(vector2);

         assertTrue(vector.getX() == vector2.getX());
         assertTrue(vector.getY() == vector2.getY());
         assertTrue(vector.getZ() == vector2.getZ());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector3D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
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

      Vector3D vectorA;
      Vector3D vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector3D(random);
         vectorB = EuclidCoreRandomTools.nextVector3D(random);

         if (((Vector3DReadOnly) vectorA).geometricallyEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
         }
         else
         {
            assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
         }
      }
   }

   @Override
   public Vector3D createEmptyTuple()
   {
      return new Vector3D();
   }

   @Override
   public Vector3D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextRotationVector(random);
   }

   @Override
   public Vector3D createTuple(double x, double y, double z)
   {
      return new Vector3D(x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}

package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Vector3D32Test extends Vector3DBasicsTest<Vector3D32>
{
   @Test
   public void testVector32()
   {
      Random random = new Random(621541L);
      Vector3D32 vector = new Vector3D32();

      { // Test Vector32()
         assertTrue(0 == vector.getX32());
         assertTrue(0 == vector.getY32());
         assertTrue(0 == vector.getZ32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector32(float x, float y, float z)
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         vector = new Vector3D32(newX, newY, newZ);

         assertTrue(newX == vector.getX32());
         assertTrue(newY == vector.getY32());
         assertTrue(newZ == vector.getZ32());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector32(float[] vectorArray)
         float[] randomVector32Array = {random.nextFloat(), random.nextFloat(), random.nextFloat()};
         float[] copyRandomVector32Array = new float[3];
         copyRandomVector32Array[0] = randomVector32Array[0];
         copyRandomVector32Array[1] = randomVector32Array[1];
         copyRandomVector32Array[2] = randomVector32Array[2];

         Vector3D32 vectorArray = new Vector3D32(randomVector32Array);

         assertTrue(randomVector32Array[0] == vectorArray.getX32());
         assertTrue(randomVector32Array[1] == vectorArray.getY32());
         assertTrue(randomVector32Array[2] == vectorArray.getZ32());

         assertTrue(copyRandomVector32Array[0] == randomVector32Array[0]);
         assertTrue(copyRandomVector32Array[1] == randomVector32Array[1]);
         assertTrue(copyRandomVector32Array[2] == randomVector32Array[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Vector32(TupleBasics tuple)
         Vector3D32 vector2 = EuclidCoreRandomTools.nextVector3D32(random);

         vector = new Vector3D32(vector2);

         assertTrue(vector.getX32() == vector2.getX32());
         assertTrue(vector.getY32() == vector2.getY32());
         assertTrue(vector.getZ32() == vector2.getZ32());
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Vector3D32 tuple1 = createEmptyTuple();

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
      Vector3D32 tuple1 = createRandomTuple(random);

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

      Vector3D32 vectorA;
      Vector3D32 vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector3D32(random);
         vectorB = EuclidCoreRandomTools.nextVector3D32(random);

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
   public Vector3D32 createEmptyTuple()
   {
      return new Vector3D32();
   }

   @Override
   public Vector3D32 createTuple(double x, double y, double z)
   {
      return new Vector3D32((float) x, (float) y, (float) z);
   }

   @Override
   public Vector3D32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector3D32(random);
   }

   @Override
   public double getEpsilon()
   {
      return 2.0e-7;
   }
}

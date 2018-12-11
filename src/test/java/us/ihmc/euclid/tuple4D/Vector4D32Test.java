package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public class Vector4D32Test extends Vector4DBasicsTest<Vector4D32>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3453L);

      { // Test empty constructor
         Vector4D32 vector = new Vector4D32();
         assertTrue(vector.getX32() == 0.0f);
         assertTrue(vector.getY32() == 0.0f);
         assertTrue(vector.getZ32() == 0.0f);
         assertTrue(vector.getS32() == 0.0f);
      }

      { // Test Vector4D32(float x, float y, float z, float s)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         float s = random.nextFloat();
         Vector4D32 vector = new Vector4D32(x, y, z, s);
         assertTrue(vector.getX32() == x);
         assertTrue(vector.getY32() == y);
         assertTrue(vector.getZ32() == z);
         assertTrue(vector.getS32() == s);
      }

      { // Test Vector4D32(float[] vectorArray)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         float s = random.nextFloat();
         float[] vectorArray = {x, y, z, s};
         Vector4D32 vector = new Vector4D32(vectorArray);
         assertTrue(vector.getX32() == x);
         assertTrue(vector.getY32() == y);
         assertTrue(vector.getZ32() == z);
         assertTrue(vector.getS32() == s);
      }

      { // Test Vector4D32(QuaternionReadOnly quaternion)
         Quaternion32 quaternion = EuclidCoreRandomTools.nextQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D32(Tuple4DReadOnly other)
         Tuple4DReadOnly quaternion = EuclidCoreRandomTools.nextQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Vector3DReadOnly vector3D)
         Vector3DReadOnly vector3D = EuclidCoreRandomTools.nextVector3D(random);
         Vector4D32 vector = new Vector4D32(vector3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.getElement32(i) == vector3D.getElement32(i));
         assertTrue(vector.getS32() == 0.0f);
      }

      { // Test Vector4D(Point3DReadOnly vector3D)
         Point3DReadOnly point3D = EuclidCoreRandomTools.nextPoint3D(random);
         Vector4D32 vector = new Vector4D32(point3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.getElement32(i) == point3D.getElement32(i));
         assertTrue(vector.getS32() == 1.0f);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector4D32 vector = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = vector.hashCode();
      assertEquals(newHashCode, vector.hashCode());

      previousHashCode = vector.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector.setElement(i % 4, random.nextDouble());
         newHashCode = vector.hashCode();
         assertNotEquals((long) newHashCode, (long) previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Vector4D32 vectorA;
      Vector4D32 vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector4D32(random);
         vectorB = EuclidCoreRandomTools.nextVector4D32(random);

         if (((Vector4DReadOnly) vectorA).geometricallyEquals(vectorB, getEpsilon()))
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
   public Vector4D32 createEmptyTuple()
   {
      return new Vector4D32();
   }

   @Override
   public Vector4D32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector4D32(random);
   }

   @Override
   public Vector4D32 createTuple(double x, double y, double z, double s)
   {
      return new Vector4D32((float) x, (float) y, (float) z, (float) s);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}

package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Axis3DTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testOrdinals()
   {
      assertEquals(Axis3D.X.ordinal(), 0);
      assertEquals(Axis3D.Y.ordinal(), 1);
      assertEquals(Axis3D.Z.ordinal(), 2);
   }

   @Test
   public void testGetElement()
   {
      Random random = new Random(436566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D point3D = EuclidCoreRandomTools.nextPoint3D(random);
         assertEquals(point3D.getX(), Axis3D.X.getElement(point3D));
         assertEquals(point3D.getY(), Axis3D.Y.getElement(point3D));
         assertEquals(point3D.getZ(), Axis3D.Z.getElement(point3D));

         for (int j = 0; j < 3; j++)
         {
            assertEquals(point3D.getElement(j), Axis3D.values[j].getElement(point3D));
         }
      }
   }

   @Test
   public void testSetElement()
   {
      Random random = new Random(436566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D actual = new Point3D(original);
         Point3D expected = new Point3D(original);
         double newValue = random.nextDouble();

         for (int j = 0; j < 3; j++)
         {
            Axis3D.values[j].setElement(actual, newValue);
            expected.setElement(j, newValue);
            assertEquals(expected, actual);
         }
      }
   }

   @Test
   public void testVectorValues()
   {
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 0, 0), Axis3D.X, 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0, 1, 0), Axis3D.Y, 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0, 0, 1), Axis3D.Z, 0.0);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(-1, 0, 0), Axis3D.X.negated(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0, -1, 0), Axis3D.Y.negated(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0, 0, -1), Axis3D.Z.negated(), 0.0);
   }

   @Test
   public void testNextAndPrevious()
   {
      assertEquals(Axis3D.X.next(), Axis3D.Y);
      assertEquals(Axis3D.Y.next(), Axis3D.Z);
      assertEquals(Axis3D.Z.next(), Axis3D.X);

      assertEquals(Axis3D.X.previous(), Axis3D.Z);
      assertEquals(Axis3D.Y.previous(), Axis3D.X);
      assertEquals(Axis3D.Z.previous(), Axis3D.Y);
   }
}

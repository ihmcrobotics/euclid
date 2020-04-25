package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class Axis2DTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testOrdinals()
   {
      assertEquals(Axis2D.X.ordinal(), 0);
      assertEquals(Axis2D.Y.ordinal(), 1);
   }

   @Test
   public void testGetElement()
   {
      Random random = new Random(436566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D point2D = EuclidCoreRandomTools.nextPoint2D(random);
         assertEquals(point2D.getX(), Axis2D.X.getElement(point2D));
         assertEquals(point2D.getY(), Axis2D.Y.getElement(point2D));

         for (int j = 0; j < 2; j++)
         {
            assertEquals(point2D.getElement(j), Axis2D.values[j].getElement(point2D));
         }
      }
   }

   @Test
   public void testSetElement()
   {
      Random random = new Random(436566);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D original = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D actual = new Point2D(original);
         Point2D expected = new Point2D(original);
         double newValue = random.nextDouble();

         for (int j = 0; j < 2; j++)
         {
            Axis2D.values[j].setElement(actual, newValue);
            expected.setElement(j, newValue);
            assertEquals(expected, actual);
         }
      }
   }

   @Test
   public void testVectorValues()
   {
      EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(1, 0), Axis2D.X, 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(0, 1), Axis2D.Y, 0.0);

      EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(-1, 0), Axis2D.X.negated(), 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(0, -1), Axis2D.Y.negated(), 0.0);
   }

   @Test
   public void testOther()
   {
      assertEquals(Axis2D.X.other(), Axis2D.Y);
      assertEquals(Axis2D.Y.other(), Axis2D.X);
   }
}

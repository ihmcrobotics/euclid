package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class Axis2DTest
{
   private double allowedDelta = 1e-7;
   private Axis2D xAxis = Axis2D.X, yAxis = Axis2D.Y;
   private boolean touchedX, touchedY;
   private Random random = new Random(12345);
   private Vector2D randomVector = new Vector2D(random.nextDouble(), random.nextDouble());

   @Test
   public void testOrdinals()
   {
      assertEquals(xAxis.ordinal(), 0, allowedDelta);
      assertEquals(yAxis.ordinal(), 1, allowedDelta);
   }

   @Test
   public void testAxisValuesGetter()
   {
      assertEquals(2, Axis2D.values().length);

      touchedX = false;
      touchedY = false;

      for (Axis2D axis : Axis2D.values())
      {
         switch (axis)
         {
            case X:
               touchedX = true;
               break;
            case Y:
               touchedY = true;
               break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
   }

   @Test
   public void testAxisValuesField()
   {
      assertEquals(2, Axis2D.values.length);

      touchedX = false;
      touchedY = false;

      for (Axis2D axis : Axis2D.values)
      {
         switch (axis)
         {
            case X:
               touchedX = true;
               break;
            case Y:
               touchedY = true;
               break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
   }

   @Test
   public void testAxisVectorsForCorrectValues()
   {
      Vector2DReadOnly xAxisVector = xAxis;
      Vector2DReadOnly yAxisVector = yAxis;

      assertTrue(xAxisVector != null);
      assertTrue(yAxisVector != null);

      assertEquals(1.0, xAxisVector.getX(), allowedDelta);
      assertEquals(0.0, xAxisVector.getY(), allowedDelta);
      assertEquals(0.0, yAxisVector.getX(), allowedDelta);
      assertEquals(1.0, yAxisVector.getY(), allowedDelta);
   }

   @Test
   public void testAxisSetterWorks()
   {
      double newXValue = random.nextDouble(), newYValue = random.nextDouble();

      Axis2D.set(randomVector, Axis2D.X, newXValue);
      Axis2D.set(randomVector, Axis2D.Y, newYValue);

      assertEquals(newXValue, randomVector.getX(), allowedDelta);
      assertEquals(newYValue, randomVector.getY(), allowedDelta);
   }

   @Test
   public void testAxisGetterWorks()
   {
      assertEquals(randomVector.getX(), Axis2D.get(randomVector, Axis2D.X), allowedDelta);
      assertEquals(randomVector.getY(), Axis2D.get(randomVector, Axis2D.Y), allowedDelta);
   }

   @Test
   public void testClockwiseAxisGetters()
   {
      assertEquals(Axis2D.Y, Axis2D.X.getNextClockwiseAxis());
      assertEquals(Axis2D.X, Axis2D.Y.getNextClockwiseAxis());
      assertEquals(Axis2D.Y, Axis2D.X.getNextCounterClockwiseAxis());
      assertEquals(Axis2D.X, Axis2D.Y.getNextCounterClockwiseAxis());
   }
}

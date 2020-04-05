package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Axis3DTest
{
   private double allowedDelta = 1e-7;
   private Axis3D xAxis = Axis3D.X, yAxis = Axis3D.Y, zAxis = Axis3D.Z;
   private boolean touchedX, touchedY, touchedZ;
   private Random random = new Random(12345);
   private Vector3D randomVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

   @Test
   public void testOrdinals()
   {
      assertEquals(xAxis.ordinal(), 0, allowedDelta);
      assertEquals(yAxis.ordinal(), 1, allowedDelta);
      assertEquals(zAxis.ordinal(), 2, allowedDelta);
   }

   @Test
   public void testAxisValuesGetter()
   {
      assertEquals(3, Axis3D.values().length);

      touchedX = false;
      touchedY = false;
      touchedZ = false;

      for (Axis3D axis : Axis3D.values())
      {
         switch (axis)
         {
            case X:
               touchedX = true;

               break;

            case Y:
               touchedY = true;

               break;

            case Z:
               touchedZ = true;

               break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
      assertTrue(touchedZ);
   }

   @Test
   public void testAxisValuesField()
   {
      assertEquals(3, Axis3D.values.length);

      touchedX = false;
      touchedY = false;
      touchedZ = false;

      for (Axis3D axis : Axis3D.values)
      {
         switch (axis)
         {
            case X:
               touchedX = true;

               break;

            case Y:
               touchedY = true;

               break;

            case Z:
               touchedZ = true;

               break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
      assertTrue(touchedZ);
   }

   @Test
   public void testAxisVectorsForCorrectValues()
   {
      Vector3DReadOnly xAxisVector = xAxis;
      Vector3DReadOnly yAxisVector = yAxis;
      Vector3DReadOnly zAxisVector = zAxis;

      assertTrue(xAxisVector != null);
      assertTrue(yAxisVector != null);
      assertTrue(zAxisVector != null);

      assertEquals(1.0, xAxisVector.getX(), allowedDelta);
      assertEquals(0.0, xAxisVector.getY(), allowedDelta);
      assertEquals(0.0, xAxisVector.getZ(), allowedDelta);
      assertEquals(0.0, yAxisVector.getX(), allowedDelta);
      assertEquals(1.0, yAxisVector.getY(), allowedDelta);
      assertEquals(0.0, yAxisVector.getZ(), allowedDelta);
      assertEquals(0.0, zAxisVector.getX(), allowedDelta);
      assertEquals(0.0, zAxisVector.getY(), allowedDelta);
      assertEquals(1.0, zAxisVector.getZ(), allowedDelta);
   }

   @Test
   public void testAxisSetterWorks()
   {
      double newXValue = random.nextDouble(), newYValue = random.nextDouble(), newZValue = random.nextDouble();

      Axis3D.set(randomVector, Axis3D.X, newXValue);
      Axis3D.set(randomVector, Axis3D.Y, newYValue);
      Axis3D.set(randomVector, Axis3D.Z, newZValue);

      assertEquals(newXValue, randomVector.getX(), allowedDelta);
      assertEquals(newYValue, randomVector.getY(), allowedDelta);
      assertEquals(newZValue, randomVector.getZ(), allowedDelta);
   }

   @Test
   public void testAxisGetterWorks()
   {
      assertEquals(randomVector.getX(), Axis3D.get(randomVector, Axis3D.X), allowedDelta);
      assertEquals(randomVector.getY(), Axis3D.get(randomVector, Axis3D.Y), allowedDelta);
      assertEquals(randomVector.getZ(), Axis3D.get(randomVector, Axis3D.Z), allowedDelta);
   }

   @Test
   public void testClockwiseAxisGetters()
   {
      assertEquals(Axis3D.Z, Axis3D.X.getNextClockwiseAxis());
      assertEquals(Axis3D.X, Axis3D.Y.getNextClockwiseAxis());
      assertEquals(Axis3D.Y, Axis3D.Z.getNextClockwiseAxis());
      assertEquals(Axis3D.Y, Axis3D.X.getNextCounterClockwiseAxis());
      assertEquals(Axis3D.Z, Axis3D.Y.getNextCounterClockwiseAxis());
      assertEquals(Axis3D.X, Axis3D.Z.getNextCounterClockwiseAxis());
   }
}

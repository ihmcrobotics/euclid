package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class AxisTest
{
   private double allowedDelta = 1e-7;
   private Axis xAxis = Axis.X, yAxis = Axis.Y, zAxis = Axis.Z;
   private boolean touchedX, touchedY, touchedZ;
   private Random random = new Random(12345);
   private Vector3D randomVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

   @Test
   public void testOrdinals()
   {
      assertEqualsDelta(xAxis.ordinal(), 0, allowedDelta);
      assertEqualsDelta(yAxis.ordinal(), 1, allowedDelta);
      assertEqualsDelta(zAxis.ordinal(), 2, allowedDelta);
   }

   @Test
   public void testAxisValuesGetter()
   {
      assertEquals(3, Axis.values().length);

      touchedX = false;
      touchedY = false;
      touchedZ = false;

      for (Axis axis : Axis.values())
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
      assertEquals(3, Axis.values.length);

      touchedX = false;
      touchedY = false;
      touchedZ = false;

      for (Axis axis : Axis.values)
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

      assertEqualsDelta(1.0, xAxisVector.getX(), allowedDelta);
      assertEqualsDelta(0.0, xAxisVector.getY(), allowedDelta);
      assertEqualsDelta(0.0, xAxisVector.getZ(), allowedDelta);
      assertEqualsDelta(0.0, yAxisVector.getX(), allowedDelta);
      assertEqualsDelta(1.0, yAxisVector.getY(), allowedDelta);
      assertEqualsDelta(0.0, yAxisVector.getZ(), allowedDelta);
      assertEqualsDelta(0.0, zAxisVector.getX(), allowedDelta);
      assertEqualsDelta(0.0, zAxisVector.getY(), allowedDelta);
      assertEqualsDelta(1.0, zAxisVector.getZ(), allowedDelta);
   }

   @Test
   public void testAxisSetterWorks()
   {
      double newXValue = random.nextDouble(), newYValue = random.nextDouble(), newZValue = random.nextDouble();

      Axis.set(randomVector, Axis.X, newXValue);
      Axis.set(randomVector, Axis.Y, newYValue);
      Axis.set(randomVector, Axis.Z, newZValue);

      assertEqualsDelta(newXValue, randomVector.getX(), allowedDelta);
      assertEqualsDelta(newYValue, randomVector.getY(), allowedDelta);
      assertEqualsDelta(newZValue, randomVector.getZ(), allowedDelta);
   }

   @Test
   public void testAxisGetterWorks()
   {
      assertEqualsDelta(randomVector.getX(), Axis.get(randomVector, Axis.X), allowedDelta);
      assertEqualsDelta(randomVector.getY(), Axis.get(randomVector, Axis.Y), allowedDelta);
      assertEqualsDelta(randomVector.getZ(), Axis.get(randomVector, Axis.Z), allowedDelta);
   }

   @Test
   public void testClockwiseAxisGetters()
   {
      assertEquals(Axis.Z, Axis.X.getNextClockwiseAxis());
      assertEquals(Axis.X, Axis.Y.getNextClockwiseAxis());
      assertEquals(Axis.Y, Axis.Z.getNextClockwiseAxis());
      assertEquals(Axis.Y, Axis.X.getNextCounterClockwiseAxis());
      assertEquals(Axis.Z, Axis.Y.getNextCounterClockwiseAxis());
      assertEquals(Axis.X, Axis.Z.getNextCounterClockwiseAxis());
   }
}

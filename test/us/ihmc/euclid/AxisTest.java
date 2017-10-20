package us.ihmc.euclid;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class AxisTest
{
   private double allowedDelta = 1e-7;
   private Axis xAxis = Axis.X, yAxis = Axis.Y, zAxis = Axis.Z;
   private boolean touchedX, touchedY, touchedZ;
   private Random random = new Random(12345);
   private Vector3D randomVector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrdinals()
   {
      assertEquals(xAxis.ordinal(), 0, allowedDelta);
      assertEquals(yAxis.ordinal(), 1, allowedDelta);
      assertEquals(zAxis.ordinal(), 2, allowedDelta);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
         case X :
            touchedX = true;

            break;

         case Y :
            touchedY = true;

            break;

         case Z :
            touchedZ = true;

            break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
      assertTrue(touchedZ);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
         case X :
            touchedX = true;

            break;

         case Y :
            touchedY = true;

            break;

         case Z :
            touchedZ = true;

            break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
      assertTrue(touchedZ);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisVectorsForCorrectValues()
   {
      Vector3DReadOnly xAxisVector = xAxis.getAxisVector();
      Vector3DReadOnly yAxisVector = yAxis.getAxisVector();
      Vector3DReadOnly zAxisVector = zAxis.getAxisVector();

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisSetterWorks()
   {
      double newXValue = random.nextDouble(), newYValue = random.nextDouble(), newZValue = random.nextDouble();

      Axis.set(randomVector, Axis.X, newXValue);
      Axis.set(randomVector, Axis.Y, newYValue);
      Axis.set(randomVector, Axis.Z, newZValue);

      assertEquals(newXValue, randomVector.getX(), allowedDelta);
      assertEquals(newYValue, randomVector.getY(), allowedDelta);
      assertEquals(newZValue, randomVector.getZ(), allowedDelta);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisGetterWorks()
   {
      assertEquals(randomVector.getX(), Axis.get(randomVector, Axis.X), allowedDelta);
      assertEquals(randomVector.getY(), Axis.get(randomVector, Axis.Y), allowedDelta);
      assertEquals(randomVector.getZ(), Axis.get(randomVector, Axis.Z), allowedDelta);
   }
}

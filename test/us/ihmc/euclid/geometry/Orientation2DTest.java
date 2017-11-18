package us.ihmc.euclid.geometry;

import org.junit.Test;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;

import java.util.Random;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class Orientation2DTest
{
   private int ITERATIONS = 1000;
   
   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(19732L);
      Orientation2D firstOrientation, secondOrientation;
      double epsilon = 1e-7;
      
      firstOrientation = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
      secondOrientation = new Orientation2D(firstOrientation);

      assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
      assertTrue(secondOrientation.geometricallyEquals(firstOrientation, epsilon));
      assertTrue(firstOrientation.geometricallyEquals(firstOrientation, epsilon));
      assertTrue(secondOrientation.geometricallyEquals(secondOrientation, epsilon));
      
      for (int i = 0; i < ITERATIONS; ++i)
      { // Orientations are geometrically equal if yaw is similar within +- epsilon
         firstOrientation = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         secondOrientation = new Orientation2D(firstOrientation);
         
         secondOrientation.setYaw(firstOrientation.getYaw() + 0.99 * epsilon);
         
         assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
         
         secondOrientation.setYaw(firstOrientation.getYaw() - 0.99 * epsilon);
         
         assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
         
         secondOrientation.setYaw(firstOrientation.getYaw() + 1.01 * epsilon);
         
         assertFalse(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
         
         secondOrientation.setYaw(firstOrientation.getYaw() - 1.01 * epsilon);

         assertFalse(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
      }
      
      for (int i = 0; i < ITERATIONS; ++i)
      { // If epsilon > 2pi, orientations are automatically considered geometrically equal
         firstOrientation = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         secondOrientation = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         
         epsilon = 4.0 * Math.PI * random.nextDouble();
         
         if (epsilon > 2.0 * Math.PI)
            assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
      }
   }
}

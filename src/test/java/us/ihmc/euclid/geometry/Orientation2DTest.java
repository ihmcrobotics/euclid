package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;

public class Orientation2DTest
{
   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(19732L);
      Orientation2D firstOrientation, secondOrientation;
      double epsilon = 1e-7;

      firstOrientation = EuclidGeometryRandomTools.nextOrientation2D(random);
      secondOrientation = new Orientation2D(firstOrientation);

      assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
      assertTrue(secondOrientation.geometricallyEquals(firstOrientation, epsilon));
      assertTrue(firstOrientation.geometricallyEquals(firstOrientation, epsilon));
      assertTrue(secondOrientation.geometricallyEquals(secondOrientation, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Orientations are geometrically equal if yaw is similar within +- epsilon
         firstOrientation = EuclidGeometryRandomTools.nextOrientation2D(random);
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
      { // If epsilon > pi, orientations are automatically considered geometrically equal
         firstOrientation = EuclidGeometryRandomTools.nextOrientation2D(random);
         secondOrientation = EuclidGeometryRandomTools.nextOrientation2D(random);

         epsilon = Math.PI;
         assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));

         epsilon = Math.PI + Math.PI * random.nextDouble();
         assertTrue(firstOrientation.geometricallyEquals(secondOrientation, epsilon));
      }
   }
}

package us.ihmc.euclid.shape.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

class EuclidShapeRandomToolsTest
{

   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   void testNextPoint2DInTriangle()
   {
      Random random = new Random(43);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 5.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 5.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 5.0);

         Point2D nextPoint = EuclidGeometryRandomTools.nextPoint2DInTriangle(random, a, b, c);
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(nextPoint, a, b, c));
      }

      {// Building equilateral triangle to verify identities from: http://mathworld.wolfram.com/TrianglePointPicking.html
         int numberOfPoints = (int) 1e6;
         double averageDistanceToCentroid = 0.0;
         double averageDistanceToA = 0.0;
         double averageDistanceToB = 0.0;
         double averageDistanceToC = 0.0;

         for (int i = 0; i < numberOfPoints; i++)
         {
            double l = random.nextDouble();
            Vector2D ab = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, l);
            Vector2D ac = new Vector2D();
            RotationMatrixTools.applyYawRotation(Math.PI / 3.0, ab, ac);
            Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 5.0);
            Point2D b = new Point2D(a);
            Point2D c = new Point2D(a);
            b.add(ab);
            c.add(ac);

            assertEquals(l, b.distance(c), EPSILON);

            Point2D nextPoint = EuclidGeometryRandomTools.nextPoint2DInTriangle(random, a, b, c);
            assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(nextPoint, a, b, c));

            Point2D centroid = EuclidGeometryTools.averagePoint2Ds(Arrays.asList(a, b, c));
            averageDistanceToCentroid += centroid.distance(nextPoint) / numberOfPoints;
            averageDistanceToA += a.distance(nextPoint) / numberOfPoints;
            averageDistanceToB += a.distance(nextPoint) / numberOfPoints;
            averageDistanceToC += a.distance(nextPoint) / numberOfPoints;
         }

         // Somehow the expected distance are exactly 2 times bigger... 
         double sqrt3 = EuclidCoreTools.squareRoot(3.0);
         double expectedDistanceToCentroid = (8.0 * sqrt3 + 3.0 * asinh(sqrt3) + Math.log(2.0 + sqrt3)) / 72.0;
         assertEquals(0.5 * expectedDistanceToCentroid, averageDistanceToCentroid, 1.0e-4);

         double expectedDistanceToAnyVertex = (4.0 + 3.0 * Math.log(3.0)) / 12.0;
         assertEquals(0.5 * expectedDistanceToAnyVertex, averageDistanceToA, 1.0e-3);
         assertEquals(0.5 * expectedDistanceToAnyVertex, averageDistanceToB, 1.0e-3);
         assertEquals(0.5 * expectedDistanceToAnyVertex, averageDistanceToC, 1.0e-3);
      }
   }

   private static double asinh(double x)
   { // From: http://wwwf.imperial.ac.uk/metric/metric_public/functions_and_graphs/hyperbolic_functions/inverses.html
      return Math.log(x + EuclidCoreTools.squareRoot(x * x + 1.0));
   }
}

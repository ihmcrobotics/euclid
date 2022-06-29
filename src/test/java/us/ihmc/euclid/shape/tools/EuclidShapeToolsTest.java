package us.ihmc.euclid.shape.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidShapeToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSupportingVertexCapsule3D()
   {
      Random random = new Random(89737893);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double capsule3DLength = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double capsule3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         Point3D capsule3DPosition = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D capsule3DAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D expected = new Point3D();
         EuclidShapeTools.supportingVertexCapsule3D(supportDirection, capsule3DPosition, capsule3DAxis, capsule3DLength, capsule3DRadius, expected);

         capsule3DAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));
         supportDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));
         Point3D actual = new Point3D();
         EuclidShapeTools.supportingVertexCapsule3D(supportDirection, capsule3DPosition, capsule3DAxis, capsule3DLength, capsule3DRadius, actual);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSupportingVertexCylinder3D()
   {
      Random random = new Random(89737893);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double cylinder3DLength = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double cylinder3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         Point3D cylinder3DPosition = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D cylinder3DAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D expected = new Point3D();
         EuclidShapeTools.supportingVertexCylinder3D(supportDirection, cylinder3DPosition, cylinder3DAxis, cylinder3DLength, cylinder3DRadius, expected);

         cylinder3DAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));
         supportDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));
         Point3D actual = new Point3D();
         EuclidShapeTools.supportingVertexCylinder3D(supportDirection, cylinder3DPosition, cylinder3DAxis, cylinder3DLength, cylinder3DRadius, actual);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSupportingVertexCircle3D()
   {
      Random random = new Random(89737893);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double circle3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         Point3D circle3DPosition = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D circle3DAxis = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D axisOrthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, circle3DAxis, true);

         Point3D expected = new Point3D();
         expected.scaleAdd(circle3DRadius, axisOrthogonal, circle3DPosition);

         Vector3D supportDirection = new Vector3D();
         supportDirection.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), axisOrthogonal);
         supportDirection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), circle3DAxis, supportDirection);

         Point3D actual = new Point3D();
         EuclidShapeTools.supportingVertexCircle3D(supportDirection, circle3DPosition, circle3DAxis, circle3DRadius, actual);

         assertEquals(circle3DRadius, actual.distance(circle3DPosition), EPSILON);
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testInnerSupportingVertexTorus3D()
   {
      Random random = new Random(78934);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double torus3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double torus3DTubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, torus3DRadius);
         Point3D torus3DPosition = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D torus3DAxis = EuclidCoreRandomTools.nextVector3D(random);
         Point3D supportingVertex = new Point3D();
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0); //EuclidCoreRandomTools.nextVector3D(random);

         EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, torus3DPosition, torus3DAxis, torus3DRadius, torus3DTubeRadius, supportingVertex);

         double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(supportingVertex,
                                                                                         torus3DPosition,
                                                                                         torus3DAxis,
                                                                                         torus3DRadius,
                                                                                         torus3DTubeRadius);
         assertEquals(0.0, signedDistance, EPSILON, "Iteration " + i);

         Vector3D normal = new Vector3D();
         Point3D closestPoint = new Point3D();
         EuclidShapeTools.evaluatePoint3DTorus3DCollision(supportingVertex,
                                                          torus3DPosition,
                                                          torus3DAxis,
                                                          torus3DRadius,
                                                          torus3DTubeRadius,
                                                          closestPoint,
                                                          normal);

         EuclidCoreTestTools.assertEquals(closestPoint, supportingVertex, EPSILON);
         // Assert that the normal is pointing towards the axis of the torus, indication that this is the inner part of the torus.
         Vector3D towardsAxis = new Vector3D();
         towardsAxis.sub(torus3DPosition, closestPoint);
         torus3DAxis.normalize();
         double dot = towardsAxis.dot(torus3DAxis);
         towardsAxis.scaleAdd(-dot, torus3DAxis, towardsAxis);
         assertTrue(normal.dot(towardsAxis) > 0.0);
      }
   }

   @Test
   public void testSignedDistanceBetweenPoint3DAndTorus3D()
   {
      Random random = new Random(9638966);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double torus3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double torus3DTubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, torus3DRadius);
         Point3D torus3DPosition = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D torus3DAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         double expected = EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(query, torus3DPosition, torus3DAxis, torus3DRadius, torus3DTubeRadius);

         torus3DAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));
         double actual = EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(query, torus3DPosition, torus3DAxis, torus3DRadius, torus3DTubeRadius);
         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testEvaluatePoint3DTorus3DCollision()
   {
      Random random = new Random(9638966);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double torus3DRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double torus3DTubeRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, torus3DRadius);
         Point3D torus3DPosition = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D torus3DAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         Point3D expectedClosestPointOnSurface = new Point3D();
         Vector3D expectedNormal = new Vector3D();
         double expectedSignedDistance = EuclidShapeTools.evaluatePoint3DTorus3DCollision(query,
                                                                                          torus3DPosition,
                                                                                          torus3DAxis,
                                                                                          torus3DRadius,
                                                                                          torus3DTubeRadius,
                                                                                          expectedClosestPointOnSurface,
                                                                                          expectedNormal);

         torus3DAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         Point3D actualClosestPointOnSurface = new Point3D();
         Vector3D actualNormal = new Vector3D();
         double actualSignedDistance = EuclidShapeTools.evaluatePoint3DTorus3DCollision(query,
                                                                                        torus3DPosition,
                                                                                        torus3DAxis,
                                                                                        torus3DRadius,
                                                                                        torus3DTubeRadius,
                                                                                        actualClosestPointOnSurface,
                                                                                        actualNormal);

         assertEquals(expectedSignedDistance, actualSignedDistance, EPSILON, "Iteration " + i);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expectedClosestPointOnSurface, actualClosestPointOnSurface, EPSILON);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expectedNormal, actualNormal, EPSILON);
      }
   }

   @Test
   public void testComputeRamp3DCentroid()
   {
      Random random = new Random(365435);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D actualCentroid = new Point3D();
         EuclidShapeTools.computeRamp3DCentroid(ramp3D.getPose(), ramp3D.getSize(), actualCentroid);

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(ramp3D.getVertices()));
         Point3D expectedCentroid = convexPolytope3D.getCentroid();

         EuclidCoreTestTools.assertEquals(expectedCentroid, actualCentroid, EPSILON);
      }
   }

   @Test
   public void testIsFirstValueMinimum()
   {
      Random random = new Random(65467547);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMin = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[4];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isLesser = (j & mask) != 0;
               values[k] = possibleMin + (isLesser ? -values[k] : +values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMinimum(possibleMin, values[0], values[1], values[2], values[3]);
            assertEquals(isMinimumValue, actualResult);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMin = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[3];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isLesser = (j & mask) != 0;
               values[k] = possibleMin + (isLesser ? -values[k] : +values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMinimum(possibleMin, values[0], values[1], values[2]);
            assertEquals(isMinimumValue, actualResult);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMin = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[2];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isLesser = (j & mask) != 0;
               values[k] = possibleMin + (isLesser ? -values[k] : +values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMinimum(possibleMin, values[0], values[1]);
            assertEquals(isMinimumValue, actualResult);
         }
      }
   }

   @Test
   public void testIsFirstValueMaximum()
   {
      Random random = new Random(65467547);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMax = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[4];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isGreater = (j & mask) != 0;
               values[k] = possibleMax + (isGreater ? +values[k] : -values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMaximum(possibleMax, values[0], values[1], values[2], values[3]);
            assertEquals(isMinimumValue, actualResult);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMax = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[3];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isGreater = (j & mask) != 0;
               values[k] = possibleMax + (isGreater ? +values[k] : -values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMaximum(possibleMax, values[0], values[1], values[2]);
            assertEquals(isMinimumValue, actualResult);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double possibleMax = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] values = new double[2];

         int numberOfCombinations = (int) Math.pow(2, values.length);

         for (int j = 0; j < numberOfCombinations; j++)
         {
            int currentByte = 0;
            boolean isMinimumValue = j == 0;

            for (int k = 0; k < values.length; k++)
            {
               values[k] = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
               int mask = (int) Math.pow(2, currentByte);
               boolean isGreater = (j & mask) != 0;
               values[k] = possibleMax + (isGreater ? +values[k] : -values[k]);
               currentByte++;
            }

            boolean actualResult = EuclidShapeTools.isFirstValueMaximum(possibleMax, values[0], values[1]);
            assertEquals(isMinimumValue, actualResult);
         }
      }
   }

   @Test
   public void testGeometricallyEduals()
   {
      Random random = new Random(5474);

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing the tolerance along the normal vector alone.
         double normalEpsilon = random.nextDouble();
         double tangentialEpsilon = Double.POSITIVE_INFINITY;

         Vector3D normal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D deNormalizedNormal = new Vector3D();
         deNormalizedNormal.setAndScale(EuclidCoreRandomTools.nextDouble(random, 100.0), normal);
         Vector3D tangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D pointB = new Point3D();

         // Build pointB such that it should be equal to pointA
         pointB.scaleAdd(random.nextDouble() * normalEpsilon, normal, pointA);
         pointB.scaleAdd(random.nextDouble() * 100.0, tangent, pointB);
         assertTrue(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);

         // Build pointB such that it should be not equal to pointA
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * normalEpsilon, normal, pointA);
         pointB.scaleAdd(random.nextDouble() * 100.0, tangent, pointB);
         assertFalse(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing the tolerance orthogonal to the normal vector alone.
         double normalEpsilon = Double.POSITIVE_INFINITY;
         double tangentialEpsilon = random.nextDouble();

         Vector3D normal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D deNormalizedNormal = new Vector3D();
         deNormalizedNormal.setAndScale(EuclidCoreRandomTools.nextDouble(random, 100.0), normal);
         Vector3D tangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D pointB = new Point3D();

         // Build pointB such that it should be equal to pointA
         pointB.scaleAdd(random.nextDouble() * 100.0, normal, pointA);
         pointB.scaleAdd(random.nextDouble() * tangentialEpsilon, tangent, pointB);
         assertTrue(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);

         // Build pointB such that it should be not equal to pointA
         pointB.scaleAdd(random.nextDouble() * 100.0, normal, pointA);
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * tangentialEpsilon, tangent, pointB);
         assertFalse(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing with both tolerances.
         double normalEpsilon = random.nextDouble();
         double tangentialEpsilon = random.nextDouble();

         Vector3D normal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D deNormalizedNormal = new Vector3D();
         deNormalizedNormal.setAndScale(EuclidCoreRandomTools.nextDouble(random, 100.0), normal);
         Vector3D tangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         Point3D pointA = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D pointB = new Point3D();

         // Build pointB such that it should be equal to pointA
         pointB.scaleAdd(random.nextDouble() * normalEpsilon, normal, pointA);
         pointB.scaleAdd(random.nextDouble() * tangentialEpsilon, tangent, pointB);
         assertTrue(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);

         // Build pointB such that it should be not equal to pointA because of the normal part.
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * normalEpsilon, normal, pointA);
         pointB.scaleAdd(random.nextDouble() * tangentialEpsilon, tangent, pointB);
         assertFalse(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);

         // Build pointB such that it should be not equal to pointA because of the tangential part.
         pointB.scaleAdd(random.nextDouble() * normalEpsilon, normal, pointA);
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * tangentialEpsilon, tangent, pointB);
         assertFalse(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);

         // Build pointB such that it should be not equal to pointA because of both normal and tagential parts.
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * normalEpsilon, normal, pointA);
         pointB.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0) * tangentialEpsilon, tangent, pointB);
         assertFalse(EuclidShapeTools.geometricallyEquals(pointA, pointB, deNormalizedNormal, normalEpsilon, tangentialEpsilon), "Iteration " + i);
      }
   }
}

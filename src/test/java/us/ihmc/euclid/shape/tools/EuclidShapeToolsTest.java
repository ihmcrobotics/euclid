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
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
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

         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid, actualCentroid, EPSILON);
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

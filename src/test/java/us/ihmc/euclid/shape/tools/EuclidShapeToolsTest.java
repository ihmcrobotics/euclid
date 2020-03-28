package us.ihmc.euclid.shape.tools;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidShapeToolsTest
{

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

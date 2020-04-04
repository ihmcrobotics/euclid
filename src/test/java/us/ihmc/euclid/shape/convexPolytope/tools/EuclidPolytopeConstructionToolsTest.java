package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidPolytopeConstructionToolsTest
{
   private final static int ITERATIONS = EuclidTestConstants.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testEigenVector() throws Exception
   {
      Random random = new Random(4535);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double minX = 500.0;
         double maxX = 1000.0;
         double minY = 50.0;
         double maxY = 100.0;
         double minZ = 0.0;
         double maxZ = 25.0;
         int numberOfPoints = random.nextInt(1000) + 100;
         List<Point3D> points = IntStream.range(0, numberOfPoints).mapToObj(h -> EuclidCoreRandomTools.nextPoint3D(random, minX, maxX, minY, maxY, minZ, maxZ))
                                         .collect(Collectors.toList());
         points.stream().filter(p -> random.nextBoolean()).forEach(Point3D::negate);

         DenseMatrix64F actualCovariance = new DenseMatrix64F(3, 3);
         EuclidPolytopeConstructionTools.computeCovariance3D(points, null, actualCovariance);

         Vector3D actualNormal = new Vector3D(1.0, 1.0, 1.0);
         EuclidPolytopeConstructionTools.updateFace3DNormal(actualCovariance, actualNormal);

         Vector3D expectedNormal = new Vector3D(Axis3D.Z);

         String errorMessage = "Iteration" + i + ", nPoints: " + numberOfPoints + ", angle: " + expectedNormal.angle(actualNormal);
         assertTrue(EuclidGeometryTools.areVector3DsParallel(expectedNormal, actualNormal, 0.30), errorMessage);

         actualNormal = new Vector3D(-1.0, -1.0, -1.0);
         EuclidPolytopeConstructionTools.updateFace3DNormal(DecompositionFactory.eig(3, true, true), actualCovariance, actualNormal);

         expectedNormal.negate();

         errorMessage = "Iteration" + i + ", nPoints: " + numberOfPoints + ", angle: " + expectedNormal.angle(actualNormal);
         assertTrue(EuclidGeometryTools.areVector3DsParallel(expectedNormal, actualNormal, 0.30), errorMessage);
      }
   }

   @Test
   public void testEigenVectorBug() throws Exception
   { // Reproducing a bug where eigenValue.getY() < eigenValue.getZ()
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         List<Point3D> vertices = EuclidShapeRandomTools.nextCircleBasedConvexPolygon3D(random, 5.0, 1.0, 15, expectedNormal);

         for (int j = 3; j <= vertices.size(); j++)
         {
            DenseMatrix64F actualCovariance = new DenseMatrix64F(3, 3);
            Vector3D actualNormal = new Vector3D();
            EuclidPolytopeConstructionTools.computeCovariance3D(vertices.subList(0, j), null, actualCovariance);
            EuclidPolytopeConstructionTools.updateFace3DNormal(actualCovariance, actualNormal);

            String errorMessage = "Iteration" + i + ", nPoints: " + j + ", angle: " + expectedNormal.angle(actualNormal);
            assertTrue(EuclidGeometryTools.areVector3DsParallel(expectedNormal, actualNormal, 1.0e-5), errorMessage);
         }
      }
   }

   @Test
   public void testCovariance3D()
   {
      Random random = new Random(4524523);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfPoints = random.nextInt(100) + 3;
         double maxAbsoluteX = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         double maxAbsoluteY = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         double maxAbsoluteZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         List<Point3D> points = IntStream.range(0, numberOfPoints)
                                         .mapToObj(h -> EuclidCoreRandomTools.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ))
                                         .collect(Collectors.toList());

         DenseMatrix64F actualCovariance = new DenseMatrix64F(3, 3);
         EuclidPolytopeConstructionTools.computeCovariance3D(points, null, actualCovariance);
         Matrix3D expectedCovariance = computeCovarianceMatrix(points);

         double maxValue = 1.0;

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               maxValue = Math.max(maxValue, Math.abs(expectedCovariance.getElement(row, column)));
            }
         }

         EuclidCoreTestTools.assertMatrix3DEquals(expectedCovariance, new Matrix3D(actualCovariance), EPSILON * maxValue);
      }
   }

   @Test
   void testComputeConvexPolygon3DArea() throws Exception
   {
      Random random = new Random(3534);

      for (int i = 0; i < ITERATIONS; i++)
      { // Comparing when staying in the XY-plane
         List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 5.0, 1.0, 20);
         List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).collect(Collectors.toList());

         Point2D centroid2D = new Point2D();
         double expectedArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(circleBasedConvexPolygon2D,
                                                                                     circleBasedConvexPolygon2D.size(),
                                                                                     true,
                                                                                     centroid2D);
         Point3D expectedCentroid3D = new Point3D(centroid2D);

         Point3D actualCentroid3D = new Point3D();
         double actualArea = EuclidPolytopeConstructionTools.computeConvexPolygon3DArea(circleBasedConvexPolygon3D,
                                                                                        Axis3D.Z,
                                                                                        circleBasedConvexPolygon3D.size(),
                                                                                        true,
                                                                                        actualCentroid3D);

         assertEquals(expectedArea, actualArea, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid3D, actualCentroid3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Applying a transform when switching to 3D
         List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 5.0, 1.0, 20);
         Point2D centroid2D = new Point2D();
         double expectedArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(circleBasedConvexPolygon2D,
                                                                                     circleBasedConvexPolygon2D.size(),
                                                                                     true,
                                                                                     centroid2D);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D normal = new Vector3D();
         transform.getRotation().getColumn(2, normal);

         List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).peek(transform::transform)
                                                                              .collect(Collectors.toList());

         Point3D expectedCentroid3D = new Point3D(centroid2D);
         expectedCentroid3D.applyTransform(transform);

         Point3D actualCentroid3D = new Point3D();
         double actualArea = EuclidPolytopeConstructionTools.computeConvexPolygon3DArea(circleBasedConvexPolygon3D,
                                                                                        normal,
                                                                                        circleBasedConvexPolygon3D.size(),
                                                                                        true,
                                                                                        actualCentroid3D);

         assertEquals(expectedArea, actualArea, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid3D, actualCentroid3D, EPSILON);
      }
   }

   /**
    * Using the actual formula of the covariance matrix,
    * <a href="https://en.wikipedia.org/wiki/Principal_component_analysis"> here</a>.
    */
   private static Matrix3D computeCovarianceMatrix(List<? extends Point3DReadOnly> dataset)
   {
      DenseMatrix64F covariance = new DenseMatrix64F(3, 3);
      int n = dataset.size();
      DenseMatrix64F datasetMatrix = new DenseMatrix64F(n, 3);

      Point3D average = EuclidGeometryTools.averagePoint3Ds(dataset);

      for (int i = 0; i < n; i++)
      {
         Point3DReadOnly dataPoint = dataset.get(i);
         datasetMatrix.set(i, 0, dataPoint.getX() - average.getX());
         datasetMatrix.set(i, 1, dataPoint.getY() - average.getY());
         datasetMatrix.set(i, 2, dataPoint.getZ() - average.getZ());
      }

      CommonOps.multInner(datasetMatrix, covariance);

      CommonOps.scale(1.0 / n, covariance);

      return new Matrix3D(covariance);
   }
}

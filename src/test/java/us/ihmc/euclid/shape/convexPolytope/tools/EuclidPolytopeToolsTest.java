package us.ihmc.euclid.shape.convexPolytope.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.testSuite.EuclidTestSuite;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidPolytopeToolsTest
{
   private final static int ITERATIONS = EuclidTestSuite.ITERATIONS;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSignedDistanceFromPoint3DToLine3D() throws Exception
   {
      Random random = new Random(243234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ensure consistency with the 2D method.
         Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
         Line2D line2D = new Line2D(new Point2D(line3D.getPoint()), new Vector2D(line3D.getDirection()));

         Point3D query3D = EuclidCoreRandomTools.nextPoint3D(random);
         Point2D query2D = new Point2D(query3D);

         Vector3DReadOnly planeNormal = Axis.Z;

         double expected = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query2D, line2D.getPoint(), line2D.getDirection());
         double actual = EuclidPolytopeTools.signedDistanceFromPoint3DToLine3D(query3D, line3D.getPoint(), line3D.getDirection(), planeNormal);
         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testIsPoint3DOnSideOfLine3D() throws Exception
   {
      Random random = new Random(243234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Ensure consistency with the 2D method.
         Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
         Line2D line2D = new Line2D(new Point2D(line3D.getPoint()), new Vector2D(line3D.getDirection()));

         Point3D query3D = EuclidCoreRandomTools.nextPoint3D(random);
         Point2D query2D = new Point2D(query3D);

         Vector3DReadOnly planeNormal = Axis.Z;

         boolean testForLeftSide = random.nextBoolean();
         boolean expected = EuclidGeometryTools.isPoint2DOnSideOfLine2D(query2D, line2D.getPoint(), line2D.getDirection(), testForLeftSide);
         boolean actual = EuclidPolytopeTools.isPoint3DOnSideOfLine3D(query3D, line3D.getPoint(), line3D.getDirection(), planeNormal, testForLeftSide);
         assertEquals(expected, actual);
      }
   }

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

         Matrix3D actualCovariance = new Matrix3D();
         Vector3D[] actual = {new Vector3D(), new Vector3D(), new Vector3D()};
         EuclidPolytopeTools.computeCovariance3D(points, null, actualCovariance);
         Point3D eigenValues = new Point3D();
         EuclidPolytopeTools.computeEigenVectors(actualCovariance, eigenValues, actual[0], actual[1], actual[2]);

         Vector3D[] expected = {new Vector3D(Axis.X), new Vector3D(Axis.Y), new Vector3D(Axis.Z)};

         assertTrue(eigenValues.getX() > eigenValues.getY());
         assertTrue(eigenValues.getY() > eigenValues.getZ());

         for (int j = 0; j < 3; j++)
         {
            assertEquals(0.0, Math.abs(actual[j].dot(actual[(j + 1) % 3])), EPSILON);
            assertEquals(1.0, actual[j].length(), EPSILON);

            String errorMessage = "Iteration" + i + ", nPoints: " + numberOfPoints + ", angle: " + expected[j].angle(actual[j]);
            assertTrue(EuclidGeometryTools.areVector3DsParallel(expected[j], actual[j], 0.30), errorMessage);
         }
      }
   }

   @Test
   public void testEigenVectorBug() throws Exception
   { // Reproducing a bug where eigenValue.getY() < eigenValue.getZ()
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         List<Point3D> vertices = EuclidPolytopeRandomTools.nextCircleBasedConvexPolygon3D(random, 5.0, 1.0, 15, expectedNormal);

         for (int j = 3; j <= vertices.size(); j++)
         {
            Matrix3D actualCovariance = new Matrix3D();
            Vector3D actualNormal = new Vector3D();
            EuclidPolytopeTools.computeCovariance3D(vertices.subList(0, j), null, actualCovariance);
            Point3D eigenValues = new Point3D();
            EuclidPolytopeTools.computeEigenVectors(actualCovariance, eigenValues, null, null, actualNormal);

            assertTrue(eigenValues.getX() > eigenValues.getY());
            assertTrue(eigenValues.getY() > eigenValues.getZ());

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

         Matrix3D actualCovariance = new Matrix3D();
         EuclidPolytopeTools.computeCovariance3D(points, null, actualCovariance);
         Matrix3D expectedCovariance = computeCovarianceMatrix(points);

         double maxValue = 1.0;

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
            {
               maxValue = Math.max(maxValue, Math.abs(expectedCovariance.getElement(row, column)));
            }
         }

         EuclidCoreTestTools.assertMatrix3DEquals(expectedCovariance, actualCovariance, EPSILON * maxValue);
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
         double expectedArea = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(circleBasedConvexPolygon2D, circleBasedConvexPolygon2D.size(), true, centroid2D);
         Point3D expectedCentroid3D = new Point3D(centroid2D);

         Point3D actualCentroid3D = new Point3D();
         double actualArea = EuclidPolytopeTools.computeConvexPolygon3DArea(circleBasedConvexPolygon3D, Axis.Z, circleBasedConvexPolygon3D.size(), true, actualCentroid3D);

         assertEquals(expectedArea, actualArea, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCentroid3D, actualCentroid3D, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      { // Applying a transform when switching to 3D
         List<Point2D> circleBasedConvexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 5.0, 1.0, 20);
         Point2D centroid2D = new Point2D();
         double expectedArea = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(circleBasedConvexPolygon2D, circleBasedConvexPolygon2D.size(), true, centroid2D);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D normal = new Vector3D();
         transform.getRotation().getColumn(2, normal);

         List<Point3D> circleBasedConvexPolygon3D = circleBasedConvexPolygon2D.stream().map(Point3D::new).peek(transform::transform).collect(Collectors.toList());
         
         Point3D expectedCentroid3D = new Point3D(centroid2D);
         expectedCentroid3D.applyTransform(transform);
         
         Point3D actualCentroid3D = new Point3D();
         double actualArea = EuclidPolytopeTools.computeConvexPolygon3DArea(circleBasedConvexPolygon3D, normal, circleBasedConvexPolygon3D.size(), true, actualCentroid3D);
         
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

      CommonOps.scale(1.0 / (double) n, covariance);

      return new Matrix3D(covariance);
   }
}

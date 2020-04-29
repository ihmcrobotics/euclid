package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class ConvexPolygon2DBasicsTest<T extends ConvexPolygon2DBasics>
{
   public static final boolean VERBOSE = false;
   public static final double EPSILON = 1.0e-10;

   public abstract T createEmptyConvexPolygon2D();

   public abstract T createRandomConvexPolygon2D(Random random);

   public abstract T createConvexPolygon2D(Vertex2DSupplier supplier);

   @Test
   public void testUpdateBoundingBox() throws Exception
   {
      Random random = new Random(432536);

      {
         T polygon = createEmptyConvexPolygon2D();
         Point2DBasics maxPoint = polygon.getBoundingBox().getMaxPoint();
         Point2DBasics minPoint = polygon.getBoundingBox().getMinPoint();

         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(maxPoint);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(minPoint);

         polygon.addVertex(new Point2D());
         polygon.update();
         EuclidCoreTestTools.assertTuple2DIsSetToZero(maxPoint);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(minPoint);

         polygon.addVertex(new Point2D(1.0, 1.0));
         polygon.update();
         EuclidCoreTestTools.assertTuple2DIsSetToZero(minPoint);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 1.0), maxPoint, EPSILON);

         polygon.translate(1.0, 1.0);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 1.0), minPoint, EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, 2.0), maxPoint, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T polygon = createEmptyConvexPolygon2D();
         List<Point2D> points = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, 1.0, 100);
         polygon.set(Vertex2DSupplier.asVertex2DSupplier(points));
         for (Point2D point : points)
         {
            assertTrue(polygon.getBoundingBox().isInsideInclusive(point));
         }
         polygon.translate(10.0, 10.0);
         for (Point2D point : points)
         {
            assertFalse(polygon.getBoundingBox().isInsideInclusive(point));
         }
         polygon.clearAndUpdate();

         Point2DBasics maxPoint = polygon.getBoundingBox().getMaxPoint();
         Point2DBasics minPoint = polygon.getBoundingBox().getMinPoint();
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(maxPoint);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(minPoint);
         polygon.clear();
         polygon.update();
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(maxPoint);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(minPoint);
      }
   }

   @Test
   public void testTranslate()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(-0.1, 0.0);
      polygon.translate(translation1);
      assertTrue(polygon.getVertex(0).epsilonEquals(translation1, EPSILON));
   }

   @Test
   public void testClear()
   {
      ArrayList<Point2D> verticesList = new ArrayList<>();
      verticesList.add(new Point2D(0.0, 0.0));
      verticesList.add(new Point2D(0.0, 1.0));
      verticesList.add(new Point2D(1.0, 0.0));
      verticesList.add(new Point2D(1.0, 1.0));

      T list = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesList));
      assertEquals(4.0, list.getNumberOfVertices(), EPSILON, "Number of vertices should be 4");
      assertTrue(list.isUpToDate());
      list.clearAndUpdate();
      assertEquals(0.0, list.getNumberOfVertices(), EPSILON, "Number of vertices should be 0");
      assertTrue(list.isUpToDate());
      list.clear();
      assertFalse(list.isUpToDate());
      list.clearAndUpdate();
      assertTrue(list.isUpToDate());

      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(list.getBoundingBox().getMinPoint());
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(list.getBoundingBox().getMaxPoint());
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(list.getCentroid());
      assertTrue(Double.isNaN(list.getArea()));
   }

   @Test
   public void testSetAndUpdates()
   {
      T doubleInt = createEmptyConvexPolygon2D();
      int numberOfVertices = 4;
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      doubleInt.set(Vertex2DSupplier.asVertex2DSupplier(verticesArray, numberOfVertices));
      assertEquals(4.0, doubleInt.getNumberOfVertices(), EPSILON, "Number of vertices should be 4");
      assertTrue(doubleInt.isUpToDate());
   }

   @Test
   public void testGetCentroid()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      T doubles = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));
      Point2D centroid = new Point2D();

      centroid.set(doubles.getCentroid());
      assertEquals(centroid, doubles.getCentroid(), "Centroids should be equal");
   }

   @Test
   public void testGetBoundingBox()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      T doubles = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));
      BoundingBox2DBasics box = doubles.getBoundingBox();

      assertEquals(box.getMinPoint().getX(), 0.0, EPSILON, "Bounding boxes should be equal");
      assertEquals(box.getMinPoint().getX(), 0.0, EPSILON, "Bounding boxes should be equal");
      assertEquals(box.getMaxPoint().getY(), 1.0, EPSILON, "Bounding boxes should be equal");
      assertEquals(box.getMaxPoint().getY(), 1.0, EPSILON, "Bounding boxes should be equal");
   }

   @Test
   public void testGetNextVertexCCWGetPreviousVertexCCW()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      T doubles = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));

      Point2DReadOnly oneNext = doubles.getNextVertexCCW(0); //(1.0, 0.0)
      Point2DReadOnly twoNext = doubles.getNextVertexCCW(1); //(1.0, 1.0)
      Point2DReadOnly threeNext = doubles.getNextVertexCCW(2); //(0.0, 1.0)
      Point2DReadOnly fourNext = doubles.getNextVertexCCW(3); //(0.0, 0.0)

      Point2DReadOnly onePrev = doubles.getPreviousVertexCCW(0); //(0.0, 1.0)
      Point2DReadOnly twoPrev = doubles.getPreviousVertexCCW(1); //(0.0, 0.0)
      Point2DReadOnly threePrev = doubles.getPreviousVertexCCW(2); //(1.0, 0.0)
      Point2DReadOnly fourPrev = doubles.getPreviousVertexCCW(3); //(1.0, 1.0)

      EuclidCoreTestTools.assertTuple2DEquals("Points should be equal", oneNext, threePrev, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("Points should be equal", twoNext, fourPrev, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("Points should be equal", threeNext, onePrev, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("Points should be equal", fourNext, twoPrev, EPSILON);
   }

   @Test
   public void testScale()
   {
      double scaleFactor = 2.0;
      double[][] verticesArray = {{-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};

      T polygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));
      polygon.scale(scaleFactor);

      Point2DReadOnly oneNext = polygon.getNextVertexCCW(0); //(2.0, -2.0)
      Point2DReadOnly twoNext = polygon.getNextVertexCCW(1); //(2.0, 2.0)
      Point2DReadOnly threeNext = polygon.getNextVertexCCW(2); //(-2.0, 2.0)
      Point2DReadOnly fourNext = polygon.getNextVertexCCW(3); //(-2.0, -2.0)

      Point2D one = new Point2D(2.0, -2.0);
      Point2D two = new Point2D(2.0, 2.0);
      Point2D three = new Point2D(-2.0, 2.0);
      Point2D four = new Point2D(-2.0, -2.0);

      EuclidCoreTestTools.assertTuple2DEquals("These should be equal", oneNext, one, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("These should be equal", twoNext, two, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("These should be equal", threeNext, three, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals("These should be equal", fourNext, four, EPSILON);
   }

   @Test
   public void testIsPointInside()
   {
      Random random = new Random(4564656L);
      double[][] verticesArray = {{-10.0, 10.0}, {10.0, 10.0}, {10.0, -10.0}, {-10.0, -10.0}};
      T doubles = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesArray));

      for (int i = 0; i < 10; i++)
      {
         int x = random.nextInt(10);
         int y = random.nextInt(10);
         assertTrue(doubles.isPointInside(x, y));
      }
   }

   @Test
   public void testConstructorWithRepeatedPoints()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<>();
      listOfPoints.add(new Point2D(0.0, 0.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 0.0));
      listOfPoints.add(new Point2D(0.0, 0.0));
      T convexPolygon2d = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      // Above point list contains 3 unique points
      assertEquals(3, convexPolygon2d.getNumberOfVertices());
   }

   @Test
   public void testGetClosestPointToRay1()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(2.0, 0.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, 0.0), polygon.getClosestPointWithRay(ray1), EPSILON);

      Line2D ray2 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(0.5, 0.5));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(4.0 / 5.0, 3.0 / 5.0), polygon.getClosestPointWithRay(ray2), EPSILON);

      Line2D ray3 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(-0.5, 0.1));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 1.0), polygon.getClosestPointWithRay(ray3), EPSILON);

      Line2D ray4 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.0, 0.1));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray4), EPSILON);

      Line2D ray5 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.3));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray5), EPSILON);

      Line2D ray6 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(-0.3, -0.3));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray6), EPSILON);

      Line2D ray7 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.31));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray7), EPSILON);

      Line2D ray8 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.29));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 1.0), polygon.getClosestPointWithRay(ray8), EPSILON);

      Line2D ray9 = new Line2D(new Point2D(1.75, -0.75), new Vector2D(1.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -0.5), polygon.getClosestPointWithRay(ray9), EPSILON);

      Line2D ray10 = new Line2D(new Point2D(1.75, -0.75), new Vector2D(-0.3, -0.3));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -0.5), polygon.getClosestPointWithRay(ray10), EPSILON);

      Line2D ray11 = new Line2D(new Point2D(1.0, -1.2), new Vector2D(-2.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.getClosestPointWithRay(ray11), EPSILON);

      Line2D ray12 = new Line2D(new Point2D(1.0, -1.2), new Vector2D(2.0, -1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.getClosestPointWithRay(ray12), EPSILON);

      Line2D ray13 = new Line2D(new Point2D(-0.1, -0.7), new Vector2D(-2.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, -0.5), polygon.getClosestPointWithRay(ray13), EPSILON);
   }

   @Test
   public void testGetClosestPointToRay2()
   {
      T polygon = createEmptyConvexPolygon2D();
      assertTrue(polygon.getClosestPointWithRay(new Line2D(0.0, 0.0, 1.0, 0.0)) == null);
   }

   @Test
   public void testGetClosestPointToRay3()
   {
      Point2D vertex = new Point2D(1.0, -1.0);

      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(vertex);
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(vertex, polygon.getClosestPointWithRay(ray1), EPSILON);

      Line2D ray2 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      EuclidCoreTestTools.assertTuple2DEquals(vertex, polygon.getClosestPointWithRay(ray2), EPSILON);
   }

   @Test
   public void testGetClosestPointToRay4()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(2.0, -5.0));
      polygon.addVertex(new Point2D(1.0, -6.0));
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(1.0, -5.0), new Vector2D(1.0, 0.1));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, -5.0), polygon.getClosestPointWithRay(ray1), EPSILON);

      Line2D ray2 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.3));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(2.0, -5.0), polygon.getClosestPointWithRay(ray2), EPSILON);

      Line2D ray3 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.8));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray3), EPSILON);

      Line2D ray4 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(1.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray4), EPSILON);

      Line2D ray5 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(-1.0, -1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray5), EPSILON);

      Line2D ray6 = new Line2D(new Point2D(1.75, -5.75), new Vector2D(1.0, 1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray6), EPSILON);

      Line2D ray7 = new Line2D(new Point2D(1.75, -5.75), new Vector2D(-1.0, -1.0));
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray7), EPSILON);
   }

   @Test
   public void testNANRay()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<>();
      listOfPoints.add(new Point2D(0.11429999999999998, 0.1397));
      listOfPoints.add(new Point2D(0.11429999999999998, 0.04444999999999999));
      listOfPoints.add(new Point2D(-0.047625, 0.04444999999999999));
      listOfPoints.add(new Point2D(-0.047625, 0.1397));

      T convexPolygon2d = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      Point2D pont2d = new Point2D(Double.NaN, Double.NaN);
      Vector2D vector2d = new Vector2D(Double.NaN, Double.NaN);
      Line2D line2d = new Line2D(pont2d, vector2d);

      convexPolygon2d.intersectionWithRay(line2d);
   }

   @Test
   public void testIntersectionWithLine1()
   {
      // add in order so vertices do not get changed when update is called.
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D(0.6, 0.4);
      Point2D result2 = new Point2D(0.1, 0.9);

      Line2D line1 = new Line2D(new Point2D(0.0, 0.5), new Vector2D(0.1, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(-0.5, 0.5), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(1.0, 0.0), new Vector2D(0.0, -8.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);
      assertTrue(polygon.intersectionWith(line2, result1, result2) == 1);
      EuclidCoreTestTools.assertTuple2DEquals(expected2[0], result1, EPSILON);

      Line2D line3 = new Line2D(new Point2D(0.0, 1.0), new Vector2D(0.5, 0.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(0.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(line3), false);
      assertTrue(polygon.intersectionWith(line3, result1, result2) == 2);
      EuclidCoreTestTools.assertTuple2DEquals(expected3[0], result1, EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expected3[1], result2, EPSILON);

      Line2D line4 = new Line2D(new Point2D(0.5, 10.0), new Vector2D(0.0, 0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(0.5, 1.0), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected4, polygon.intersectionWith(line4), false);

      Line2D line5 = new Line2D(new Point2D(-1.0, -0.5), new Vector2D(1.0, 1.0));
      Point2D[] expected5 = new Point2D[] {new Point2D(-0.5, 0.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(line5), false);

      Line2D line6 = new Line2D(new Point2D(0.0, -1.5), new Vector2D(1.0, 1.0));
      Point2D[] expected6 = null;
      result1.set(0.0, 0.0);
      result2.set(0.0, 0.0);
      assertPointsEqual(expected6, polygon.intersectionWith(line6), false);
      assertTrue(polygon.intersectionWith(line6, result1, result2) == 0);

      Line2D line7 = new Line2D(new Point2D(0.0, -1.5), new Vector2D(0.0, 2.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(line7), false);
   }

   @Test
   public void testIntersectionWithLine2()
   {
      // line polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(-1.0, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected1 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(-0.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected2 = new Point2D[] {new Point2D(-0.5, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);

      Line2D line3 = new Line2D(new Point2D(1.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWith(line3), false);

      Line2D line4 = new Line2D(new Point2D(-0.8, 0.0), new Vector2D(0.1, 0.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected4, polygon.intersectionWith(line4), false);

      Line2D line5 = new Line2D(new Point2D(1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected5 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(line5), false);

      Line2D line6 = new Line2D(new Point2D(-1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(line6), false);
   }

   @Test
   public void testIntersectionWithLine3()
   {
      // point polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(3.0, 1.0), new Vector2D(-2.0, -1.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(2.0, 1.0), new Vector2D(-1.3, -0.8));
      Point2D[] expected2 = null;
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);
   }

   @Test
   public void testIntersectionWithLine4()
   {
      // empty polygon
      T polygon = createEmptyConvexPolygon2D();

      Line2D line1 = new Line2D(new Point2D(3.0, 1.0), new Vector2D(-1.6, -0.8));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);
   }

   @Test
   public void testIntersectionWithRay1()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      Line2D ray1 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWithRay(ray1), false);
      assertTrue(polygon.intersectionWithRay(ray1, result1, result2) == 1);

      Line2D ray2 = new Line2D(new Point2D(-1.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWithRay(ray2), false);
      assertTrue(polygon.intersectionWithRay(ray2, result1, result2) == 2);

      Line2D ray3 = new Line2D(new Point2D(2.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWithRay(ray3), false);
      assertTrue(polygon.intersectionWithRay(ray3, result1, result2) == 0);

      Line2D ray4 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected4, polygon.intersectionWithRay(ray4), false);
      assertTrue(polygon.intersectionWithRay(ray4, result1, result2) == 1);

      Line2D ray5 = new Line2D(new Point2D(1.5, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, polygon.intersectionWithRay(ray5), false);
      assertTrue(polygon.intersectionWithRay(ray5, result1, result2) == 0);

      Line2D ray6 = new Line2D(new Point2D(-1.0, -2.0), new Vector2D(0.3, 0.3));
      Point2D[] expected6 = new Point2D[] {new Point2D(0.0, -1.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected6, polygon.intersectionWithRay(ray6), false);
      assertTrue(polygon.intersectionWithRay(ray6, result1, result2) == 2);

      Line2D ray7 = new Line2D(new Point2D(-1.0, -2.0), new Vector2D(0.0, 1.7));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, -1.0), new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWithRay(ray7), false);
      assertTrue(polygon.intersectionWithRay(ray7, result1, result2) == 2);

      Line2D ray8 = new Line2D(new Point2D(-0.5, 0.5), new Vector2D(-0.3, -0.3));
      Point2D[] expected8 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected8, polygon.intersectionWithRay(ray8), false);
      assertTrue(polygon.intersectionWithRay(ray8, result1, result2) == 1);

      Line2D ray9 = new Line2D(new Point2D(-0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected9 = new Point2D[] {new Point2D(-0.25, 1.0)};
      assertPointsEqual(expected9, polygon.intersectionWithRay(ray9), false);
      assertTrue(polygon.intersectionWithRay(ray9, result1, result2) == 1);

      Line2D ray10 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(-0.15, 0.3));
      Point2D[] expected10 = new Point2D[] {new Point2D(0.25, 1.0)};
      assertPointsEqual(expected10, polygon.intersectionWithRay(ray10), false);
      assertTrue(polygon.intersectionWithRay(ray10, result1, result2) == 1);

      Line2D ray11 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected11 = new Point2D[] {new Point2D(0.75, 1.0)};
      assertPointsEqual(expected11, polygon.intersectionWithRay(ray11), false);
      assertTrue(polygon.intersectionWithRay(ray11, result1, result2) == 1);

      Line2D ray12 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.15, -0.3));
      Point2D[] expected12 = new Point2D[] {new Point2D(1.0, -0.5)};
      assertPointsEqual(expected12, polygon.intersectionWithRay(ray12), false);
      assertTrue(polygon.intersectionWithRay(ray12, result1, result2) == 1);

      Line2D ray13 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.0, -0.3));
      Point2D[] expected13 = new Point2D[] {new Point2D(0.5, -1.0)};
      assertPointsEqual(expected13, polygon.intersectionWithRay(ray13), false);
      assertTrue(polygon.intersectionWithRay(ray13, result1, result2) == 1);

      Line2D ray14 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.0, 0.3));
      Point2D[] expected14 = new Point2D[] {new Point2D(0.5, 1.0)};
      assertPointsEqual(expected14, polygon.intersectionWithRay(ray14), false);
      assertTrue(polygon.intersectionWithRay(ray14, result1, result2) == 1);

      Line2D ray15 = new Line2D(new Point2D(1.5, 1.5), new Vector2D(0.0, 0.3));
      Point2D[] expected15 = null;
      assertPointsEqual(expected15, polygon.intersectionWithRay(ray15), false);
      assertTrue(polygon.intersectionWithRay(ray15, result1, result2) == 0);
   }

   @Test
   public void testIsInside()
   {
      double[][] polygonPoints = new double[][] {{-0.05107802536335158, 0.04155594197133163}, {-0.05052044462374434, 0.1431544119584275},
            {0.12219695435431863, 0.14220652470109518}, {0.12219695435431865, -0.041946248489056696}, {0.12163937361471142, -0.1435447184761526},
            {-0.05107802536335154, -0.14259683121882027}};

      Point2D testPoint = new Point2D(-0.04907805548171582, 2.6934439541712686E-4);

      T polygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(polygonPoints));

      boolean isInside = polygon.isPointInside(testPoint);

      assertTrue(isInside);
   }

   @Test
   public void testIsPointInside1()
   {
      // single point polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, 1.0);
      assertTrue(polygon.isPointInside(point1, EPSILON));

      Point2D point2 = new Point2D(0.8, 0.9);
      assertFalse(polygon.isPointInside(point2));

      Point2D point3 = new Point2D(0.8, 1.1);
      assertTrue(polygon.isPointInside(point3, 0.3));

      Point2D point4 = new Point2D(1.0, 0.9);
      assertFalse(polygon.isPointInside(point4));

      Point2D point5 = new Point2D(2.0, 1.0);
      assertFalse(polygon.isPointInside(point5));
      assertTrue(polygon.isPointInside(point5, 1.0));

      Point2D point6 = new Point2D(1.0, 2.0);
      assertFalse(polygon.isPointInside(point6));
      assertTrue(polygon.isPointInside(point6, 1.0));
   }

   @Test
   public void testIsPointInside2()
   {
      {
         // line polygon
         T polygon = createEmptyConvexPolygon2D();
         polygon.addVertex(new Point2D(0.0, 0.0));
         polygon.addVertex(new Point2D(1.0, 0.0));
         polygon.update();

         assertTrue(polygon.isPointInside(0.1, 0.0, EPSILON));
         assertTrue(polygon.isPointInside(0.5, 0.0, EPSILON));
         assertTrue(polygon.isPointInside(0.9, 0.0, EPSILON));

         assertFalse(polygon.isPointInside(0.1, 0.1, EPSILON));

         assertFalse(polygon.isPointInside(1.5, 0.0, EPSILON));

         assertTrue(polygon.isPointInside(1.0, 0.0));

         assertFalse(polygon.isPointInside(1.0, EPSILON * 0.1));

         assertTrue(polygon.isPointInside(1.0, EPSILON * 0.1, EPSILON));

         assertTrue(polygon.isPointInside(1.5, 0.0, 0.5));
      }

      {
         // line polygon
         T polygon = createEmptyConvexPolygon2D();
         polygon.addVertex(new Point2D(0.0, 0.0));
         polygon.addVertex(new Point2D(1.0, 1.0));
         polygon.update();

         assertFalse(polygon.isPointInside(0.1, 0.0));
         assertFalse(polygon.isPointInside(0.5, 0.0));
         assertFalse(polygon.isPointInside(0.9, 0.0));

         assertFalse(polygon.isPointInside(-0.1, -0.1));
         assertTrue(polygon.isPointInside(0.1, 0.1));
         assertTrue(polygon.isPointInside(0.5, 0.5));
         assertTrue(polygon.isPointInside(0.9, 0.9));

         assertTrue(polygon.isPointInside(0.0, 0.0));
         assertTrue(polygon.isPointInside(1.0, 1.0));
      }
   }

   @Test
   public void testIsPointInside3()
   {
      // triangle polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(5.0, 0.0));
      polygon.addVertex(new Point2D(3.0, 5.0));
      polygon.update();

      Point2D point1 = new Point2D(0.3, 0.0);
      assertTrue(polygon.isPointInside(point1, EPSILON));

      Point2D point2 = new Point2D(0.0, 0.0);
      assertTrue(polygon.isPointInside(point2, EPSILON));

      Point2D point3 = new Point2D(2.0, 2.0);
      assertTrue(polygon.isPointInside(point3));

      Point2D point4 = new Point2D(1.0, 0.3);
      assertTrue(polygon.isPointInside(point4, EPSILON));

      Point2D point5 = new Point2D(-1.0, 4.0);
      assertFalse(polygon.isPointInside(point5.getX(), point5.getY(), EPSILON));

      Point2D point6 = new Point2D(6.0, 7.0);
      assertFalse(polygon.isPointInside(point6, EPSILON));

      Point2D point7 = new Point2D(10.0, 0.0);
      assertFalse(polygon.isPointInside(point7, EPSILON));

      Point2D point8 = new Point2D(0.1, 0.2);
      assertFalse(polygon.isPointInside(point8));

      Point2D point9 = new Point2D(3.5, 4.9);
      assertFalse(polygon.isPointInside(point9.getX(), point9.getY(), EPSILON));

      Point2D point10 = new Point2D(3.5, -1.0);
      assertFalse(polygon.isPointInside(point10));
   }

   @Test
   public void testIsPointInside4()
   {
      // empty polygon
      T polygon = createEmptyConvexPolygon2D();

      Point2D point1 = new Point2D(10.0, 0.0);
      assertFalse(polygon.isPointInside(point1, EPSILON));
   }

   @Test
   public void testIsPointInside5()
   {
      // foot polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(-0.06, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.19));
      polygon.addVertex(new Point2D(-0.06, -0.19));
      polygon.update();

      Point2D point1 = new Point2D(0.03, 0.0);
      assertFalse(polygon.isPointInside(point1, 0.02));

      Point2D point2 = new Point2D(0.03, -0.09);
      assertTrue(polygon.isPointInside(point2));
   }

   @Test
   public void testDistancePoint2dConvexPolygon2d()
   {
      ArrayList<Point2D> points = new ArrayList<>();
      points.add(new Point2D());
      points.add(new Point2D());
      points.add(new Point2D());
      T test = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
      test.distance(new Point2D());
   }

   @Test
   public void testOrthogonalProjection1()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(0.5, 0.5);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 0.0), polygon.orthogonalProjectionCopy(point1), EPSILON);

      Point2D point2 = new Point2D(-0.25, -0.25);
      assertNull(polygon.orthogonalProjectionCopy(point2));

      Point2D point3 = new Point2D(-2.0, -2.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, -1.0), polygon.orthogonalProjectionCopy(point3), EPSILON);

      Point2D point4 = new Point2D(-0.9, -2.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.9, -1.0), polygon.orthogonalProjectionCopy(point4), EPSILON);

      Point2D point5 = new Point2D(-1.1, -2.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, -1.0), polygon.orthogonalProjectionCopy(point5), EPSILON);

      Point2D point6 = new Point2D(1.8, -1.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point6), EPSILON);

      Point2D point7 = new Point2D(1.8, -0.8);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point7), EPSILON);

      Point2D point8 = new Point2D(0.5, 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.25, -0.25), polygon.orthogonalProjectionCopy(point8), EPSILON);

      Point2D point9 = new Point2D(0.0, 0.5);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-0.25, 0.25), polygon.orthogonalProjectionCopy(point9), EPSILON);

      Point2D point10 = new Point2D(0.0, 0.0);
      assertNull(polygon.orthogonalProjectionCopy(point10));

      Point2D point11 = new Point2D(1.0, -1.0);
      assertNull(polygon.orthogonalProjectionCopy(point11));

      Point2D point12 = new Point2D(-1.1, 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, 0.0), polygon.orthogonalProjectionCopy(point12), EPSILON);

      Point2D point13 = new Point2D(-1.5, 3.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, 1.0), polygon.orthogonalProjectionCopy(point13), EPSILON);

      Point2D point14 = new Point2D(3.0, -1.5);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point14), EPSILON);

      Point2D point15 = new Point2D(1.6, -1.5);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point15), EPSILON);

      Point2D point16 = new Point2D(-2.0, 0.9);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, 0.9), polygon.orthogonalProjectionCopy(point16), EPSILON);

      Point2D point17 = new Point2D(-2.0, -0.9);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-1.0, -0.9), polygon.orthogonalProjectionCopy(point17), EPSILON);
   }

   @Test
   public void testOrthogonalProjection2()
   {
      // empty polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.orthogonalProjectionCopy(new Point2D());
   }

   @Test
   public void testOrthogonalProjection3()
   {
      // single point polygon
      Point2D vertex = new Point2D(1.0, 2.0);
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(vertex);
      polygon.update();

      EuclidCoreTestTools.assertTuple2DEquals(vertex, polygon.orthogonalProjectionCopy(new Point2D(0.0, 0.0)), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(vertex, polygon.orthogonalProjectionCopy(new Point2D(1.0, -0.2)), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(vertex, polygon.orthogonalProjectionCopy(new Point2D(1.0, 2.0)), EPSILON);
   }

   @Test
   public void testOrthogonalProjection4()
   {
      // line polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, -1.0);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 1.0), polygon.orthogonalProjectionCopy(point1), EPSILON);

      Point2D point2 = new Point2D(3.0, 2.1);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 2.0), polygon.orthogonalProjectionCopy(point2), EPSILON);

      Point2D point3 = new Point2D(0.2, 1.2);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(1.0, 1.2), polygon.orthogonalProjectionCopy(point3), EPSILON);
   }

   @Test
   public void testGetClosestEdge()
   {
      T polygon = createSomeValidPolygon();
      Point2D point = new Point2D(1.314592, 6.0221415); // Useless Riddle: first number is easy, second one multiplied by 1e23 is called?

      assertFalse(polygon.isPointInside(point));

      LineSegment2DBasics closestEdge = polygon.getClosestEdgeCopy(point);

      Point2DBasics closestVertex = polygon.getClosestVertexCopy(point);

      int otherEdgeVertexIndex = 0;
      boolean isClosestVertexPartOfClosestEdge = false;

      Point2DReadOnly[] segmentVertices = {closestEdge.getFirstEndpoint(), closestEdge.getSecondEndpoint()};

      for (int i = 0; i < 2; i++)
      {
         Point2DReadOnly segmentVertex = segmentVertices[i];
         if (arePointsAtExactlyEqualPosition(closestVertex, segmentVertex))
         {
            isClosestVertexPartOfClosestEdge = true;
            if (i == 0)
               otherEdgeVertexIndex = 1;
         }
      }

      assertTrue(isClosestVertexPartOfClosestEdge);

      int numberOfVertices = polygon.getNumberOfVertices();
      int prevIndex = numberOfVertices - 1;
      int nextIndex = 0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly currentPoint = polygon.getVertex(i);
         if (arePointsAtExactlyEqualPosition(closestVertex, currentPoint))
         {
            if (i < numberOfVertices - 1)
               nextIndex = i + 1;

            break;
         }

         prevIndex = i;
      }

      Point2DReadOnly[] neighbourPoints = new Point2DReadOnly[2];
      neighbourPoints[0] = polygon.getVertex(prevIndex);
      neighbourPoints[1] = polygon.getVertex(nextIndex);
      int neighbourIndex = 1;
      int wrongNeighbourIndex = 0;
      boolean isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex = false;
      for (int i = 0; i < 2; i++)
      {
         Point2DReadOnly neighbourPoint = neighbourPoints[i];
         Point2DReadOnly closestEdgeVertexThatIsNotClosest = segmentVertices[otherEdgeVertexIndex];
         if (arePointsAtExactlyEqualPosition(closestEdgeVertexThatIsNotClosest, neighbourPoint))
         {
            isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex = true;
            neighbourIndex = i;
            if (i == 0)
               wrongNeighbourIndex = 1;
         }

      }

      assertTrue(isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex);

      Line2D segmentLine = new Line2D(neighbourPoints[neighbourIndex], closestVertex);
      Line2D otherLine = new Line2D(neighbourPoints[wrongNeighbourIndex], closestVertex);

      Line2DBasics interiorBiSector = segmentLine.interiorBisector(otherLine);

      boolean isPointBehindLine = interiorBiSector.isPointBehindLine(point);
      boolean isOtherEdgeVertexBehindLine = interiorBiSector.isPointBehindLine(segmentVertices[otherEdgeVertexIndex]);

      // TODO this may fail, if the point is really close to the "true" biSecotor and the biSector float calc error moves it to the wrong side...
      // TODO edge cases unsolved...
      assertEquals(isPointBehindLine, isOtherEdgeVertexBehindLine);

   }

   private boolean arePointsAtExactlyEqualPosition(Point2DReadOnly point1, Point2DReadOnly point2)
   {
      return point1.getX() == point2.getX() && point1.getY() == point2.getY();
   }

   private T createSomeValidPolygon()
   {
      double[][] polygonPoints = new double[][] {{-0.05107802536335158, 0.04155594197133163}, {-0.05052044462374434, 0.1431544119584275},
            {0.12219695435431863, 0.14220652470109518}, {0.12219695435431865, -0.041946248489056696}, {0.12163937361471142, -0.1435447184761526},
            {-0.05107802536335154, -0.14259683121882027}};

      T polygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(polygonPoints));

      return polygon;
   }

   @Test
   public void testOrthogonalProjectionPointConvexPolygon2d()
   {
      Random random = new Random(32);
      ArrayList<Point2D> pointList = new ArrayList<>();
      T convexPolygon;
      Point2D testPoint = new Point2D();

      for (int i = 4; i < 10; i++) // polygon sizes
      {
         pointList.clear();

         for (int j = 0; j < i; j++) // points from which the polygon is constructed
         {
            pointList.add(new Point2D(random.nextDouble(), random.nextDouble()));
         }

         convexPolygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));

         for (int k = 0; k < 100; k++) // testPoints for isPointInside()
         {
            testPoint.set(random.nextDouble(), random.nextDouble());

            if (VERBOSE)
            {
               if (i == 9 && k == 69)
               {
                  System.out.println("convexPolygon = " + convexPolygon);
                  System.out.println("testPoint = " + testPoint);
               }
            }

            Point2DBasics projectedPoint = convexPolygon.orthogonalProjectionCopy(testPoint);

            if (convexPolygon.isPointInside(testPoint))
               assertNull(projectedPoint);
            else
               assertTrue(convexPolygon.isPointInside(projectedPoint, 1.0E-10),
                          "Projected point was not inside the polygon for point\n" + projectedPoint + "\nand convex polygon \n" + convexPolygon);
         }
      }
   }

   @Test
   public void testGetSignedDistance1()
   {
      // single point polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Point2D point = new Point2D(2.5, 1.0);
      double distance = polygon.signedDistance(point);
      assertDistanceCorrect(EuclidCoreTools.norm(2.5, 1.0), distance);
   }

   @Test
   public void testGetSignedDistance2()
   {
      // line polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Point2D point1 = new Point2D(2.5, 1.0);
      double distance1 = polygon.signedDistance(point1);
      assertDistanceCorrect(EuclidCoreTools.norm(1.5, 1.0), distance1);

      Point2D point2 = new Point2D(0.5, 1.0);
      double distance2 = polygon.signedDistance(point2);
      assertDistanceCorrect(1.0, distance2);
   }

   @Test
   public void testGetSignedDistance3()
   {
      // triangle polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Point2D point1 = new Point2D(10.0, 10.0);
      double distance1 = polygon.signedDistance(point1);
      assertDistanceCorrect(5.0 * EuclidCoreTools.squareRoot(2.0), distance1);

      Point2D point2 = new Point2D(1.2, 1.1);
      double distance2 = polygon.signedDistance(point2);
      assertDistanceCorrect(-1.1, distance2);

      Point2D point3 = new Point2D(0.05, 9.8);
      double distance3 = polygon.signedDistance(point3);
      assertDistanceCorrect(-0.05, distance3);

      Point2D point4 = new Point2D(9.8, 0.15);
      double distance4 = polygon.signedDistance(point4);
      assertDistanceCorrect(-0.5 * EuclidCoreTools.squareRoot(0.05 * 0.05 * 2.0), distance4);

      Point2D point5 = new Point2D(5.0, -0.15);
      double distance5 = polygon.signedDistance(point5);
      assertDistanceCorrect(0.15, distance5);

      Point2D point6 = new Point2D(15.0, -0.15);
      double distance6 = polygon.signedDistance(point6);
      assertDistanceCorrect(EuclidCoreTools.norm(5.0, 0.15), distance6);
   }

   private static void assertDistanceCorrect(double expected, double actual)
   {
      assertEquals(expected, actual, EPSILON, "Distance does not equal expected.");
   }

   @Test
   public void testGetClosestEdge1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(-1.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 1.0);
      Point2D vertex4 = new Point2D(1.0, 1.0);

      // add in order so vertices do not get changed when update is called.
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.update();

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);
      LineSegment2D edge2 = new LineSegment2D(vertex2, vertex3);
      LineSegment2D edge3 = new LineSegment2D(vertex3, vertex4);
      LineSegment2D edge4 = new LineSegment2D(vertex4, vertex1);

      Point2D point1 = new Point2D(0.5, 0.1);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge4, polygon.getClosestEdgeCopy(point1), EPSILON);

      Point2D point2 = new Point2D(-0.5, -0.5);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge1, polygon.getClosestEdgeCopy(point2), EPSILON);

      Point2D point3 = new Point2D(-0.5, 0.5);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge2, polygon.getClosestEdgeCopy(point3), EPSILON);

      Point2D point4 = new Point2D(-0.5, 0.25);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge2, polygon.getClosestEdgeCopy(point4), EPSILON);

      Point2D point5 = new Point2D(-0.1, 3.0);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge2, polygon.getClosestEdgeCopy(point5), EPSILON);

      Point2D point6 = new Point2D(0.1, 0.8);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge3, polygon.getClosestEdgeCopy(point6), EPSILON);

      Point2D point7 = new Point2D(-0.11, 0.2);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge1, polygon.getClosestEdgeCopy(point7), EPSILON);
   }

   @Test
   public void testGetClosestEdge2()
   {
      Point2D vertex1 = new Point2D(2.0, 2.0);
      Point2D vertex2 = new Point2D(3.0, 3.0);

      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.update();

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);

      Point2D point1 = new Point2D(0.5, 0.1);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge1, polygon.getClosestEdgeCopy(point1), EPSILON);

      Point2D point2 = new Point2D(4.0, 4.0);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge1, polygon.getClosestEdgeCopy(point2), EPSILON);

      Point2D point3 = new Point2D(1.0, 1.0);
      EuclidGeometryTestTools.assertLineSegment2DGeometricallyEquals(edge1, polygon.getClosestEdgeCopy(point3), EPSILON);
   }

   @Test
   public void testGetClosestEdge3()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D());
      polygon.update();
      assertTrue(polygon.getClosestEdgeCopy(new Point2D()) == null);
   }

   @Test
   public void testGetClosestEdge4()
   {
      T polygon = createEmptyConvexPolygon2D();
      assertTrue(polygon.getClosestEdgeCopy(new Point2D()) == null);
   }

   @Test
   public void testGetClosestVertexPoint1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(10.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 10.0);

      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Point2D point1 = new Point2D(-1.0, -1.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex1, polygon.getClosestVertexCopy(point1), EPSILON);

      Point2D point2 = new Point2D(1.0, 1.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex1, polygon.getClosestVertexCopy(point2), EPSILON);

      Point2D point3 = new Point2D(10.0, 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex2, polygon.getClosestVertexCopy(point3), EPSILON);

      Point2D point4 = new Point2D(9.8, 0.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex2, polygon.getClosestVertexCopy(point4), EPSILON);

      Point2D point5 = new Point2D(10.0, 11.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex3, polygon.getClosestVertexCopy(point5), EPSILON);

      Point2D point6 = new Point2D(-3.0, 8.0);
      EuclidCoreTestTools.assertTuple2DEquals(vertex3, polygon.getClosestVertexCopy(point6), EPSILON);
   }

   @Test
   public void testGetClosestVertexPoint2()
   {
      // make sure the method fails as expected with an empty polygon
      T polygon = createEmptyConvexPolygon2D();
      Point2D closestVertex = new Point2D();

      assertFalse(polygon.getClosestVertex(new Point2D(), closestVertex));
      assertTrue(polygon.getClosestVertexCopy(new Point2D()) == null);
   }

   @Test
   public void testCanObserverSeeEdge1()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      // observer inside polygon can not see any outside edges
      Point2D observer1 = new Point2D(0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         assertFalse(polygon.canObserverSeeEdge(i, observer1));

      // this observer should be able to see the edge starting at vertex (0.0, 0.0)
      Point2D observer2 = new Point2D(-0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 0.0), EPSILON))
            assertTrue(polygon.canObserverSeeEdge(i, observer2));
         else
            assertFalse(polygon.canObserverSeeEdge(i, observer2));
      }

      // this observer should be able to see the edges starting at vertex (0.0, 1.0) and at (1.0, 1.0)
      Point2D observer3 = new Point2D(1.5, 1.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 1.0), EPSILON))
            assertTrue(polygon.canObserverSeeEdge(i, observer3));
         else if (polygon.getVertex(i).epsilonEquals(new Point2D(1.0, 1.0), EPSILON))
            assertTrue(polygon.canObserverSeeEdge(i, observer3));
         else
            assertFalse(polygon.canObserverSeeEdge(i, observer3));
      }
   }

   @Test
   public void testCanObserverSeeEdge2()
   {
      // line polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.update();

      // should be able to see one edge
      Point2D observer1 = new Point2D(0.0, 0.0);
      boolean seeEdge1 = polygon.canObserverSeeEdge(0, observer1);
      boolean seeEdge2 = polygon.canObserverSeeEdge(1, observer1);
      assertTrue((seeEdge1 || seeEdge2) && !(seeEdge1 && seeEdge2));
   }

   @Test
   public void testCanObserverSeeEdge3()
   {
      // point polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D observer1 = new Point2D(0.0, 0.0);
      assertFalse(polygon.canObserverSeeEdge(0, observer1));
   }

   @Test
   public void testIntersectionWithLineSegment1()
   {
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 1);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(-2.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 2);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(-0.5, 0.0), new Point2D(0.5, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 0);

      LineSegment2D segment4 = new LineSegment2D(new Point2D(-3.5, 0.0), new Point2D(-1.5, 0.0));
      Point2D[] expected4 = null;
      assertPointsEqual(expected4, polygon.intersectionWith(segment4), false);
      assertTrue(polygon.intersectionWith(segment4, result1, result2) == 0);

      LineSegment2D segment5 = new LineSegment2D(new Point2D(-1.5, 0.0), new Point2D(0.0, 1.5));
      Point2D[] expected5 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(segment5), false);
      assertTrue(polygon.intersectionWith(segment5, result1, result2) == 2);

      LineSegment2D segment6 = new LineSegment2D(new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(segment6), false);
      assertTrue(polygon.intersectionWith(segment6, result1, result2) == 2);

      LineSegment2D segment7 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(segment7), false);
      assertTrue(polygon.intersectionWith(segment7, result1, result2) == 2);

      LineSegment2D segment8 = new LineSegment2D(new Point2D(-2.5, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected8 = null;
      assertPointsEqual(expected8, polygon.intersectionWith(segment8), false);
      assertTrue(polygon.intersectionWith(segment8, result1, result2) == 0);

      LineSegment2D segment9 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(1.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected9, polygon.intersectionWith(segment9), false);
      assertTrue(polygon.intersectionWith(segment9, result1, result2) == 2);

      LineSegment2D segment10 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(1.0, 0.5));
      Point2D[] expected10 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected10, polygon.intersectionWith(segment10), false);
      result1.set(expected10[0]);
      result2.set(expected10[0]);
      assertTrue(polygon.intersectionWith(segment10, result1, result2) == 2);
      result1.set(expected10[1]);
      result2.set(expected10[1]);
      assertTrue(polygon.intersectionWith(segment10, result1, result2) == 2);

      LineSegment2D segment11 = new LineSegment2D(new Point2D(-0.5, 1.0), new Point2D(-1.0, 0.5));
      Point2D[] expected11 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected11, polygon.intersectionWith(segment11), false);
      assertTrue(polygon.intersectionWith(segment11, result1, result2) == 2);

      LineSegment2D segment12 = new LineSegment2D(new Point2D(-1.5, 0.5), new Point2D(1.5, 0.5));
      Point2D[] expected12 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected12, polygon.intersectionWith(segment12), false);
      result1.set(expected12[0]);
      result2.set(expected12[0]);
      assertTrue(polygon.intersectionWith(segment12, result1, result2) == 2);
      result1.set(expected12[1]);
      result2.set(expected12[1]);
      assertTrue(polygon.intersectionWith(segment12, result1, result2) == 2);

      LineSegment2D segment13 = new LineSegment2D(new Point2D(0.0, -1.5), new Point2D(1.5, -1.5));
      Point2D[] expected13 = null;
      assertPointsEqual(expected13, polygon.intersectionWith(segment13), false);
      assertTrue(polygon.intersectionWith(segment13, result1, result2) == 0);

      LineSegment2D segment14 = new LineSegment2D(new Point2D(0.0, 1.5), new Point2D(1.5, 1.5));
      Point2D[] expected14 = null;
      assertPointsEqual(expected14, polygon.intersectionWith(segment14), false);
      assertTrue(polygon.intersectionWith(segment14, result1, result2) == 0);

      LineSegment2D segment15 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(0.5, 1.0));
      Point2D[] expected15 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected15, polygon.intersectionWith(segment15), false);
      assertTrue(polygon.intersectionWith(segment15, result1, result2) == 2);

      LineSegment2D segment16 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(1.0, 0.5));
      Point2D[] expected16 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected16, polygon.intersectionWith(segment16), false);
      assertTrue(polygon.intersectionWith(segment16, result1, result2) == 2);

      LineSegment2D segment17 = new LineSegment2D(new Point2D(0.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected17 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected17, polygon.intersectionWith(segment17), false);
      assertTrue(polygon.intersectionWith(segment17, result1, result2) == 2);

      LineSegment2D segment18 = new LineSegment2D(new Point2D(1.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected18 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected18, polygon.intersectionWith(segment18), false);
      assertTrue(polygon.intersectionWith(segment18, result1, result2) == 2);

      LineSegment2D segment19 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(-0.5, 1.0));
      Point2D[] expected19 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected19, polygon.intersectionWith(segment19), false);
      assertTrue(polygon.intersectionWith(segment19, result1, result2) == 2);

      LineSegment2D segment20 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(-1.0, 1.0));
      Point2D[] expected20 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected20, polygon.intersectionWith(segment20), false);
      assertTrue(polygon.intersectionWith(segment20, result1, result2) == 1);

      LineSegment2D segment21 = new LineSegment2D(new Point2D(-1.0, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected21 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected21, polygon.intersectionWith(segment21), false);
      assertTrue(polygon.intersectionWith(segment21, result1, result2) == 1);

      LineSegment2D segment22 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected22 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected22, polygon.intersectionWith(segment22), false);
      assertTrue(polygon.intersectionWith(segment22, result1, result2) == 1);

      LineSegment2D segment23 = new LineSegment2D(new Point2D(1.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected23 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected23, polygon.intersectionWith(segment23), false);
      assertTrue(polygon.intersectionWith(segment23, result1, result2) == 1);

      LineSegment2D segment24 = new LineSegment2D(new Point2D(1.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected24 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected24, polygon.intersectionWith(segment24), false);
      assertTrue(polygon.intersectionWith(segment24, result1, result2) == 1);

      LineSegment2D segment25 = new LineSegment2D(new Point2D(0.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected25 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected25, polygon.intersectionWith(segment25), false);
      result1.set(expected25[0]);
      result2.set(expected25[0]);
      assertTrue(polygon.intersectionWith(segment25, result1, result2) == 1);

      LineSegment2D segment26 = new LineSegment2D(new Point2D(-1.0, -1.0), new Point2D(0.8, 1.0));
      Point2D[] expected26 = new Point2D[] {new Point2D(0.8, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected26, polygon.intersectionWith(segment26), false);
      result1.set(expected26[0]);
      result2.set(expected26[0]);
      assertTrue(polygon.intersectionWith(segment26, result1, result2) == 2);
      result1.set(expected26[1]);
      result2.set(expected26[1]);
      assertTrue(polygon.intersectionWith(segment26, result1, result2) == 2);

      LineSegment2D segment27 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0));
      Point2D[] expected27 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected27, polygon.intersectionWith(segment27), false);
      result1.set(expected27[0]);
      result2.set(expected27[0]);
      assertTrue(polygon.intersectionWith(segment27, result1, result2) == 2);
      result1.set(expected27[1]);
      result2.set(expected27[1]);
      assertTrue(polygon.intersectionWith(segment27, result1, result2) == 2);

      LineSegment2D segment28 = new LineSegment2D(new Point2D(1.0, -0.5), new Point2D(1.0, 0.0));
      Point2D[] expected28 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, -0.5)};
      assertPointsEqual(expected28, polygon.intersectionWith(segment28), false);
      result1.set(expected28[0]);
      result2.set(expected28[0]);
      assertTrue(polygon.intersectionWith(segment28, result1, result2) == 2);
      result1.set(expected28[1]);
      result2.set(expected28[1]);
      assertTrue(polygon.intersectionWith(segment28, result1, result2) == 2);

      LineSegment2D segment29 = new LineSegment2D(new Point2D(1.0, -1.5), new Point2D(1.0, 0.5));
      Point2D[] expected29 = new Point2D[] {new Point2D(1.0, 0.5), new Point2D(1.0, -1.0)};
      assertPointsEqual(expected29, polygon.intersectionWith(segment29), false);
      result1.set(expected29[0]);
      result2.set(expected29[0]);
      assertTrue(polygon.intersectionWith(segment29, result1, result2) == 2);
      result1.set(expected29[1]);
      result2.set(expected29[1]);
      assertTrue(polygon.intersectionWith(segment29, result1, result2) == 2);
   }

   @Test
   public void testIntersectionWithLineSegment2()
   {
      // empty polygon
      T polygon = createEmptyConvexPolygon2D();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D();
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 0);
   }

   @Test
   public void testIntersectionWithLineSegment3()
   {
      // point polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 0);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 1);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 1);
   }

   @Test
   public void testIntersectionWithLineSegment4()
   {
      // line polygon
      T polygon = createEmptyConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(3.0, 3.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(3.0, 3.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 2);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(1.5, 1.5), new Point2D(2.5, 2.5));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.5, 1.5), new Point2D(2.5, 2.5)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 2);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(0.5, 0.5), new Point2D(3.5, 3.5));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 2);

      LineSegment2D segment4 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(3.0, 3.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected4, polygon.intersectionWith(segment4), false);
      assertTrue(polygon.intersectionWith(segment4, result1, result2) == 2);

      LineSegment2D segment5 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(0.5, 0.5));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, polygon.intersectionWith(segment5), false);
      assertTrue(polygon.intersectionWith(segment5, result1, result2) == 0);

      LineSegment2D segment6 = new LineSegment2D(new Point2D(0.5, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(segment6), false);
      assertTrue(polygon.intersectionWith(segment6, result1, result2) == 1);

      LineSegment2D segment7 = new LineSegment2D(new Point2D(2.0, 0.5), new Point2D(2.0, 5.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(2.0, 2.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(segment7), false);
      assertTrue(polygon.intersectionWith(segment7, result1, result2) == 1);

      LineSegment2D segment8 = new LineSegment2D(new Point2D(2.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected8 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected8, polygon.intersectionWith(segment8), false);
      assertTrue(polygon.intersectionWith(segment8, result1, result2) == 1);

      LineSegment2D segment9 = new LineSegment2D(new Point2D(4.0, 4.0), new Point2D(2.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(3.0, 3.0), new Point2D(2.0, 2.0)};
      assertPointsEqual(expected9, polygon.intersectionWith(segment9), false);
      assertTrue(polygon.intersectionWith(segment9, result1, result2) == 2);
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(89762L);
      T firstPolygon, secondPolygon;
      int numberOfVertices;
      Vector2D translation;

      numberOfVertices = 3 + random.nextInt(10);

      firstPolygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random,
                                                                                                                                        0,
                                                                                                                                        0.5,
                                                                                                                                        numberOfVertices)));
      secondPolygon = createConvexPolygon2D(firstPolygon);

      assertTrue(firstPolygon.geometricallyEquals(secondPolygon, EPSILON));
      assertTrue(secondPolygon.geometricallyEquals(firstPolygon, EPSILON));
      assertTrue(firstPolygon.geometricallyEquals(firstPolygon, EPSILON));
      assertTrue(secondPolygon.geometricallyEquals(secondPolygon, EPSILON));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Convex polygons are only equal if all points lie within +- epsilon of each other
         numberOfVertices = 3 + random.nextInt(10);

         firstPolygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random,
                                                                                                                                           0,
                                                                                                                                           0.5,
                                                                                                                                           numberOfVertices)));
         firstPolygon.scale(10.0);
         secondPolygon = createConvexPolygon2D(firstPolygon);

         secondPolygon.applyTransform(new RigidBodyTransform(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), 0.1 * EPSILON), new Vector3D()));

         assertTrue(firstPolygon.geometricallyEquals(secondPolygon, EPSILON));

         secondPolygon = createConvexPolygon2D(firstPolygon);

         secondPolygon.applyTransform(new RigidBodyTransform(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), 1000.0 * EPSILON), new Vector3D()));

         assertFalse(firstPolygon.geometricallyEquals(secondPolygon, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Convex polygons are equal if translations are equal within +- epsilon and are otherwise the same
         numberOfVertices = 3 + random.nextInt(10);

         firstPolygon = createConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random,
                                                                                                                                           0,
                                                                                                                                           0.5,
                                                                                                                                           numberOfVertices)));
         firstPolygon.scale(10.0);

         secondPolygon = createConvexPolygon2D(firstPolygon);
         translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON);
         secondPolygon.translate(translation);

         assertTrue(firstPolygon.geometricallyEquals(secondPolygon, EPSILON));

         secondPolygon = createConvexPolygon2D(firstPolygon);
         translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON);
         secondPolygon.translate(translation);

         assertFalse(firstPolygon.geometricallyEquals(secondPolygon, EPSILON));
      }
   }

   private static void assertPointsEqual(Point2DReadOnly[] expected, Point2DReadOnly[] actual, boolean enforceOrder)
   {
      if (expected == null || actual == null)
      {
         assertTrue(expected == actual, "Expected did not equal actual. One of them was null.");
         return;
      }

      assertEquals(expected.length, actual.length, "Array lengths are not equal.");
      int points = expected.length;
      for (int i = 0; i < points; i++)
      {
         if (enforceOrder)
         {
            EuclidCoreTestTools.assertTuple2DEquals(expected[i], actual[i], EPSILON);
            continue;
         }

         boolean foundPoint = false;
         for (int j = 0; j < points; j++)
         {
            if (expected[i].epsilonEquals(actual[j], EPSILON))
               foundPoint = true;
         }
         assertTrue(foundPoint, "Did not find point.");
      }
   }

   @Test
   public void testRemoveVertex()
   {
      Random random = new Random(6325);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point2D> vertices = new ArrayList<>();
         T polygon = createRandomConvexPolygon2D(random);
         polygon.getPolygonVerticesView().forEach(vertex -> vertices.add(new Point2D(vertex)));

         int numberOfVerticesToRemove = random.nextInt(polygon.getNumberOfVertices());

         for (int j = 0; j < numberOfVerticesToRemove; j++)
         {
            int indexToRemove = random.nextInt(polygon.getNumberOfVertices());
            polygon.removeVertex(indexToRemove);

            Collections.swap(vertices, indexToRemove, vertices.size() - 1);
            vertices.remove(vertices.size() - 1);
         }

         assertEquals(polygon.getNumberOfVertices(), vertices.size());

         for (int j = 0; j < vertices.size(); j++)
         {
            EuclidCoreTestTools.assertTuple2DEquals(vertices.get(j), polygon.getVertexUnsafe(j), 0.0);
         }
      }
   }
}

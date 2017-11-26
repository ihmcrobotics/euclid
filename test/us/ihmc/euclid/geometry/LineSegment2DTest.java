package us.ihmc.euclid.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class LineSegment2DTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-11;

   @Test
   public void testDistancePoint2dLineSegment2d()
   {
      LineSegment2D line1 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      Point2D point1 = new Point2D(-10.0, 10.0);
      Point2D point2 = new Point2D(0.0, 10.0);
      Point2D point3 = new Point2D(0.0, -10.0);
      Point2D point4 = new Point2D(-10.0, 0.0);
      Point2D point5 = new Point2D(10.0, 0.0);
      Point2D point6 = new Point2D(10.5, 0.0);
      Point2D point7 = new Point2D(0.0, 1.2);
      Point2D point8 = new Point2D(10.1, 0.0);
      Point2D point9 = new Point2D(0.0, 0.0);
      double delta = 0.00001;
      assertEquals(10.0, line1.distance(point1), delta);
      assertEquals(10.0, line1.distance(point2), delta);
      assertEquals(10.0, line1.distance(point3), delta);
      assertEquals(0.0, line1.distance(point4), delta);
      assertEquals(0.0, line1.distance(point5), delta);
      assertEquals(0.5, line1.distance(point6), delta);
      assertEquals(1.2, line1.distance(point7), delta);
      assertEquals(0.1, line1.distance(point8), delta);
      assertEquals(0.0, line1.distance(point9), delta);
   }

   @Test
   public void testLineSegment2dDoubleDoubleDoubleDouble()
   {
      Random random = new Random(234234);

      // WORKING CASES
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x1 = random.nextDouble() * 500 - 250;
         double y1 = random.nextDouble() * 500 - 250;
         double x2 = random.nextDouble() * 500 - 250;
         double y2 = random.nextDouble() * 500 - 250;

         if (x1 != x2 || y1 != y2)
         {
            LineSegment2D test = new LineSegment2D(x1, y1, x2, y2);
            assertEquals(test.getFirstEndpointCopy().getX(), x1, 0.001);
            assertEquals(test.getFirstEndpointCopy().getY(), y1, 0.001);
            assertEquals(test.getSecondEndpointCopy().getX(), x2, 0.001);
            assertEquals(test.getSecondEndpointCopy().getY(), y2, 0.001);
         }
      }
   }

   @Test
   public void testLineSegment2dPoint2dArray()
   {
      Random random = new Random(32423);

      // WORKING CASES
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x1 = random.nextDouble() * 500 - 250;
         double y1 = random.nextDouble() * 500 - 250;
         double x2 = random.nextDouble() * 500 - 250;
         double y2 = random.nextDouble() * 500 - 250;

         if (x1 != x2 || y1 != y2)
         {
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2D test = new LineSegment2D(points);
            assertEquals(test.getFirstEndpointCopy(), points[0]);
            assertEquals(test.getSecondEndpointCopy(), points[1]);
         }
      }
   }

   @Test
   public void testLineSegment2dPoint2dPoint2d()
   {
      Random random = new Random(45089);
      // WORKING CASES
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x1 = random.nextDouble() * 500 - 250;
         double y1 = random.nextDouble() * 500 - 250;
         double x2 = random.nextDouble() * 500 - 250;
         double y2 = random.nextDouble() * 500 - 250;

         if (x1 != x2 || y1 != y2)
         {
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2D test = new LineSegment2D(points[0], points[1]);
            assertEquals(test.getFirstEndpointCopy(), points[0]);
            assertEquals(test.getSecondEndpointCopy(), points[1]);
         }
      }
   }

   @Test
   public void testLineSegment2dLineSegment2d()
   {
      Random random = new Random(97);
      // WORKING CASES
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x1 = random.nextDouble() * 500 - 250;
         double y1 = random.nextDouble() * 500 - 250;
         double x2 = random.nextDouble() * 500 - 250;
         double y2 = random.nextDouble() * 500 - 250;

         if (x1 != x2 || y1 != y2)
         {
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2D test = new LineSegment2D(points[0], points[1]);
            LineSegment2D test2 = new LineSegment2D(test);
            assertEquals(test.getFirstEndpointCopy(), test2.getFirstEndpointCopy());
            assertEquals(test.getSecondEndpointCopy(), test2.getSecondEndpointCopy());
         }
      }

   }

   @Test
   public void testGetEndpointsCopy()
   {
      Random random = new Random(3453);
      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      LineSegment2D testSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();
      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
      assertEquals(pointsCopy[0], segment1Point1);
      assertEquals(pointsCopy[1], segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      assertFalse(pointsCopy[0].equals(testSegment1.getFirstEndpointCopy()));
      assertFalse(pointsCopy[1].equals(testSegment1.getSecondEndpointCopy()));
      assertFalse(pointsCopy[0].equals(segment1Point1));
      assertFalse(pointsCopy[1].equals(segment1Point2));

   }

   @Test
   public void testGetFirstEndPointCopy()
   {
      Random random = new Random(3453);
      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      LineSegment2D testSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

      Point2D pointCopy = testSegment1.getFirstEndpointCopy();
      assertEquals(pointCopy, segment1Point1);

      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.getX() - 10.0, pointCopy.getY() - 10.0);

      assertFalse(pointCopy.equals(segment1Point1));

   }

   @Test
   public void testGetSecondEndPointCopy()
   {
      Random random = new Random(3453);
      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      LineSegment2D testSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

      Point2D pointCopy = testSegment1.getSecondEndpointCopy();
      assertEquals(pointCopy, segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.getX() - 10.0, pointCopy.getY() - 10.0);

      assertFalse(pointCopy.equals(segment1Point2));

   }

   @Test
   public void testSetPoint2dPoint2d()
   {
      Random random = new Random(3453);
      LineSegment2D testSegment1 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);

      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy[0], pointsCopy[1]);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
   }

   @Test
   public void testSetDoubleDoubleDoubleDouble()
   {
      Random random = new Random(3453);
      LineSegment2D testSegment1 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);

      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy[0].getX(), pointsCopy[0].getY(), pointsCopy[1].getX(), pointsCopy[1].getY());

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
   }

   @Test
   public void testSetPoint2dArray()
   {
      Random random = new Random(7653);
      LineSegment2D testSegment1 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
   }

   @Test
   public void testSetLineSegment2d()
   {
      Random random = new Random(3453);
      LineSegment2D testSegment1 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);
      LineSegment2D testSegment2 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);

      testSegment1.set(testSegment2);

      assertEquals(testSegment2.getFirstEndpointCopy(), testSegment1.getFirstEndpointCopy());
      assertEquals(testSegment2.getSecondEndpointCopy(), testSegment1.getSecondEndpointCopy());
   }

   @Test
   public void testFlipDirection()
   {
      Random random = new Random(3453);
      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      LineSegment2D testSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

      testSegment1.flipDirection();
      assertEquals(segment1Point2, testSegment1.getFirstEndpointCopy());
      assertEquals(segment1Point1, testSegment1.getSecondEndpointCopy());

   }

   @Test
   public void testMidpoint()
   {
      Random random = new Random(3453);
      LineSegment2D testSegment1 = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);

      Point2D midPoint = testSegment1.midpoint();
      assertEquals(midPoint.distance(testSegment1.getFirstEndpointCopy()), midPoint.distance(testSegment1.getSecondEndpointCopy()), 0.001);
   }

   @Test
   public void testLength()
   {
      Random random = new Random(3453);
      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      LineSegment2D testSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

      assertEquals(segment1Point1.distance(segment1Point2), testSegment1.length(), 0.001);
      testSegment1.flipDirection();
      assertEquals(segment1Point1.distance(segment1Point2), testSegment1.length(), 0.001);

   }

   @Test
   public void testTranslate() throws Exception
   {
      Random random = new Random(3653);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test translate(double x, double y)
         LineSegment2D originalLineSegment = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);
         LineSegment2D translatedLineSegment = new LineSegment2D(originalLineSegment);
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         translatedLineSegment.translate(x, y);

         assertEquals(originalLineSegment.length(), translatedLineSegment.length(), EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(originalLineSegment.direction(false), translatedLineSegment.direction(false), EPSILON);

         LineSegment2D expectedLineSegment = new LineSegment2D(originalLineSegment);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(x, y, 0.0);
         expectedLineSegment.applyTransform(transform);
         EuclidGeometryTestTools.assertLineSegment2DEquals(expectedLineSegment, translatedLineSegment, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      { // Test translate(Tuple2DReadOnly translation)
         LineSegment2D originalLineSegment = EuclidGeometryRandomTools.nextLineSegment2D(random, 10.0);
         LineSegment2D translatedLineSegment = new LineSegment2D(originalLineSegment);
         Tuple2DReadOnly translation = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         translatedLineSegment.translate(translation);
         
         assertEquals(originalLineSegment.length(), translatedLineSegment.length(), EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(originalLineSegment.direction(false), translatedLineSegment.direction(false), EPSILON);
         
         LineSegment2D expectedLineSegment = new LineSegment2D(originalLineSegment);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(translation.getX(), translation.getY(), 0.0);
         expectedLineSegment.applyTransform(transform);
         EuclidGeometryTestTools.assertLineSegment2DEquals(expectedLineSegment, translatedLineSegment, EPSILON);
      }
   }

   @Test
   public void testPercentageAlongLineSegment()
   {
      LineSegment2D line1 = new LineSegment2D(-10, 0, 10, 0);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2D(0.0, 0.0)), 0.001);
      assertEquals(0.0, line1.percentageAlongLineSegment(new Point2D(-10.0, 0.0)), 0.001);
      assertEquals(1.0, line1.percentageAlongLineSegment(new Point2D(10.0, 0.0)), 0.001);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2D(0.0, 5.0)), 0.001);

      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);

         Point2D pointOnLineSegment = new Point2D();

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);
      }

      // Test off line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);

         Point2D pointOffLineSegment = new Point2D();
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineSegmentDirection);
         orthogonal.normalize();

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testIntersectionWithLineSegment2d1()
   {
      LineSegment2D line1 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      LineSegment2D line2 = new LineSegment2D(-10.0, 10.0, 10.0, 0.0);
      LineSegment2D line3 = new LineSegment2D(0.0, 10.0, 0.0, -10.0);
      LineSegment2D line4 = new LineSegment2D(0.0, -10.0, 0.0, 10.0);
      LineSegment2D line5 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      LineSegment2D line6 = new LineSegment2D(10.0, 0.0, -10.0, 0.0);
      LineSegment2D line7 = new LineSegment2D(10.0, 0.0, 20.0, 0.0);
      LineSegment2D line8 = new LineSegment2D(10.0, 0.0, -20.0, 0.0);
      LineSegment2D line9 = new LineSegment2D(10.1, 0.0, 20.0, 0.0);

      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line1));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line6));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }

   @Test
   public void testIntersectionWithLineSegment2d2()
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfIntersectionWith(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);

         Point2D expectedIntersection = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfIntersectionWith(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if (0.0 < alpha1 && alpha1 < 1.0 || 0.0 < alpha2 && alpha2 < 1.0 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionAllCombinations(true, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }
         else
         {
            assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }
   }

   private void assertOnlyExistenceOfIntersectionAllCombinations(boolean intersectionExist, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                 Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      LineSegment2D lineSegment1 = new LineSegment2D(lineSegmentStart1, lineSegmentEnd1);
      LineSegment2D reverseLineSegment1 = new LineSegment2D(lineSegmentEnd1, lineSegmentStart1);
      LineSegment2D lineSegment2 = new LineSegment2D(lineSegmentStart2, lineSegmentEnd2);
      LineSegment2D reverseLineSegment2 = new LineSegment2D(lineSegmentEnd2, lineSegmentStart2);

      if (intersectionExist)
      {
         assertNotNull(lineSegment1.intersectionWith(lineSegment2));
         assertNotNull(lineSegment1.intersectionWith(reverseLineSegment2));
         assertNotNull(reverseLineSegment1.intersectionWith(lineSegment2));
         assertNotNull(reverseLineSegment1.intersectionWith(reverseLineSegment2));

         assertNotNull(lineSegment2.intersectionWith(lineSegment1));
         assertNotNull(lineSegment2.intersectionWith(reverseLineSegment1));
         assertNotNull(reverseLineSegment2.intersectionWith(lineSegment1));
         assertNotNull(reverseLineSegment2.intersectionWith(reverseLineSegment1));
      }
      else
      {
         assertNull(lineSegment1.intersectionWith(lineSegment2));
         assertNull(lineSegment1.intersectionWith(reverseLineSegment2));
         assertNull(reverseLineSegment1.intersectionWith(lineSegment2));
         assertNull(reverseLineSegment1.intersectionWith(reverseLineSegment2));

         assertNull(lineSegment2.intersectionWith(lineSegment1));
         assertNull(lineSegment2.intersectionWith(reverseLineSegment1));
         assertNull(reverseLineSegment2.intersectionWith(lineSegment1));
         assertNull(reverseLineSegment2.intersectionWith(reverseLineSegment1));
      }
   }

   private void assertAllCombinationsOfIntersectionWith(Point2D expectedIntersection, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                        Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {

      LineSegment2D lineSegment1 = new LineSegment2D(lineSegmentStart1, lineSegmentEnd1);
      LineSegment2D reverseLineSegment1 = new LineSegment2D(lineSegmentEnd1, lineSegmentStart1);
      LineSegment2D lineSegment2 = new LineSegment2D(lineSegmentStart2, lineSegmentEnd2);
      LineSegment2D reverseLineSegment2 = new LineSegment2D(lineSegmentEnd2, lineSegmentStart2);

      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment1.intersectionWith(lineSegment2), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment1.intersectionWith(reverseLineSegment2), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment1.intersectionWith(lineSegment2), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment1.intersectionWith(reverseLineSegment2), EPSILON);

      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment2.intersectionWith(lineSegment1), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment2.intersectionWith(reverseLineSegment1), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment2.intersectionWith(lineSegment1), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment2.intersectionWith(reverseLineSegment1), EPSILON);
   }

   @Test
   public void testIntersectionWithLine2d2()
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);
         LineSegment2D reverseLineSegment = new LineSegment2D(lineSegmentEnd, lineSegmentStart);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Point2D pointOnLine = new Point2D(expectedIntersection);
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         // Expecting intersection
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);
      }

      // Make the intersection happen outside the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);
         LineSegment2D reverseLineSegment = new LineSegment2D(lineSegmentEnd, lineSegmentStart);

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineLineIntersection = new Point2D();
         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -1.0, 0.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         assertNull(actualIntersection);
      }

      // Make the intersection happen on each end point of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);
         LineSegment2D reverseLineSegment = new LineSegment2D(lineSegmentEnd, lineSegmentStart);

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(lineSegmentStart);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(lineSegmentEnd);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);
      }

      // Make the line segment and the line parallel not collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getY());

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), orthogonal, pointOnLine);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         assertNull(actualIntersection);
      }

      // Make the line segment and the line collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);
         LineSegment2D reverseLineSegment = new LineSegment2D(lineSegmentEnd, lineSegmentStart);

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, EPSILON);
         actualIntersection = reverseLineSegment.intersectionWith(new Line2D(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, EPSILON);
      }
   }

   @Test
   public void testDistancePoint2d()
   {
      LineSegment2D line1 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      Point2D point1 = new Point2D(-10.0, 10.0);
      Point2D point2 = new Point2D(0.0, 10.0);
      Point2D point3 = new Point2D(0.0, -10.0);
      Point2D point4 = new Point2D(-10.0, 0.0);
      Point2D point5 = new Point2D(10.0, 0.0);
      Point2D point6 = new Point2D(10.5, 0.0);
      Point2D point7 = new Point2D(0.0, 1.2);
      Point2D point8 = new Point2D(10.1, 0.0);
      Point2D point9 = new Point2D(0.0, 0.0);
      double delta = 0.00001;
      assertEquals(10.0, line1.distance(point1), delta);
      assertEquals(10.0, line1.distance(point2), delta);
      assertEquals(10.0, line1.distance(point3), delta);
      assertEquals(0.0, line1.distance(point4), delta);
      assertEquals(0.0, line1.distance(point5), delta);
      assertEquals(0.5, line1.distance(point6), delta);
      assertEquals(1.2, line1.distance(point7), delta);
      assertEquals(0.1, line1.distance(point8), delta);
      assertEquals(0.0, line1.distance(point9), delta);
   }

   @Test
   public void testShiftToLeftAndRightCopy()
   {
      double distanceToShift = 0.2;

      // Pointing straight up:
      LineSegment2D lineSegment = new LineSegment2D(0.0, 0.0, 0.0, 1.0);
      LineSegment2D shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      Point2D firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      Point2D secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(0.0, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(0.0, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getY(), EPSILON);

      // Pointing straight along x:
      lineSegment = new LineSegment2D(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), EPSILON);

      // Pointing at (1,1)
      lineSegment = new LineSegment2D(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);
      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), EPSILON);
   }

   @Test
   public void testShiftToLeftAndRight()
   {
      double distanceToShift = 0.2;

      // Pointing straight up:
      LineSegment2D lineSegment = new LineSegment2D(0.0, 0.0, 0.0, 1.0);
      LineSegment2D shiftedLineSegment = new LineSegment2D(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      Point2D firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      Point2D secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(0.0, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(0.0, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getY(), EPSILON);

      // Pointing straight along x:
      lineSegment = new LineSegment2D(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), EPSILON);

      // Pointing at (1,1)
      lineSegment = new LineSegment2D(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), EPSILON);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), EPSILON);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), EPSILON);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), EPSILON);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), EPSILON);
   }

   @Test
   public void testIsPointOnLeftRightSide()
   {
      LineSegment2D lineSegment = new LineSegment2D(0.0, 0.0, 0.0, 1.0);

      Point2D point = new Point2D(0.1, 0.5);
      assertFalse(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(-0.1, 0.5);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertFalse(lineSegment.isPointOnRightSideOfLineSegment(point));

      lineSegment = new LineSegment2D(0.0, 2.0, 4.0, 6.0);

      point = new Point2D(0.0, 0.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(4.1, 6.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(3.9, 6.0);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));

   }

   @Test
   public void testOrthogonalProjectionCopyPoint2dLineSegment2d()
   {
      Point2D startPoint = new Point2D(-10.0, 0.0);
      Point2D endPoint = new Point2D(10.0, 0.0);
      LineSegment2D line1 = new LineSegment2D(startPoint, endPoint);

      Point2D origionalPoint = new Point2D(-20.0, 10.0);
      Point2D projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(-20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(-20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(0.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(0.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(5.0, 0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(5.0, 0.0), projectedPoint);

      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         LineSegment2D lineSegment = new LineSegment2D(lineSegmentStart, lineSegmentEnd);
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D expectedProjection = new Point2D();
         Point2D testPoint = new Point2D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         Point2D actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testIntersectionLine2dLineSegment2d()
   {
      LineSegment2D line1 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      Line2D line2 = new Line2D(new Point2D(-10.0, 10.0), new Point2D(10.0, 0.0));
      Line2D line3 = new Line2D(new Point2D(0.0, 10.0), new Point2D(0.0, -10.0));
      Line2D line4 = new Line2D(new Point2D(0.0, -10.0), new Point2D(0.0, 10.0));
      Line2D line5 = new Line2D(new Point2D(-10.0, 0.0), new Point2D(10.0, 0.0));
      Line2D line6 = new Line2D(new Point2D(10.0, 0.0), new Point2D(-10.0, 0.0));
      Line2D line7 = new Line2D(new Point2D(10.0, 0.0), new Point2D(20.0, 0.0));
      Line2D line8 = new Line2D(new Point2D(10.0, 0.0), new Point2D(-20.0, 0.0));
      Line2D line9 = new Line2D(new Point2D(10.1, 0.0), new Point2D(20.0, 0.0));
      Line2D line10 = new Line2D(new Point2D(10.0, 0.0), new Point2D(20.0, 1.0));

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line6), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line10), EPSILON);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4), EPSILON);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line7), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line8), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line9), EPSILON);
   }

   @Test
   public void testIntersectionLineSegment2dLineSegment2d()
   {
      Point2D colTestp1 = new Point2D(1.5, -5);
      Point2D colTestp2 = new Point2D(1.5, 0);
      Point2D colTestp3 = new Point2D(1.5, 5);

      LineSegment2D colTestLine1 = new LineSegment2D(colTestp1, colTestp2);
      LineSegment2D colTestLine2 = new LineSegment2D(colTestp2, colTestp1);

      LineSegment2D colTestLine3 = new LineSegment2D(colTestp1, colTestp3);
      LineSegment2D colTestLine4 = new LineSegment2D(colTestp3, colTestp1);

      LineSegment2D colTestLine5 = new LineSegment2D(colTestp2, colTestp3);
      LineSegment2D colTestLine6 = new LineSegment2D(colTestp3, colTestp2);

      assertEquals(colTestp2, colTestLine1.intersectionWith(colTestLine2));
      assertEquals(colTestp1, colTestLine1.intersectionWith(colTestLine3));
      assertEquals(colTestp1, colTestLine1.intersectionWith(colTestLine4));
      assertEquals(colTestp1, colTestLine2.intersectionWith(colTestLine4));

      assertEquals(colTestp2, colTestLine1.intersectionWith(colTestLine5));
      assertEquals(colTestp2, colTestLine2.intersectionWith(colTestLine6));

      LineSegment2D line1 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      LineSegment2D line2 = new LineSegment2D(-10.0, 10.0, 10.0, 0.0);
      LineSegment2D line3 = new LineSegment2D(0.0, 10.0, 0.0, -10.0);
      LineSegment2D line4 = new LineSegment2D(0.0, -10.0, 0.0, 10.0);
      LineSegment2D line5 = new LineSegment2D(-10.0, 0.0, 10.0, 0.0);
      LineSegment2D line6 = new LineSegment2D(10.0, 0.0, -10.0, 0.0);
      LineSegment2D line7 = new LineSegment2D(10.0, 0.0, 20.0, 0.0);
      LineSegment2D line8 = new LineSegment2D(10.0, 0.0, -20.0, 0.0);
      LineSegment2D line9 = new LineSegment2D(10.1, 0.0, 20.0, 0.0);

      assertEquals(new Point2D(-10.0, 0.0), line5.intersectionWith(line1));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line6));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(19263L);

      Point2D segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random);
      Point2D segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random);
      Point2D segment2Point1 = new Point2D(segment1Point1);
      Point2D segment2Point2 = new Point2D(segment1Point2);

      LineSegment2D lineSegment1 = new LineSegment2D(segment1Point1, segment1Point2);
      LineSegment2D lineSegment2 = new LineSegment2D(segment2Point1, segment2Point2);

      assertTrue(lineSegment1.geometricallyEquals(lineSegment1, EPSILON));
      assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
      assertTrue(lineSegment2.geometricallyEquals(lineSegment1, EPSILON));
      assertTrue(lineSegment2.geometricallyEquals(lineSegment2, EPSILON));

      for (int i = 0; i < ITERATIONS; ++i)
      {
         segment1Point1 = EuclidCoreRandomTools.nextPoint2D(random);
         segment1Point2 = EuclidCoreRandomTools.nextPoint2D(random);
         segment2Point1 = new Point2D();
         segment2Point2 = new Point2D();

         lineSegment1 = new LineSegment2D(segment1Point1, segment1Point2);

         segment2Point1.set(segment1Point1);
         segment2Point2.add(segment1Point2, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point1, segment2Point2);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point2, segment2Point1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.set(segment1Point1);
         segment2Point2.add(segment1Point2, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point1, segment2Point2);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point2, segment2Point1);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.add(segment1Point1, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON));
         segment2Point2.set(segment1Point2);
         lineSegment2 = new LineSegment2D(segment2Point1, segment2Point2);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point2, segment2Point1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.add(segment1Point1, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON));
         segment2Point2.set(segment1Point2);
         lineSegment2 = new LineSegment2D(segment2Point1, segment2Point2);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment2D(segment2Point2, segment2Point1);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         lineSegment2.set(lineSegment1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.translate(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON));
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.flipDirection();
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         lineSegment2.set(lineSegment1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.translate(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON));
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.flipDirection();
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
      }
   }
}

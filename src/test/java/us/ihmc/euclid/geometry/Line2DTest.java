package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Line2DTest
{
   private static final double maxRandomValue = 1.0e5;

   private double randomDouble(Random random, double maxRandomValue)
   {
      return random.nextDouble() * maxRandomValue * 2.0 - maxRandomValue;
   }

   private double randomDouble(Random random)
   {
      return randomDouble(random, maxRandomValue);
   }

   private Point2D randomPoint(Random random)
   {
      return new Point2D(randomDouble(random, 1.0e5), randomDouble(random, 1.0e5));
   }

   @Test
   public void testConstructors()
   {
      double delta = 1.0e-5;

      Random random = new Random(1000L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Vector2D vector = new Vector2D(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY());
         Line2D line2dByPointVector = new Line2D(firstPointOnLine, vector);
         Line2D line2dByPointPoint = new Line2D(firstPointOnLine, secondPointOnLine);

         assertEquals(line2dByPointPoint.getPoint(), line2dByPointVector.getPoint());
         assertEquals(line2dByPointPoint.getDirection().getX(), line2dByPointVector.getDirection().getX(), delta);
         assertEquals(line2dByPointPoint.getDirection().getY(), line2dByPointVector.getDirection().getY(), delta);

         Line2D line2dByCopy = new Line2D(line2dByPointVector);
         assertFalse(line2dByPointVector == line2dByCopy);
         assertEquals(line2dByPointVector.getPoint(), line2dByCopy.getPoint());
         assertEquals(line2dByPointVector.getDirection().getX(), line2dByCopy.getDirection().getX(), delta);
         assertEquals(line2dByPointVector.getDirection().getY(), line2dByCopy.getDirection().getY(), delta);

      }

   }

   @Test
   public void testIsPointInFrontOfLine2d()
   {
      Line2D line2d = new Line2D();
      Point2D point1 = new Point2D();
      Point2D point2 = new Point2D();
      Point2D point3 = new Point2D();
      Vector2D frontDirection = new Vector2D();

      point1.set(0.0, 0.0);
      point2.set(1.0, 1.0);
      line2d.set(point1, point2);
      frontDirection.set(0.0, 1.0);

      point3.set(0.0, 1.0);
      assertEquals(true, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");

      point3.set(0.0, -1.0);
      assertEquals(false, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");

      point1.set(0.0, 0.0);
      point2.set(-1.0, 1.0);
      line2d.set(point1, point2);

      point3.set(0.0, 1.0);
      assertEquals(true, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");

      point3.set(0.0, -1.0);
      assertEquals(false, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");

      frontDirection.set(0.0, -1.0);

      point3.set(0.0, 1.0);
      assertEquals(false, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");

      point3.set(0.0, -1.0);
      assertEquals(true, line2d.isPointInFrontOfLine(frontDirection, point3), "not equal");
   }

   @Test
   public void testPointPointConstructorForException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         // TODO: Test this at various random points, or is this sufficient?
         Point2D firstPointOnLine = new Point2D(0.0, 0.0);
         new Line2D(firstPointOnLine, firstPointOnLine);
      });
   }

   @Test
   public void testGetPoint()
   {
      // TODO: Test this at various random points, or is this sufficient?
      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
      assertEquals(firstPointOnLine, line2d.getPoint());
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(32423L);

      Line2D line2D = EuclidGeometryRandomTools.nextLine2D(random);
      line2D.setToZero();
      try
      {
         EuclidCoreTestTools.assertTuple2DIsSetToZero(line2D.getPoint());
      }
      catch (RuntimeException e)
      {
         // Good
      }
      try
      {
         EuclidCoreTestTools.assertTuple2DIsSetToZero(line2D.getDirection());
      }
      catch (RuntimeException e)
      {
         // Good
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(32423L);

      Line2D line2D = EuclidGeometryRandomTools.nextLine2D(random);
      line2D.setToNaN();
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(line2D.getPoint());
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(line2D.getDirection());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Line2D line2D = new Line2D();
      assertFalse(line2D.containsNaN());
      line2D.set(0.0, 0.0, 0.0, 1.0);
      assertFalse(line2D.containsNaN());
      line2D.set(Double.NaN, 0.0, 0.0, 1.0);
      assertTrue(line2D.containsNaN());
      line2D.set(0.0, Double.NaN, 0.0, 1.0);
      assertTrue(line2D.containsNaN());
      line2D.set(0.0, 0.0, Double.NaN, 1.0);
      assertTrue(line2D.containsNaN());
      line2D.set(0.0, 0.0, 1.0, Double.NaN);
      assertTrue(line2D.containsNaN());
   }

   @Test
   public void testGetNormalizedVector()
   {
      double delta = 1.0e-5;

      Random random = new Random(789L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         double xdiff = secondPointOnLine.getX() - firstPointOnLine.getX();
         double ydiff = secondPointOnLine.getY() - firstPointOnLine.getY();
         double length = EuclidCoreTools.norm(xdiff, ydiff);
         assertEquals(xdiff / length, line2d.getDirection().getX(), delta);
         assertEquals(ydiff / length, line2d.getDirection().getY(), delta);
      }
   }

   @Test
   public void testGetNormalizedVectorCopy()
   {
      double delta = 1.0e-5;

      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      Vector2D normalizedVector = new Vector2D(Math.sqrt(2.0) / 2.0, Math.sqrt(2.0) / 2.0);
      assertEquals(normalizedVector.getX(), line2d.getDirection().getX(), delta);
      assertEquals(normalizedVector.getY(), line2d.getDirection().getY(), delta);
      Vector2D normalizedVector2 = new Vector2D(line2d.getDirection());
      assertFalse(line2d.getDirection() == normalizedVector2);
   }

   @Test
   public void testGetSlope()
   {
      Random random = new Random(2048L);
      double delta = 1.0e-5;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         double slope = (secondPointOnLine.getY() - firstPointOnLine.getY()) / (secondPointOnLine.getX() - firstPointOnLine.getX());
         assertEquals(slope, line2d.slope(), delta);
      }

      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(0.0, 5.0);
      Line2D verticalLine = new Line2D(firstPointOnLine, secondPointOnLine);
      assertEquals(Double.POSITIVE_INFINITY, verticalLine.slope(), delta);

      secondPointOnLine = new Point2D(0.0, -5.0);
      Line2D horizontalLine = new Line2D(firstPointOnLine, secondPointOnLine);
      assertEquals(Double.NEGATIVE_INFINITY, horizontalLine.slope(), delta);
   }

   @Test
   public void testGetXIntercept()
   {
      double delta = 1.0e-5;
      Random random = new Random(1886L);
      for (int i = 0; i < 1000; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         double slope = (secondPointOnLine.getY() - firstPointOnLine.getY()) / (secondPointOnLine.getX() - firstPointOnLine.getX());
         double additive = firstPointOnLine.getY() - slope * firstPointOnLine.getX();
         assertEquals(-additive / slope, line2d.xIntercept(), delta);
      }

      // Edge cases: on top of x-axis and parallel to x-axis
      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(5.0, 0.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
      assertEquals(Double.NaN, line2d.xIntercept(), delta);

      firstPointOnLine = new Point2D(1.0, 1.0);
      secondPointOnLine = new Point2D(2.0, 1.0);
      line2d.set(firstPointOnLine, secondPointOnLine);
      assertEquals(Double.NEGATIVE_INFINITY, line2d.xIntercept(), delta);

      line2d.set(secondPointOnLine, firstPointOnLine);
      assertEquals(Double.POSITIVE_INFINITY, line2d.xIntercept(), delta);
   }

   @Test
   public void testGetYIntercept()
   {
      double delta = 1.0e-5;
      Random random = new Random(1972L);
      for (int i = 0; i < 1000; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         double slope = (secondPointOnLine.getY() - firstPointOnLine.getY()) / (secondPointOnLine.getX() - firstPointOnLine.getX());
         double additive = firstPointOnLine.getY() - slope * firstPointOnLine.getX();
         assertEquals(additive, line2d.yIntercept(), delta);
      }

      // Edge cases: on top of x-axis and parallel to x-axis
      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(0.0, 5.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      assertEquals(Double.NaN, line2d.yIntercept(), delta);

      firstPointOnLine = new Point2D(1.0, 1.0);
      secondPointOnLine = new Point2D(1.0, 2.0);
      line2d.set(firstPointOnLine, secondPointOnLine);
      assertEquals(Double.NEGATIVE_INFINITY, line2d.yIntercept(), delta);

      line2d.set(secondPointOnLine, firstPointOnLine);
      assertEquals(Double.POSITIVE_INFINITY, line2d.yIntercept(), delta);
   }

   @Test
   public void testNegateDirection()
   {
      double delta = 1.0e-5;

      Random random = new Random(2036L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2D normalizedVectorCopy = new Vector2D(line2d.getDirection());
         line2d.negateDirection();
         assertEquals(-normalizedVectorCopy.getX(), line2d.getDirection().getX(), delta);
         assertEquals(-normalizedVectorCopy.getY(), line2d.getDirection().getY(), delta);
      }
   }

   @Test
   public void testSetPointPoint()
   {
      double delta = 1.0e-5;

      Random random = new Random(9999L);
      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
      for (int i = 0; i < ITERATIONS; i++)
      {
         firstPointOnLine = randomPoint(random);
         secondPointOnLine = randomPoint(random);
         line2d.set(firstPointOnLine, secondPointOnLine);
         assertFalse(firstPointOnLine == line2d.getPoint());
         assertEquals(firstPointOnLine, line2d.getPoint());
         double xdiff = secondPointOnLine.getX() - firstPointOnLine.getX();
         double ydiff = secondPointOnLine.getY() - firstPointOnLine.getY();
         double length = EuclidCoreTools.norm(xdiff, ydiff);

         assertEquals(xdiff / length, line2d.getDirection().getX(), delta);
         assertEquals(ydiff / length, line2d.getDirection().getY(), delta);

      }
   }

   @Test
   public void testSetPointPointException()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
         Point2D firstPointOnLine = new Point2D(0.0, 0.0);
         Point2D secondPointOnLine = new Point2D(1.0, 1.0);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         line2d.set(firstPointOnLine, firstPointOnLine);
      });
   }

   @Test
   public void testSetLine()
   {
      // TODO: Is it necessary to do random tests here?

      double delta = 1.0e-5;

      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      Point2D newFirstPointOnLine = new Point2D(10.0, 15.0);
      Point2D newSecondPointOnLine = new Point2D(15.0, 8.0);
      Line2D secondLine2d = new Line2D(newFirstPointOnLine, newSecondPointOnLine);

      line2d.set(secondLine2d);
      assertFalse(secondLine2d == line2d);
      assertEquals(secondLine2d.getPoint().getX(), line2d.getPoint().getX(), delta);
      assertEquals(secondLine2d.getPoint().getY(), line2d.getPoint().getY(), delta);
      assertFalse(secondLine2d.getPoint() == line2d.getPoint());

      assertEquals(secondLine2d.getDirection().getX(), line2d.getDirection().getX(), delta);
      assertEquals(secondLine2d.getDirection().getY(), line2d.getDirection().getY(), delta);
      assertFalse(secondLine2d.getDirection() == line2d.getDirection());
   }

   @Test
   public void testSetPoint2d()
   {
      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
      Point2D newPoint = new Point2D(11.0, 9.0);
      line2d.setPoint(newPoint);

      assertEquals(newPoint, line2d.getPoint());
      assertFalse(newPoint == line2d.getPoint());
   }

   @Test
   public void testRotate()
   {
      double delta = 1.0e-5;
      Random random = new Random(777L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = new Point2D(0.0, 0.0);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         double angle = Math.atan2(line2d.getDirection().getY(), line2d.getDirection().getX());
         double rotation = randomDouble(random, 2.0 * Math.PI);
         double newAngle = angle + rotation;
         line2d.rotate(rotation);

         assertEquals(Math.cos(newAngle), line2d.getDirection().getX(), delta);
         assertEquals(Math.sin(newAngle), line2d.getDirection().getY(), delta);
      }
   }

   @Test
   public void testShiftToLeftAndRight()
   {
      double distanceToShift = 0.2;
      double epsilon = 1e-7;

      // Pointing straight up:
      Line2D line = new Line2D(0.0, 0.0, 0.0, 1.0);
      Line2D shiftedLine = new Line2D(line);
      shiftedLine.shiftToRight(distanceToShift);

      Point2DReadOnly shiftedLineOrigin = shiftedLine.getPoint();
      Vector2DReadOnly lineVector = line.getDirection();
      Vector2DReadOnly shiftedLineVector = shiftedLine.getDirection();

      assertEquals(distanceToShift, shiftedLineOrigin.getX(), epsilon);
      assertEquals(0.0, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);

      shiftedLine.set(line);
      shiftedLine.shiftToLeft(distanceToShift);

      shiftedLineOrigin = shiftedLine.getPoint();
      lineVector = line.getDirection();
      shiftedLineVector = shiftedLine.getDirection();

      assertEquals(-distanceToShift, shiftedLineOrigin.getX(), epsilon);
      assertEquals(0.0, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);

      // Pointing straight along x:
      line = new Line2D(0.0, 0.0, 1.0, 0.0);
      shiftedLine.set(line);
      shiftedLine.shiftToRight(distanceToShift);

      shiftedLineOrigin = shiftedLine.getPoint();
      lineVector = line.getDirection();
      shiftedLineVector = shiftedLine.getDirection();

      assertEquals(0.0, shiftedLineOrigin.getX(), epsilon);
      assertEquals(-distanceToShift, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);

      shiftedLine.set(line);
      shiftedLine.shiftToLeft(distanceToShift);

      shiftedLineOrigin = shiftedLine.getPoint();
      lineVector = line.getDirection();
      shiftedLineVector = shiftedLine.getDirection();

      assertEquals(0.0, shiftedLineOrigin.getX(), epsilon);
      assertEquals(distanceToShift, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);

      // Pointing at (1,1)
      line = new Line2D(0.0, 0.0, 1.0, 1.0);
      shiftedLine.set(line);
      shiftedLine.shiftToRight(distanceToShift);

      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      shiftedLineOrigin = shiftedLine.getPoint();
      lineVector = line.getDirection();
      shiftedLineVector = shiftedLine.getDirection();

      assertEquals(distanceAtFortyFiveDegrees, shiftedLineOrigin.getX(), epsilon);
      assertEquals(-distanceAtFortyFiveDegrees, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);

      shiftedLine.set(line);
      shiftedLine.shiftToLeft(distanceToShift);

      shiftedLineOrigin = shiftedLine.getPoint();
      lineVector = line.getDirection();
      shiftedLineVector = shiftedLine.getDirection();

      assertEquals(-distanceAtFortyFiveDegrees, shiftedLineOrigin.getX(), epsilon);
      assertEquals(distanceAtFortyFiveDegrees, shiftedLineOrigin.getY(), epsilon);
      assertEquals(lineVector.getX(), shiftedLineVector.getX(), epsilon);
      assertEquals(lineVector.getY(), shiftedLineVector.getY(), epsilon);
   }

   @Test
   public void testInteriorBisector()
   {
      Random random = new Random(1982);
      double delta = 1.0e-5;

      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 0.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      Line2D secondLine2d = new Line2D(line2d);

      Line2DBasics interiorBisector = line2d.interiorBisector(secondLine2d);

      assertEquals(line2d.getPoint(), interiorBisector.getPoint());
      assertEquals(line2d.getDirection().getX(), interiorBisector.getDirection().getX(), delta);
      assertEquals(line2d.getDirection().getY(), interiorBisector.getDirection().getY(), delta);

      Line2D parallelLine2d = new Line2D(line2d);
      parallelLine2d.setPoint(new Point2D(5.5, 18));
      assertNull(line2d.interiorBisector(parallelLine2d));

      for (int i = 0; i < ITERATIONS; i++)
      {
         line2d.set(randomDouble(random, 10.0), randomDouble(random, 10.0), randomDouble(random, 10.0), randomDouble(random, 10.0));
         secondLine2d.set(randomDouble(random, 10.0), randomDouble(random, 10.0), randomDouble(random, 10.0), randomDouble(random, 10.0));
         interiorBisector = line2d.interiorBisector(secondLine2d);

         double tangent1 = line2d.slope();
         double additive1 = line2d.getPoint().getY() - tangent1 * line2d.getPoint().getX();
         double tangent2 = secondLine2d.slope();
         double additive2 = secondLine2d.getPoint().getY() - tangent2 * secondLine2d.getPoint().getX();

         double intersectX = (additive2 - additive1) / (tangent1 - tangent2);
         double intersectY = tangent1 * intersectX + additive1;

         assertEquals(intersectX, interiorBisector.getPoint().getX(), delta);
         assertEquals(intersectY, interiorBisector.getPoint().getY(), delta);

         Vector2D interiorNormalizedVector = new Vector2D(line2d.getDirection());
         Vector2D vector = new Vector2D(secondLine2d.getDirection());
         interiorNormalizedVector.add(vector);
         double length = interiorNormalizedVector.length();
         assertEquals(interiorNormalizedVector.getX() / length, interiorBisector.getDirection().getX(), delta);
         assertEquals(interiorNormalizedVector.getY() / length, interiorBisector.getDirection().getY(), delta);
      }
   }

   @Test
   public void testPerpendicularVector()
   {
      double delta = 1.0e-5;

      Random random = new Random(1984L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2DBasics perpendicular = line2d.perpendicularVector();

         assertEquals(0.0, perpendicular.getX() * line2d.getDirection().getX() + perpendicular.getY() * line2d.getDirection().getY(), delta);
      }
   }

   @Test
   public void testPerpendicularLineThroughPoint()
   {
      double delta = 1.0e-5;
      Random random = new Random(8888L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Point2D pointOnPerpendicularLine = randomPoint(random);
         Line2DBasics perpendicularLine = line2d.perpendicularLineThroughPoint(pointOnPerpendicularLine);

         assertTrue(perpendicularLine.isPointOnLine(pointOnPerpendicularLine, delta));
         assertEquals(0.0, perpendicularLine.getDirection().getX() * line2d.getDirection().getX()
         + perpendicularLine.getDirection().getY() * line2d.getDirection().getY(), delta);
      }
   }

   @Test
   public void testOrthogonalProjection()
   {
      double delta = 1.0e-5;
      Random random = new Random(2000L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2DReadOnly normalizedVector = line2d.getDirection();
         Vector2DBasics perpendicular = line2d.perpendicularVector();

         Point2D pointOnLine = new Point2D(firstPointOnLine);
         double distance = randomDouble(random, 10.0);
         double perpendicularDistance = randomDouble(random, 10.0);
         pointOnLine.setX(pointOnLine.getX() + distance * normalizedVector.getX());
         pointOnLine.setY(pointOnLine.getY() + distance * normalizedVector.getY());
         Point2D pointOffLine = new Point2D(pointOnLine);
         pointOffLine.setX(pointOffLine.getX() + perpendicularDistance * perpendicular.getX());
         pointOffLine.setY(pointOffLine.getY() + perpendicularDistance * perpendicular.getY());

         line2d.orthogonalProjection(pointOffLine);

         assertEquals(pointOnLine.getX(), pointOffLine.getX(), delta);
         assertEquals(pointOnLine.getY(), pointOffLine.getY(), delta);
      }
   }

   @Test
   public void testOrthogonalProjectionCopy()
   {
      double delta = 1.0e-5;
      Random random = new Random(1111L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2DReadOnly normalizedVector = line2d.getDirection();
         Vector2DBasics perpendicular = line2d.perpendicularVector();

         Point2D pointOnLine = new Point2D(firstPointOnLine);
         double distance = randomDouble(random, 10.0);
         double perpendicularDistance = randomDouble(random, 10.0);
         pointOnLine.setX(pointOnLine.getX() + distance * normalizedVector.getX());
         pointOnLine.setY(pointOnLine.getY() + distance * normalizedVector.getY());
         Point2D pointOffLine = new Point2D(pointOnLine);
         pointOffLine.setX(pointOffLine.getX() + perpendicularDistance * perpendicular.getX());
         pointOffLine.setY(pointOffLine.getY() + perpendicularDistance * perpendicular.getY());

         Point2DBasics orthogonalCopy = line2d.orthogonalProjectionCopy(pointOffLine);

         assertEquals(pointOnLine.getX(), orthogonalCopy.getX(), delta);
         assertEquals(pointOnLine.getY(), orthogonalCopy.getY(), delta);
         assertNotSame(pointOnLine, orthogonalCopy);
         assertNotSame(pointOffLine, orthogonalCopy);
      }
   }

   @Test
   public void testIntersectionWithLineSegment2d()
   {
      Random random = new Random(3333L);
      double delta = 1.0e-5;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPoint = randomPoint(random);
         Point2D secondPoint = randomPoint(random);
         LineSegment2D lineSegment2d = new LineSegment2D(firstPoint, secondPoint);

         Line2D colinearLine2d = new Line2D(firstPoint, secondPoint);

         // TODO: Sometimes fails.
         //       assertNull(colinearLine2d.intersectionWith(lineSegment2d));

         Line2D parallelLine2d = new Line2D(colinearLine2d);
         double distance = randomDouble(random, 10.0);
         parallelLine2d.shiftToLeft(distance);
         assertNull(parallelLine2d.intersectionWith(lineSegment2d));

         Vector2D direction = new Vector2D(randomDouble(random, 10.0), randomDouble(random, 10.0));
         Line2D lineThroughEndPoint = new Line2D(firstPoint, direction);
         Point2DBasics intersection = lineThroughEndPoint.intersectionWith(lineSegment2d);
         assertEquals(firstPoint.getX(), intersection.getX(), delta);
         assertEquals(firstPoint.getY(), intersection.getY(), delta);
         lineThroughEndPoint.setPoint(secondPoint);
         intersection = lineThroughEndPoint.intersectionWith(lineSegment2d);

         // TODO intersection is null, which is unexpected.
         //       assertEquals(secondPoint.x, intersection.x, delta);
         //       assertEquals(secondPoint.y, intersection.y, delta);

         Point2D midPoint = new Point2D();
         midPoint.add(firstPoint, secondPoint);
         midPoint.scale(0.5);

         Line2D intersectingLine = new Line2D(midPoint, direction);
         intersection = intersectingLine.intersectionWith(lineSegment2d);
         assertEquals(midPoint.getX(), intersection.getX(), delta);
         assertEquals(midPoint.getY(), intersection.getY(), delta);
      }
   }

   @Test
   public void testIntersectionWithLine2d()
   {
      double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         Line2D line1 = new Line2D(pointOnLine1, lineDirection1);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, pointOnLine1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         Point2DBasics actualIntersection = line1.intersectionWith(new Line2D(pointOnLine2, lineDirection2));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = line1.intersectionWith(new Line2D(pointOnLine2, lineDirection2));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Test when parallel but not collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         Line2D line1 = new Line2D(pointOnLine1, lineDirection1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         if (random.nextBoolean())
            lineDirection2.negate();
         Point2D pointOnLine2 = new Point2D(pointOnLine1);

         Vector2D orthogonal = new Vector2D(-lineDirection1.getY(), lineDirection1.getX());

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOnLine2);
         pointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         Point2DBasics actualIntersection = line1.intersectionWith(new Line2D(pointOnLine2, lineDirection2));
         assertNull(actualIntersection);
      }

      // Test when collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         Line2D line1 = new Line2D(pointOnLine1, lineDirection1);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(pointOnLine1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         Point2DBasics actualIntersection = line1.intersectionWith(new Line2D(pointOnLine2, lineDirection2));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = line1.intersectionWith(new Line2D(pointOnLine2, lineDirection2));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }
   }

   @Test
   public void testDistancePoint2d()
   {
      Random random = new Random(743L);
      double delta = 1.0e-3;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Point2D distantPoint = randomPoint(random);

         double calculatedDistance = line2d.distance(distantPoint);

         Line2DBasics orthogonalLine = line2d.perpendicularLineThroughPoint(firstPointOnLine);
         Point2DBasics orthogonalProjection = orthogonalLine.orthogonalProjectionCopy(distantPoint);
         double xdiff = orthogonalProjection.getX() - firstPointOnLine.getX();
         double ydiff = orthogonalProjection.getY() - firstPointOnLine.getY();
         double distance = Math.sqrt(xdiff * xdiff + ydiff * ydiff);

         assertEquals(distance, calculatedDistance, delta);
      }
   }

   @Test
   public void testDistanceLine2d()
   {
      Random random = new Random(23L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 10.0);
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         Line2D line = new Line2D(pointOnLine, lineDirection);

         Point2D randomPointOnLine = new Point2D();
         randomPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnLine);

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getX());
         orthogonal.normalize();
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point2D point = new Point2D();
         point.scaleAdd(expectedDistance, orthogonal, randomPointOnLine);

         double actualDistance = line.distance(point);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void areLinesPerpendicularTest()
   {
      Point2D point1 = new Point2D(1.0, 1.0);
      Point2D point2 = new Point2D(5.0, 5.0);

      Point2D point3 = new Point2D(2.2, 3.3);
      Point2D point4 = new Point2D(10, 10);

      Point2D point5 = new Point2D(-2, 4);
      Point2D point5UnProjected = new Point2D(point5);

      Line2D line1 = new Line2D(point1, point2);
      Line2D line2 = new Line2D(point3, point4);

      line1.orthogonalProjection(point5); //project point5 onto line1, this ensures we have a point for creating a perpendicular line

      Line2D line3 = new Line2D(point5, point5UnProjected);

      assertFalse(line1.areLinesPerpendicular(line2));
      assertTrue(line1.areLinesPerpendicular(line3));
   }

   @Test
   public void testIsPointOnLeftSideOfLine()
   {
      Random random = new Random(8989L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2D normalizedVector = new Vector2D(line2d.getDirection());
         Vector2DBasics perpendicularVector = line2d.perpendicularVector();
         perpendicularVector.negate();
         Point2D checkPoint = new Point2D();

         double longitude = randomDouble(random);
         double latitude = (randomDouble(random) + maxRandomValue) / 2.0 + 1.0; // Makes sure the point ends up on the right side.
         checkPoint.scaleAdd(longitude, normalizedVector, firstPointOnLine);
         checkPoint.scaleAdd(latitude, perpendicularVector, checkPoint);

         assertFalse(line2d.isPointOnLeftSideOfLine(checkPoint));

         // TODO: Do we need a check for a point on the line? Floating point errors might put this at the wrong side.
         //       checkPoint.scaleAdd(longitude, normalizedVector, firstPointOnLine);
         //       assertFalse(line2d.isPointOnLeftSideOfLine(checkPoint));

         checkPoint.scaleAdd(-2.0 * latitude, perpendicularVector, checkPoint);
         assertTrue(line2d.isPointOnLeftSideOfLine(checkPoint));
      }
   }

   @Test
   public void testIsPointOnRightSideOfLine()
   {
      // TODO: check strictness on this method, currently inconsistent with isPointOnLeftSideOfLine
      Random random = new Random(9999L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2D normalizedVector = new Vector2D(line2d.getDirection());
         Vector2DBasics perpendicularVector = line2d.perpendicularVector();
         perpendicularVector.negate();
         Point2D checkPoint = new Point2D();

         double longitude = randomDouble(random);
         double latitude = (randomDouble(random) + maxRandomValue) / 2.0 + 1.0; // Makes sure the point ends up on the right side.
         checkPoint.scaleAdd(longitude, normalizedVector, firstPointOnLine);
         checkPoint.scaleAdd(latitude, perpendicularVector, checkPoint);

         assertTrue(line2d.isPointOnRightSideOfLine(checkPoint));

         // TODO: Do we need a check for a point on the line? Floating point errors might put this at the wrong side.
         //       checkPoint.scaleAdd(longitude, normalizedVector, firstPointOnLine);
         //       assertTrue(line2d.isPointOnRightSideOfLine(checkPoint));

         checkPoint.scaleAdd(-2.0 * latitude, perpendicularVector, checkPoint);
         assertFalse(line2d.isPointOnRightSideOfLine(checkPoint));
      }
   }

   @Test
   public void testSideConsistency()
   {
      Random random = new Random(1234L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Vector2D normalizedVector = new Vector2D(line2d.getDirection());
         Vector2DBasics perpendicularVector = line2d.perpendicularVector();
         Point2D checkPoint = new Point2D();

         double longitude = randomDouble(random);
         double latitude = randomDouble(random);
         checkPoint.scaleAdd(longitude, normalizedVector, firstPointOnLine);
         checkPoint.scaleAdd(latitude, perpendicularVector, checkPoint);

         assertFalse(line2d.isPointOnRightSideOfLine(checkPoint) == line2d.isPointOnLeftSideOfLine(checkPoint));
      }
   }

   @Test
   public void testIsPointInFrontOfLine()
   {
      Random random = new Random(7777L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Point2D checkPoint = new Point2D(secondPointOnLine);
         double shift = (randomDouble(random) + maxRandomValue) / 2.0 + 1.0; // Makes sure the shift is strictly positive.
         checkPoint.setX(checkPoint.getX() + shift);
         assertTrue(line2d.isPointInFrontOfLine(checkPoint));
         checkPoint.setX(checkPoint.getX() - 2.0 * shift);
         assertFalse(line2d.isPointInFrontOfLine(checkPoint));

         // TODO: is a test necessary where the point is exactly on the line? These kind of tests might fail due to floating point errors.
      }
   }

   @Test
   public void testIsPointInFrontOfLineException()
   {
      Random random = new Random(1776L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Vector2D direction = new Vector2D(1.0, 0.0);
         Line2D line2d = new Line2D(firstPointOnLine, direction);
         Point2D checkPoint = randomPoint(random);
         try
         {
            line2d.isPointInFrontOfLine(checkPoint);
            fail("Failed to throw exception.");
         }
         catch (RuntimeException exception)
         {
         }
      }
   }

   @Test
   public void testIsPointBehindLine()
   {
      Random random = new Random(1776L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Point2D checkPoint = new Point2D(secondPointOnLine);
         double shift = (randomDouble(random) - maxRandomValue) / 2.0 - 1.0; // Makes sure the shift is strictly negative.
         checkPoint.setX(checkPoint.getX() + shift);
         assertTrue(line2d.isPointBehindLine(checkPoint));
         checkPoint.setX(checkPoint.getX() - 2.0 * shift);
         assertFalse(line2d.isPointBehindLine(checkPoint));

         // TODO: is a test necessary where the point is exactly on the line? These kind of tests might fail due to floating point errors.
      }
   }

   @Test
   public void testIsPointBehindLineException()
   {
      Random random = new Random(1776L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Vector2D direction = new Vector2D(1.0, 0.0);
         Line2D line2d = new Line2D(firstPointOnLine, direction);
         Point2D checkPoint = randomPoint(random);
         try
         {
            line2d.isPointBehindLine(checkPoint);
            fail("Failed to throw exception.");
         }
         catch (RuntimeException exception)
         {
         }
      }
   }

   @Test
   public void testFrontBehindConsistency()
   {
      Random random = new Random(1776L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);
         Point2D checkPoint = new Point2D(secondPointOnLine);
         double shift = randomDouble(random);
         checkPoint.setX(checkPoint.getX() + shift);
         assertFalse(line2d.isPointBehindLine(checkPoint) == line2d.isPointInFrontOfLine(checkPoint));
      }
   }

   @Test
   public void testGetParameterGivenPointEpsilon()
   {
      Random random = new Random(1776L);
      double epsilon = 1e-5;
      double delta = 1.0e-5;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firsPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firsPointOnLine, secondPointOnLine);
         double parameter = randomDouble(random);
         Point2D checkPoint = new Point2D(firsPointOnLine);
         Vector2D normalizedVector = new Vector2D(line2d.getDirection());
         checkPoint.scaleAdd(parameter, normalizedVector, firsPointOnLine);

         double calculatedParameter = line2d.parameterGivenPointOnLine(checkPoint, epsilon);

         assertEquals(parameter, calculatedParameter, delta);
      }
   }

   @Test
   public void testGetParameterGivenPointEpsilonException()
   {
      Random random = new Random(1776L);
      double epsilon = 1e-5;
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firsPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firsPointOnLine, secondPointOnLine);
         double parameter = randomDouble(random);
         double perpendicularDistance = randomDouble(random);
         perpendicularDistance = perpendicularDistance + Math.signum(perpendicularDistance); // Ensures that the point cannot be on the line
         Point2D checkPoint = new Point2D(firsPointOnLine);
         Vector2D normalizedVector = new Vector2D(line2d.getDirection());
         checkPoint.scaleAdd(parameter, normalizedVector, firsPointOnLine);
         checkPoint.scaleAdd(perpendicularDistance, line2d.perpendicularVector(), checkPoint);

         try
         {
            line2d.parameterGivenPointOnLine(checkPoint, epsilon);
            fail("Failed to throw an exception");
         }
         catch (RuntimeException exception)
         {
         }
      }
   }

   @Test
   public void testIntersectionWithConvexPolygon()
   {
      // TODO: Failing test case ignored
      Random random = new Random(1776L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);
         Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

         ArrayList<Point2D> pointList = new ArrayList<>();
         for (int j = 0; j < 25; j++)
         {
            pointList.add(randomPoint(random));
         }

         ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));
         Point2DBasics[] intersectionList = line2d.intersectionWith(convexPolygon);

         assertTrue(intersectionList == null || intersectionList.length % 2 == 0);
         assertTrue(intersectionList == null || intersectionList.length <= 2);
      }

      Point2D firstPointOnLine = new Point2D(0.0, 0.0);
      Point2D secondPointOnLine = new Point2D(1.0, 1.0);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      Point2D firstPolygonPoint = new Point2D(0.0, 0.0);
      Point2D secondPolygonPoint = new Point2D(0.0, 1.0);
      Point2D thirdPolygonPoint = new Point2D(-1.0, 0.0);
      ArrayList<Point2D> polygonPoints = new ArrayList<>();
      polygonPoints.add(firstPolygonPoint);
      polygonPoints.add(secondPolygonPoint);
      polygonPoints.add(thirdPolygonPoint);
      ConvexPolygon2D triangle = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(polygonPoints));

      Point2DBasics[] intersections = line2d.intersectionWith(triangle);
      assertEquals(1, intersections.length);

      line2d.setPoint(new Point2D(-0.5, 0));
      intersections = line2d.intersectionWith(triangle);
      assertEquals(2, intersections.length);

      line2d.setPoint(new Point2D(0.5, 0));
      intersections = line2d.intersectionWith(triangle);
      assertNull(intersections);

      line2d.set(0.0, 0.0, 0.0, 1.0);
      intersections = line2d.intersectionWith(triangle);

      // TODO: Define how many intersections there should be.
      // JvE: personal opinion is 2, current result is 1.
      // assertEquals(2, intersections.length);
   }

   @Test
   public void testcontainsNaN()
   {
      Random random = new Random(1776L);
      Point2D firstPointOnLine = randomPoint(random);
      Point2D secondPointOnLine = randomPoint(random);
      Line2D line2d = new Line2D(firstPointOnLine, secondPointOnLine);

      assertFalse(line2d.containsNaN());

      line2d.set(0.0, 0.0, 0.0, 1.0);
      assertFalse(line2d.containsNaN());
      line2d.set(Double.NaN, 0.0, 0.0, 1.0);
      assertTrue(line2d.containsNaN());
      line2d.set(0.0, Double.NaN, 0.0, 1.0);
      assertTrue(line2d.containsNaN());
      line2d.set(0.0, 0.0, Double.NaN, 1.0);
      assertTrue(line2d.containsNaN());
      line2d.set(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(line2d.containsNaN());
   }

   @Test
   public void testApplyTransformTranslation()
   {
      Random random = new Random(1776L);
      double delta = 1.0e-5;

      RigidBodyTransform transform = new RigidBodyTransform();

      // pure translation:
      Vector3D translation = new Vector3D(random.nextDouble(), random.nextDouble(), 0.0);
      Vector3D eulerAngles = new Vector3D(0.0, 0.0, 0.0);

      transform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.setTranslation(translation);

      Point2D firstPointOnLine = randomPoint(random);
      Point2D secondPointOnLine = randomPoint(random);
      Line2D line = new Line2D(firstPointOnLine, secondPointOnLine);
      Point2D point = new Point2D(line.getPoint());
      Vector2D vector = new Vector2D(line.getDirection());

      line.applyTransform(transform);
      assertEquals(point.getX() + translation.getX(), line.getPointX(), delta, "pure translation failed");
      assertEquals(point.getY() + translation.getY(), line.getPointY(), delta, "pure translation failed");
      assertEquals(vector.getX(), line.getDirectionX(), delta, "pure translation failed");
      assertEquals(vector.getY(), line.getDirectionY(), delta, "pure translation failed");
   }

   @Test
   public void testApplyTransformRotation()
   {
      Random random = new Random(1776L);
      double delta = 1.0e-5;

      RigidBodyTransform transform = new RigidBodyTransform();
      Point2D firstPointOnLine = randomPoint(random);
      Point2D secondPointOnLine = randomPoint(random);

      // pure translation:
      Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
      Vector3D eulerAngles = new Vector3D(0.0, 0.0, randomDouble(random, 2.0 * Math.PI));

      transform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.setTranslation(translation);

      Line2D line = new Line2D(firstPointOnLine, secondPointOnLine);
      Point2D point = new Point2D(line.getPoint());
      Vector2D vector = new Vector2D(line.getDirection());

      line.applyTransform(transform);

      double alpha = eulerAngles.getZ();
      double sina = Math.sin(alpha);
      double cosa = Math.cos(alpha);

      assertEquals(point.getX() * cosa - point.getY() * sina, line.getPointX(), delta, "pure rotation failed");
      assertEquals(point.getX() * sina + point.getY() * cosa, line.getPointY(), delta, "pure rotation failed");
      assertEquals(vector.getX() * cosa - vector.getY() * sina, line.getDirectionX(), delta, "pure rotation failed");
      assertEquals(vector.getX() * sina + vector.getY() * cosa, line.getDirectionY(), delta, "pure rotation failed");
   }

   @Test
   public void testApplyTransformRotationXaxisException()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
         Random random = new Random(1776L);
         RigidBodyTransform transform = new RigidBodyTransform();
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);

         // pure translation:
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Vector3D eulerAngles = new Vector3D(randomDouble(random, 2.0 * Math.PI), 0.0, 0.0);

         transform.setRotationEulerAndZeroTranslation(eulerAngles);
         transform.setTranslation(translation);

         Line2D line = new Line2D(firstPointOnLine, secondPointOnLine);

         line.applyTransform(transform);
      });
   }

   @Test
   public void testApplyTransformRotationYaxisException()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
         Random random = new Random(1776L);
         RigidBodyTransform transform = new RigidBodyTransform();
         Point2D firstPointOnLine = randomPoint(random);
         Point2D secondPointOnLine = randomPoint(random);

         // pure translation:
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Vector3D eulerAngles = new Vector3D(0.0, randomDouble(random, 2.0 * Math.PI), 0.0);

         transform.setRotationEulerAndZeroTranslation(eulerAngles);
         transform.setTranslation(translation);

         Line2D line = new Line2D(firstPointOnLine, secondPointOnLine);

         line.applyTransform(transform);
      });
   }

   @Test
   public void testApplyTransformCombination()
   {
      Random random = new Random(1776L);
      double delta = 1.0e-5;

      RigidBodyTransform transform = new RigidBodyTransform();
      Point2D firstPointOnLine = randomPoint(random);
      Point2D secondPointOnLine = randomPoint(random);

      // pure translation:
      Vector3D translation = new Vector3D(randomDouble(random), randomDouble(random), 0.0);
      Vector3D eulerAngles = new Vector3D(0.0, 0.0, randomDouble(random, 2.0 * Math.PI));

      transform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.setTranslation(translation);

      Line2D line = new Line2D(firstPointOnLine, secondPointOnLine);
      Point2D point = new Point2D(line.getPoint());
      Vector2D vector = new Vector2D(line.getDirection());

      line.applyTransform(transform);

      double alpha = eulerAngles.getZ();
      double sina = Math.sin(alpha);
      double cosa = Math.cos(alpha);

      assertEquals(point.getX() * cosa - point.getY() * sina + translation.getX(), line.getPointX(), delta, "pure rotation failed");
      assertEquals(point.getX() * sina + point.getY() * cosa + translation.getY(), line.getPointY(), delta, "pure rotation failed");
      assertEquals(vector.getX() * cosa - vector.getY() * sina, line.getDirectionX(), delta, "pure rotation failed");
      assertEquals(vector.getX() * sina + vector.getY() * cosa, line.getDirectionY(), delta, "pure rotation failed");
   }

   @Test
   public void testOrthogonalProjectionCopyPoint2dLine2d()
   {
      Point2D startPoint = new Point2D(-10.0, 0.0);
      Point2D endPoint = new Point2D(10.0, 0.0);
      Line2D line1 = new Line2D(startPoint, endPoint);

      Point2D origionalPoint = new Point2D(-20.0, 10.0);
      Point2DBasics projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(-20.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(-20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(-20, 0.0), projectedPoint);

      origionalPoint = new Point2D(-20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(-20.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(20.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(20.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(20.0, 0.0), projectedPoint);
      origionalPoint = new Point2D(20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(20.0, 0.0), projectedPoint);
      origionalPoint = new Point2D(0.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);
      origionalPoint = new Point2D(0.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(5.0, 0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(5.0, 0.0), projectedPoint);
   }

   @Test
   public void testIntersectionLine2dLine2d()
   {
      Line2D line1 = new Line2D(new Point2D(-10.0, 0.0), new Point2D(10.0, 0.0));
      Line2D line2 = new Line2D(new Point2D(-10.0, 10.0), new Point2D(10.0, 0.0));
      Line2D line3 = new Line2D(new Point2D(0.0, 10.0), new Point2D(0.0, -10.0));
      Line2D line4 = new Line2D(new Point2D(0.0, -10.0), new Point2D(0.0, 10.0));
      Line2D line5 = new Line2D(new Point2D(-10.0, 0.0), new Point2D(10.0, 0.0));
      Line2D line6 = new Line2D(new Point2D(10.0, 0.0), new Point2D(-10.0, 0.0));
      Line2D line7 = new Line2D(new Point2D(10.0, 0.0), new Point2D(20.0, 0.0));
      Line2D line8 = new Line2D(new Point2D(10.0, 0.0), new Point2D(-20.0, 0.0));
      Line2D line9 = new Line2D(new Point2D(10.1, 0.0), new Point2D(20.0, 0.0));
      Line2D line10 = new Line2D(new Point2D(10.0, 0.0), new Point2D(20.0, 1.0));
      Line2D line11 = new Line2D(new Point2D(-10.0, 1.0), new Point2D(10.0, 1.0));

      assertEquals(null, line1.intersectionWith(line11));

      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line6));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line10));

      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4));

      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line8));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line9));
   }

   @Test
   public void testDistancePointLineTwo()
   {
      Point2D pointOnLine = new Point2D(0.0, 1.0);
      Vector2D directionVector = new Vector2D(1.0, 0.0);
      Line2D line = new Line2D(pointOnLine, directionVector);

      Point2D point = new Point2D(0.0, 2.0);
      double distance = line.distance(point);
      double delta = 1e-12;
      assertEquals(1.0, distance, delta, "Distance to a horizontal line not calculated correctly");

      pointOnLine = new Point2D(-1.0, 0.0);
      directionVector = new Vector2D(0.0, 1.0);
      line = new Line2D(pointOnLine, directionVector);

      point = new Point2D(2.0, 3.0);
      distance = line.distance(point);
      delta = 1e-12;
      assertEquals(3.0, distance, delta, "Distance to a horizontal line not calculated correctly");
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(56021L);
      Line2D firstLine, secondLine;
      double epsilon = 1e-6;
      double scale;
      Vector2DBasics orthogonal;
      Vector2D direction = new Vector2D();

      firstLine = EuclidGeometryRandomTools.nextLine2D(random);
      secondLine = new Line2D(firstLine);

      assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      assertTrue(secondLine.geometricallyEquals(firstLine, epsilon));
      assertTrue(firstLine.geometricallyEquals(firstLine, epsilon));
      assertTrue(secondLine.geometricallyEquals(secondLine, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if translations are equal within +- epsilon and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine2D(random);
         secondLine = new Line2D(firstLine);

         orthogonal = firstLine.perpendicularVector();
         orthogonal.scale(0.99 * epsilon / orthogonal.length());

         secondLine.translate(orthogonal.getX(), orthogonal.getY());
         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));

         secondLine.set(firstLine);
         orthogonal = firstLine.perpendicularVector();
         orthogonal.scale(1.01 * epsilon / orthogonal.length());

         secondLine.translate(orthogonal.getX(), orthogonal.getY());
         assertFalse(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if directions are equal within +- epsilon and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine2D(random);
         secondLine = new Line2D(firstLine);

         direction = new Vector2D(secondLine.getDirection());
         RotationMatrixTools.applyYawRotation(epsilon * 0.99, direction, direction);
         secondLine.setDirection(direction);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));

         direction = new Vector2D(secondLine.getDirection());
         RotationMatrixTools.applyYawRotation(epsilon * 1.01, direction, direction);
         secondLine.setDirection(direction);

         assertFalse(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if translations lie somewhere on the same direction
         firstLine = EuclidGeometryRandomTools.nextLine2D(random);
         secondLine = new Line2D(firstLine);
         scale = random.nextDouble() - random.nextDouble();

         secondLine.translate(secondLine.getDirectionX() * scale, secondLine.getDirectionY() * scale);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if directions are equal but opposite and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine2D(random);
         direction.set(firstLine.getDirection());
         direction.negate();
         secondLine = new Line2D(firstLine.getPoint(), direction);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      }
   }
}

package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class Line3DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(2342L);

      Point3D expectedPoint = new Point3D();
      Vector3D expectedDirection = new Vector3D();

      Line3D line3d = new Line3D();
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      expectedPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0);
      expectedDirection.normalize();

      line3d = new Line3D(expectedPoint, expectedDirection);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
      assertTrue(expectedPoint != line3d.getPoint());
      assertTrue(expectedDirection != line3d.getDirection());

      Point3D firstPointOnLine = new Point3D(expectedPoint);
      Point3D secondPointOnLine = new Point3D();
      secondPointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), expectedDirection, firstPointOnLine);

      line3d = new Line3D(firstPointOnLine, secondPointOnLine);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      Line3D otherLine = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
      line3d = new Line3D(otherLine);
      EuclidCoreTestTools.assertTuple3DEquals(otherLine.getPoint(), line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(otherLine.getDirection(), line3d.getDirection(), EPSILON);
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(2342L);

      Point3D expectedPoint = new Point3D();
      Vector3D expectedDirection = new Vector3D();
      Vector3D direction = new Vector3D();

      Line3D line3d = new Line3D();

      expectedPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0);
      direction.set(expectedDirection);
      expectedDirection.normalize();

      line3d.setPoint(expectedPoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      line3d.setDirection(direction);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      expectedPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0);
      expectedDirection.normalize();

      line3d.set(expectedPoint, expectedDirection);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      Point3D firstPointOnLine = new Point3D(expectedPoint);
      Point3D secondPointOnLine = new Point3D();
      secondPointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), expectedDirection, firstPointOnLine);

      line3d.set(firstPointOnLine, secondPointOnLine);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      line3d = new Line3D();
      line3d.set(new Line3D(expectedPoint, expectedDirection));
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(2342L);

      Point3D expectedPoint = new Point3D();
      Vector3D expectedDirection = new Vector3D();
      Point3D actualPoint = new Point3D();
      Vector3D actualDirection = new Vector3D();

      expectedPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0);
      expectedDirection.normalize();
      Line3D line3d = new Line3D(expectedPoint, expectedDirection);

      line3d.getPoint(actualPoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, actualPoint, EPSILON);
      line3d.getDirection(actualDirection);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(32423L);

      Line3D line3D = EuclidGeometryRandomTools.generateRandomLine3D(random);
      line3D.setToZero();
      EuclidCoreTestTools.assertTuple3DIsSetToZero(line3D.getPoint());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(line3D.getDirection());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(32423L);

      Line3D line3D = EuclidGeometryRandomTools.generateRandomLine3D(random);
      line3D.setToNaN();
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(line3D.getPoint());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(line3D.getDirection());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Line3D line3D = new Line3D();
      assertFalse(line3D.containsNaN());
      line3D.set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      assertFalse(line3D.containsNaN());
      line3D.set(Double.NaN, 0.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(line3D.containsNaN());
      line3D.set(0.0, Double.NaN, 0.0, 0.0, 0.0, 1.0);
      assertTrue(line3D.containsNaN());
      line3D.set(0.0, 0.0, Double.NaN, 0.0, 0.0, 1.0);
      assertTrue(line3D.containsNaN());
      line3D.set(0.0, 0.0, 0.0, Double.NaN, 0.0, 1.0);
      assertTrue(line3D.containsNaN());
      line3D.set(0.0, 0.0, 0.0, 0.0, Double.NaN, 1.0);
      assertTrue(line3D.containsNaN());
      line3D.set(0.0, 0.0, 0.0, 0.0, 1.0, Double.NaN);
      assertTrue(line3D.containsNaN());
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

         double expectedDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, line.getPoint(), line.getDirection());
         double actualDistance = line.distance(query);
         assertEquals(expectedDistance, actualDistance, EPSILON);

         Line3D otherLine = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         expectedDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(line.getPoint(), line.getDirection(), otherLine.getPoint(), otherLine.getDirection());
         actualDistance = line.distance(otherLine);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testClosestPointsWith() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         Line3D otherLine = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);

         Point3D expectedClosestPointOnLine = new Point3D();
         Point3D expectedClosestPointOnOtherLine = new Point3D();
         Point3D actualClosestPointOnLine = new Point3D();
         Point3D actualClosestPointOnOtherLine = new Point3D();

         double expectedDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(line.getPoint(), line.getDirection(), otherLine.getPoint(),
                                                                                        otherLine.getDirection(), expectedClosestPointOnLine,
                                                                                        expectedClosestPointOnOtherLine);
         double actualDistance = line.closestPointsWith(otherLine, actualClosestPointOnLine, actualClosestPointOnOtherLine);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPointOnLine, actualClosestPointOnLine, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedClosestPointOnOtherLine, actualClosestPointOnOtherLine, EPSILON);
      }
   }

   @Test
   public void testNormalizeDirection() throws Exception
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line = new Line3D();

         Vector3D direction = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         line.setDirection(direction);
         assertFalse(direction.epsilonEquals(line.getDirection(), 1.0e-3));

         assertTrue(direction.dot(line.getDirection()) > 0.0);
         assertEquals(direction.length(), direction.dot(line.getDirection()), EPSILON);
         assertEquals(1.0, line.getDirection().length(), EPSILON);

         direction = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         line.setDirection(direction);
         assertTrue(line.containsNaN());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(line.getDirection());
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line3d = EuclidGeometryRandomTools.generateRandomLine3D(random, EPSILON);
         Transform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         Point3D expectedPoint = new Point3D();
         Vector3D expectedDirection = new Vector3D();
         line3d.getPoint(expectedPoint);
         line3d.getDirection(expectedDirection);

         line3d.applyTransform(transform);
         expectedPoint.applyTransform(transform);
         expectedDirection.applyTransform(transform);

         EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line1 = EuclidGeometryRandomTools.generateRandomLine3D(random, EPSILON);
         Line3D line2 = new Line3D(line1);
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         assertTrue(line1.epsilonEquals(line2, epsilon));

         for (int j = 0; j < 3; j++)
         {
            Point3D point = new Point3D();
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getPoint().getElement(j);
            line1.getPoint(point);
            point.setElement(j, element + 0.999 * epsilon);
            line2.setPoint(point);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line1.getPoint(point);
            point.setElement(j, element - 0.999 * epsilon);
            line2.setPoint(point);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line1.getPoint(point);
            point.setElement(j, element + 1.001 * epsilon);
            line2.setPoint(point);
            assertFalse(line1.epsilonEquals(line2, epsilon));
            line1.getPoint(point);
            point.setElement(j, element - 1.001 * epsilon);
            line2.setPoint(point);
            assertFalse(line1.epsilonEquals(line2, epsilon));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getDirection().getElement(j);
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element + 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element - 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element + 1.001 * epsilon);
            assertFalse(line1.epsilonEquals(line2, epsilon));
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element - 1.001 * epsilon);
            assertFalse(line1.epsilonEquals(line2, epsilon));
         }
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line1 = EuclidGeometryRandomTools.generateRandomLine3D(random, EPSILON);
         Line3D line2 = new Line3D(line1);
         double epsilon = 1.0e-15;
         assertTrue(line1.equals(line2));
         assertTrue(line1.equals((Object) line2));

         assertFalse(line1.equals((Line3D) null));
         assertFalse(line1.equals((Object) null));
         assertFalse(line1.equals((Object) new double[3]));

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.equals(line2));
            double element = line1.getPoint().getElement(j);
            ((Tuple3DBasics) line2.getPoint()).setElement(j, element + epsilon);
            assertFalse(line1.equals(line2));
            ((Tuple3DBasics) line2.getPoint()).setElement(j, element - epsilon);
            assertFalse(line1.equals(line2));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.equals(line2));
            double element = line1.getDirection().getElement(j);
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element + epsilon);
            assertFalse(line1.equals(line2));
            ((Tuple3DBasics) line2.getDirection()).setElement(j, element - epsilon);
            assertFalse(line1.equals(line2));
         }
      }
   }
}

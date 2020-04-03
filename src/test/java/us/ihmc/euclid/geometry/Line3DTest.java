package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Line3DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(2342L);

      Point3D expectedPoint = new Point3D();
      Vector3D expectedDirection = new Vector3D(1.0, 0.0, 0.0);

      Line3D line3d = new Line3D();
      try
      {
         EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      }
      catch (RuntimeException e)
      {
         // good
      }
      try
      {
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
      }
      catch (RuntimeException e)
      {
         // good
      }

      expectedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
      expectedDirection.normalize();

      line3d = new Line3D(expectedPoint, expectedDirection);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
      assertTrue(expectedPoint != line3d.getPoint());
      assertTrue(expectedDirection != line3d.getDirection());

      Point3D firstPointOnLine = new Point3D(expectedPoint);
      Point3D secondPointOnLine = new Point3D();
      secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), expectedDirection, firstPointOnLine);

      line3d = new Line3D(firstPointOnLine, secondPointOnLine);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      Line3D otherLine = EuclidGeometryRandomTools.nextLine3D(random, 10.0);
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

      Line3D line3d = new Line3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3D(random));

      expectedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
      direction.set(expectedDirection);
      expectedDirection.normalize();

      line3d.setPoint(expectedPoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      line3d.setDirection(direction);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      expectedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedDirection = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
      expectedDirection.normalize();

      line3d.set(expectedPoint, expectedDirection);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      Point3D firstPointOnLine = new Point3D(expectedPoint);
      Point3D secondPointOnLine = new Point3D();
      secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), expectedDirection, firstPointOnLine);

      line3d.set(firstPointOnLine, secondPointOnLine);
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);

      line3d = new Line3D();
      line3d.set(new Line3D(expectedPoint, expectedDirection));
      EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, line3d.getPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, line3d.getDirection(), EPSILON);
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(32423L);

      Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
      line3D.setToZero();
      try
      {
         EuclidCoreTestTools.assertTuple3DIsSetToZero(line3D.getPoint());
      }
      catch (RuntimeException e)
      {
         // Good
      }
      try
      {
         EuclidCoreTestTools.assertTuple3DEquals(Axis.X, line3D.getDirection(), EPSILON);
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

      Line3D line3D = EuclidGeometryRandomTools.nextLine3D(random);
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
         Line3D line = EuclidGeometryRandomTools.nextLine3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         double expectedDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, line.getPoint(), line.getDirection());
         double actualDistance = line.distance(query);
         assertEquals(expectedDistance, actualDistance, EPSILON);

         Line3D otherLine = EuclidGeometryRandomTools.nextLine3D(random, 10.0);
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
         Line3D line = EuclidGeometryRandomTools.nextLine3D(random, 10.0);
         Line3D otherLine = EuclidGeometryRandomTools.nextLine3D(random, 10.0);

         Point3D expectedClosestPointOnLine = new Point3D();
         Point3D expectedClosestPointOnOtherLine = new Point3D();
         Point3D actualClosestPointOnLine = new Point3D();
         Point3D actualClosestPointOnOtherLine = new Point3D();

         double expectedDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(line.getPoint(),
                                                                                        line.getDirection(),
                                                                                        otherLine.getPoint(),
                                                                                        otherLine.getDirection(),
                                                                                        expectedClosestPointOnLine,
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

         Point3D point = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D direction = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
         line.set(point, direction);
         assertFalse(direction.epsilonEquals(line.getDirection(), 1.0e-3));

         assertTrue(direction.dot(line.getDirection()) > 0.0);
         assertEquals(direction.length(), direction.dot(line.getDirection()), EPSILON);
         assertEquals(1.0, line.getDirection().length(), EPSILON);
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line3d = EuclidGeometryRandomTools.nextLine3D(random, EPSILON);
         Transform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Point3D expectedPoint = new Point3D(line3d.getPoint());
         Vector3D expectedDirection = new Vector3D(line3d.getDirection());

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
         Line3D line1 = EuclidGeometryRandomTools.nextLine3D(random, EPSILON);
         Line3D line2 = new Line3D(line1);
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         assertTrue(line1.epsilonEquals(line2, epsilon));

         for (int j = 0; j < 3; j++)
         {
            Point3D point = new Point3D();
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getPoint().getElement(j);
            point.set(line1.getPoint());
            point.setElement(j, element + 0.999 * epsilon);
            line2.setPoint(point);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            point.set(line1.getPoint());
            point.setElement(j, element - 0.999 * epsilon);
            line2.setPoint(point);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            point.set(line1.getPoint());
            point.setElement(j, element + 1.001 * epsilon);
            line2.setPoint(point);
            assertFalse(line1.epsilonEquals(line2, epsilon));
            point.set(line1.getPoint());
            point.setElement(j, element - 1.001 * epsilon);
            line2.setPoint(point);
            assertFalse(line1.epsilonEquals(line2, epsilon));
         }

         line2.set(line1);
         assertTrue(line1.epsilonEquals(line2, epsilon));
         AxisAngle axisAngle = new AxisAngle(EuclidCoreRandomTools.nextOrthogonalVector3D(random, line1.getDirection(), true), 0.0);
         axisAngle.setAngle(0.999 * epsilon);
         axisAngle.transform(line1.getDirection(), line2.getDirection());
         assertTrue(line1.epsilonEquals(line2, epsilon));
         axisAngle.setAngle(-0.999 * epsilon);
         axisAngle.transform(line1.getDirection(), line2.getDirection());
         assertTrue(line1.epsilonEquals(line2, epsilon));
         axisAngle.setAngle(2.0 * epsilon);
         axisAngle.transform(line1.getDirection(), line2.getDirection());
         assertFalse(line1.epsilonEquals(line2, epsilon));
         axisAngle.setAngle(-2.0 * epsilon);
         axisAngle.transform(line1.getDirection(), line2.getDirection());
         assertFalse(line1.epsilonEquals(line2, epsilon));
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line1 = EuclidGeometryRandomTools.nextLine3D(random);
         Line3D line2 = new Line3D(line1);
         double epsilon = 1.0e-12;
         //assertTrue(line1.equals(line2));
         //Object line2AsObject = line2;
         //assertTrue(line1.equals(line2AsObject));

         Line3D nullAsLine3D = null;
         assertFalse(line1.equals(nullAsLine3D));
         Object nullAsObject = null;
         assertFalse(line1.equals(nullAsObject));
         assertFalse(line1.equals(new double[3]));

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            //assertTrue(line1.equals(line2));

            Point3D point = new Point3D(line2.getPoint());
            double element = point.getElement(j);

            point.setElement(j, element + epsilon);
            line2.setPoint(point);

            assertFalse(line1.equals(line2));

            point.setElement(j, element - epsilon);
            line2.setPoint(point);

            assertFalse(line1.equals(line2));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            //assertTrue(line1.equals(line2));

            Vector3D direction = new Vector3D(line2.getDirection());
            double element = direction.getElement(j);

            direction.setElement(j, element + epsilon);
            line2.setDirection(direction);

            assertFalse(line1.equals(line2));

            direction.setElement(j, element - epsilon);
            line2.setDirection(direction);

            assertFalse(line1.equals(line2));
         }
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(57021L);
      Line3D firstLine, secondLine;
      double epsilon = 1e-6;
      double scale;
      Vector3D orthogonal, direction = new Vector3D();

      firstLine = EuclidGeometryRandomTools.nextLine3D(random);
      secondLine = new Line3D(firstLine);

      assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      assertTrue(secondLine.geometricallyEquals(firstLine, epsilon));
      assertTrue(firstLine.geometricallyEquals(firstLine, epsilon));
      assertTrue(secondLine.geometricallyEquals(secondLine, epsilon));

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if translations are equal within +- epsilon and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine3D(random);
         secondLine = new Line3D(firstLine);

         orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstLine.getDirection(), true);
         orthogonal.scale(0.99 * epsilon / orthogonal.length());

         secondLine.translate(orthogonal.getX(), orthogonal.getY(), orthogonal.getZ());
         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));

         secondLine.set(firstLine);

         orthogonal.scale(1.01 * epsilon / orthogonal.length());

         secondLine.translate(orthogonal.getX(), orthogonal.getY(), orthogonal.getZ());
         assertFalse(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if directions are equal within +- epsilon and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine3D(random);
         secondLine = new Line3D(firstLine);

         orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstLine.getDirection(), true);

         direction.set(secondLine.getDirection());
         direction.applyTransform(new RigidBodyTransform(new AxisAngle(orthogonal, epsilon * 0.99), new Vector3D()));
         secondLine.setDirection(direction);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));

         secondLine.set(firstLine);

         direction.set(secondLine.getDirection());
         direction.applyTransform(new RigidBodyTransform(new AxisAngle(orthogonal, epsilon * 1.01), new Vector3D()));
         secondLine.setDirection(direction);

         assertFalse(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if translations lie somewhere on the same direction
         firstLine = EuclidGeometryRandomTools.nextLine3D(random);
         secondLine = new Line3D(firstLine);
         scale = random.nextDouble() - random.nextDouble();

         secondLine.translate(secondLine.getDirectionX() * scale, secondLine.getDirectionY() * scale, secondLine.getDirectionZ() * scale);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Lines are equal if directions are equal but opposite and are otherwise the same
         firstLine = EuclidGeometryRandomTools.nextLine3D(random);
         direction.set(firstLine.getDirection());
         direction.negate();
         secondLine = new Line3D(firstLine.getPoint(), direction);

         assertTrue(firstLine.geometricallyEquals(secondLine, epsilon));
      }
   }
}

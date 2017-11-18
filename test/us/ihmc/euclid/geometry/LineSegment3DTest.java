package us.ihmc.euclid.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class LineSegment3DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(324324L);

      Point3D expectedFirstEndpoint = new Point3D();
      Point3D expectedSecondEndpoint = new Point3D();

      LineSegment3D lineSegment3D = new LineSegment3D();
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      lineSegment3D = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      lineSegment3D = new LineSegment3D(expectedFirstEndpoint.getX(), expectedFirstEndpoint.getY(), expectedFirstEndpoint.getZ(), expectedSecondEndpoint.getX(),
                                        expectedSecondEndpoint.getY(), expectedSecondEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      LineSegment3D otherLineSegment = new LineSegment3D(lineSegment3D);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, otherLineSegment.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, otherLineSegment.getSecondEndpoint(), EPSILON);
   }

   @Test
   public void testSetFirstEndpoint() throws Exception
   {
      Random random = new Random(324324L);

      Point3D expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

      LineSegment3D lineSegment3D = new LineSegment3D();
      lineSegment3D.setFirstEndpoint(expectedFirstEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      lineSegment3D.setFirstEndpoint(expectedFirstEndpoint.getX(), expectedFirstEndpoint.getY(), expectedFirstEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
   }

   @Test
   public void testSetSecondEndpoint() throws Exception
   {
      Random random = new Random(324324L);

      Point3D expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

      LineSegment3D lineSegment3D = new LineSegment3D();
      lineSegment3D.setSecondEndpoint(expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      lineSegment3D.setSecondEndpoint(expectedSecondEndpoint.getX(), expectedSecondEndpoint.getY(), expectedSecondEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(324324L);

      LineSegment3D lineSegment3D = new LineSegment3D();
      Point3D expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      Point3D expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

      lineSegment3D.set(expectedFirstEndpoint.getX(), expectedFirstEndpoint.getY(), expectedFirstEndpoint.getZ(), expectedSecondEndpoint.getX(),
                        expectedSecondEndpoint.getY(), expectedSecondEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      lineSegment3D.set(expectedFirstEndpoint, expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      Vector3D fromFirstToSecondEndpoint = new Vector3D();
      fromFirstToSecondEndpoint.sub(expectedSecondEndpoint, expectedFirstEndpoint);
      lineSegment3D.set(expectedFirstEndpoint, fromFirstToSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      LineSegment3D other = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
      lineSegment3D.set(other);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(32423L);

      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
      lineSegment3D.setToZero();
      EuclidCoreTestTools.assertTuple3DIsSetToZero(lineSegment3D.getFirstEndpoint());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(lineSegment3D.getSecondEndpoint());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(32423L);

      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
      lineSegment3D.setToNaN();
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(lineSegment3D.getFirstEndpoint());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(lineSegment3D.getSecondEndpoint());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      for (int i = 0; i < 3; i++)
      {
         LineSegment3D lineSegment3D = new LineSegment3D();
         assertFalse(lineSegment3D.containsNaN());
         lineSegment3D.getFirstEndpoint().setElement(i, Double.NaN);
         assertTrue(lineSegment3D.containsNaN());
         assertTrue(lineSegment3D.firstEndpointContainsNaN());
         assertFalse(lineSegment3D.secondEndpointContainsNaN());
      }

      for (int i = 0; i < 3; i++)
      {
         LineSegment3D lineSegment3D = new LineSegment3D();
         assertFalse(lineSegment3D.containsNaN());
         lineSegment3D.getSecondEndpoint().setElement(i, Double.NaN);
         assertTrue(lineSegment3D.containsNaN());
         assertFalse(lineSegment3D.firstEndpointContainsNaN());
         assertTrue(lineSegment3D.secondEndpointContainsNaN());
      }
   }

   @Test
   public void testLength() throws Exception
   {
      Random random = new Random(32434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double actualLength = lineSegment3D.length();
         double expectedLength = lineSegment3D.getFirstEndpoint().distance(lineSegment3D.getSecondEndpoint());
         assertEquals(expectedLength, actualLength, EPSILON);
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      { // distanceSquared(Point3DReadOnly point)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query, lineSegment3D.getFirstEndpoint(),
                                                                                         lineSegment3D.getSecondEndpoint());
         double actual = lineSegment3D.distanceSquared(query);
         assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // distance(Point3DReadOnly point)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         double actual = lineSegment3D.distance(query);
         assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // distance(LineSegment3D otherLineSegment)
         LineSegment3D lineSegment1 = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         LineSegment3D lineSegment2 = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegment1.getFirstEndpoint(), lineSegment1.getSecondEndpoint(),
                                                                                lineSegment2.getFirstEndpoint(), lineSegment2.getSecondEndpoint());
         double actual = lineSegment1.distance(lineSegment2);
         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjection() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // orthogonalProjection(Point3DReadOnly pointToProject)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         expected = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         actual = lineSegment3D.orthogonalProjectionCopy(query);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         expected = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         assertTrue(lineSegment3D.orthogonalProjection(query, actual));
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPointBetweenEndpointsGivenPercentage() throws Exception
   {
      Random random = new Random(342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // pointBetweenEndPointsGivenPercentage(double percentage)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);

         Point3D expected = new Point3D();
         expected.interpolate(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), percentage);
         Point3D actual = new Point3D();
         actual = lineSegment3D.pointBetweenEndpointsGivenPercentage(percentage);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // pointBetweenEndPointsGivenPercentage(double percentage, Point3DBasics pointToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);

         Point3D expected = new Point3D();
         expected.interpolate(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), percentage);
         Point3D actual = new Point3D();
         lineSegment3D.pointBetweenEndpointsGivenPercentage(percentage, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      try
      {
         new LineSegment3D().pointBetweenEndpointsGivenPercentage(-Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         new LineSegment3D().pointBetweenEndpointsGivenPercentage(1.0 + 1.11025e-16);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         new LineSegment3D().pointBetweenEndpointsGivenPercentage(-Double.MIN_VALUE, new Point3D());
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         new LineSegment3D().pointBetweenEndpointsGivenPercentage(1.0 + 1.11025e-16, new Point3D());
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testPointOnLineGivenPercentage() throws Exception
   {
      Random random = new Random(342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, 10.0);

         Point3D expected = new Point3D();
         expected.interpolate(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), percentage);
         Point3D actual = new Point3D();
         actual = lineSegment3D.pointOnLineGivenPercentage(percentage);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testMidPoint() throws Exception
   {
      Random random = new Random(32244L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D expected = new Point3D();
         expected = EuclidGeometryTools.averagePoint3Ds(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         Point3D actual = new Point3D();
         lineSegment3D.midpoint(actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGetDirection() throws Exception
   {
      Random random = new Random(ITERATIONS);

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(false, Vector3DBasics directionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Vector3D actualDirection = new Vector3D();
         lineSegment3D.getDirection(false, actualDirection);

         Point3D firstEndpoint = lineSegment3D.getFirstEndpoint();
         Point3D secondEndpoint = lineSegment3D.getSecondEndpoint();
         Vector3D expectedDirection = new Vector3D();
         expectedDirection.sub(secondEndpoint, firstEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
         assertEquals(lineSegment3D.length(), actualDirection.length(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(true, Vector3DBasics directionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Vector3D actualDirection = new Vector3D();
         lineSegment3D.getDirection(true, actualDirection);

         assertEquals(1.0, actualDirection.length(), EPSILON);

         Vector3D expectedDirection = new Vector3D();
         lineSegment3D.getDirection(false, expectedDirection);
         expectedDirection.scale(1.0 / lineSegment3D.length());
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(false)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Vector3D actualDirection = lineSegment3D.getDirection(false);

         Vector3D expectedDirection = new Vector3D();
         lineSegment3D.getDirection(false, expectedDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
         assertEquals(lineSegment3D.length(), actualDirection.length(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(true, Vector3DBasics directionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Vector3D actualDirection = lineSegment3D.getDirection(true);

         assertEquals(1.0, actualDirection.length(), EPSILON);

         Vector3D expectedDirection = new Vector3D();
         lineSegment3D.getDirection(true, expectedDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
      }
   }

   @Test
   public void testIsBetweenEndpoints() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         Point3D pointInside;
         pointInside = lineSegment3D.pointBetweenEndpointsGivenPercentage(percentage);
         assertTrue(lineSegment3D.isBetweenEndpoints(pointInside));
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double epsilonLength = epsilon * lineSegment3D.length();
         double distanceInside = Math.min(lineSegment3D.getFirstEndpoint().distance(pointInside), lineSegment3D.getSecondEndpoint().distance(pointInside));
         if (distanceInside < epsilonLength)
         {
            assertFalse(lineSegment3D.isBetweenEndpoints(pointInside, epsilon));
            assertFalse(lineSegment3D.isBetweenEndpoints(pointInside.getX(), pointInside.getY(), pointInside.getZ(), epsilon));
         }
         else
         {
            assertTrue(lineSegment3D.isBetweenEndpoints(pointInside, epsilon));
            assertTrue(lineSegment3D.isBetweenEndpoints(pointInside.getX(), pointInside.getY(), pointInside.getZ(), epsilon));
         }

         assertTrue(lineSegment3D.isBetweenEndpoints(new Point3D(lineSegment3D.getFirstEndpoint())));
         assertTrue(lineSegment3D.isBetweenEndpoints(new Point3D(lineSegment3D.getSecondEndpoint())));
         assertFalse(lineSegment3D.isBetweenEndpoints(new Point3D(lineSegment3D.getFirstEndpoint()), Double.MIN_VALUE));
         assertFalse(lineSegment3D.isBetweenEndpoints(new Point3D(lineSegment3D.getSecondEndpoint()), 0.555555e-16));
         assertFalse(lineSegment3D.isBetweenEndpoints(lineSegment3D.getFirstEndpoint().getX(), lineSegment3D.getFirstEndpoint().getY(),
                                                      lineSegment3D.getFirstEndpoint().getZ(), Double.MIN_VALUE));
         assertFalse(lineSegment3D.isBetweenEndpoints(lineSegment3D.getSecondEndpoint().getX(), lineSegment3D.getSecondEndpoint().getY(),
                                                      lineSegment3D.getSecondEndpoint().getZ(), 0.555555e-16));

         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointInside.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, pointInside);
         assertTrue(lineSegment3D.isBetweenEndpoints(pointInside));
         if (distanceInside < epsilonLength)
         {
            assertFalse(lineSegment3D.isBetweenEndpoints(pointInside, epsilon));
            assertFalse(lineSegment3D.isBetweenEndpoints(pointInside.getX(), pointInside.getY(), pointInside.getZ(), epsilon));
         }
         else
         {
            assertTrue(lineSegment3D.isBetweenEndpoints(pointInside, epsilon));
            assertTrue(lineSegment3D.isBetweenEndpoints(pointInside.getX(), pointInside.getY(), pointInside.getZ(), epsilon));
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0);
         Point3D pointBefore;
         pointBefore = lineSegment3D.pointOnLineGivenPercentage(percentage);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore));
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore.getX(), pointBefore.getY(), pointBefore.getZ(), epsilon));

         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointBefore.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, pointBefore);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore.getX(), pointBefore.getY(), pointBefore.getZ(), epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0);
         Point3D pointAfter;
         pointAfter = lineSegment3D.pointOnLineGivenPercentage(percentage);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter));
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter.getX(), pointAfter.getY(), pointAfter.getZ(), epsilon));

         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointAfter.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, pointAfter);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter.getX(), pointAfter.getY(), pointAfter.getZ(), epsilon));
      }
   }

   @Test
   public void testPercentageAlongLineSegment() throws Exception
   {
      Random random = new Random(23452L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);

         double expected = EuclidGeometryTools.percentageAlongLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         double actual = lineSegment3D.percentageAlongLineSegment(query);
         assertEquals(expected, actual, EPSILON);
         actual = lineSegment3D.percentageAlongLineSegment(query.getX(), query.getY(), query.getZ());
         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getters for the endpoints
         Point3D expectedFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point3D expectedSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         LineSegment3D lineSegment3D = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
         assertFalse(expectedFirstEndpoint == lineSegment3D.getFirstEndpoint());
         assertFalse(expectedSecondEndpoint == lineSegment3D.getSecondEndpoint());

         Point3D actualFirstEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point3D actualSecondEndpoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         lineSegment3D.getFirstEndpoint(actualFirstEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, actualFirstEndpoint, EPSILON);
         lineSegment3D.getSecondEndpoint(actualSecondEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, actualSecondEndpoint, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Line getters
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         Line3D actualLine = new Line3D();

         lineSegment3D.getLine(actualLine);
         EuclidCoreTestTools.assertTuple3DEquals(lineSegment3D.getFirstEndpoint(), actualLine.getPoint(), EPSILON);
         Vector3D expectedDirection = lineSegment3D.getDirection(true);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualLine.getDirection(), EPSILON);

         lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         actualLine = lineSegment3D.getLine();
         EuclidCoreTestTools.assertTuple3DEquals(lineSegment3D.getFirstEndpoint(), actualLine.getPoint(), EPSILON);
         expectedDirection = lineSegment3D.getDirection(true);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualLine.getDirection(), EPSILON);
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3d = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, EPSILON);
         Transform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         Point3D expectedFirstEndPoint = new Point3D();
         Point3D expectedSecondEndpoint = new Point3D();
         lineSegment3d.getFirstEndpoint(expectedFirstEndPoint);
         lineSegment3d.getSecondEndpoint(expectedSecondEndpoint);

         lineSegment3d.applyTransform(transform);
         expectedFirstEndPoint.applyTransform(transform);
         expectedSecondEndpoint.applyTransform(transform);

         EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndPoint, lineSegment3d.getFirstEndpoint(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3d.getSecondEndpoint(), EPSILON);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D line1 = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, EPSILON);
         LineSegment3D line2 = new LineSegment3D(line1);
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         assertTrue(line1.epsilonEquals(line2, epsilon));

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getFirstEndpoint().getElement(j);
            line2.getFirstEndpoint().setElement(j, element + 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line2.getFirstEndpoint().setElement(j, element - 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line2.getFirstEndpoint().setElement(j, element + 1.001 * epsilon);
            assertFalse(line1.epsilonEquals(line2, epsilon));
            line2.getFirstEndpoint().setElement(j, element - 1.001 * epsilon);
            assertFalse(line1.epsilonEquals(line2, epsilon));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getSecondEndpoint().getElement(j);
            line2.getSecondEndpoint().setElement(j, element + 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line2.getSecondEndpoint().setElement(j, element - 0.999 * epsilon);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            line2.getSecondEndpoint().setElement(j, element + 1.001 * epsilon);
            assertFalse(line1.epsilonEquals(line2, epsilon));
            line2.getSecondEndpoint().setElement(j, element - 1.001 * epsilon);
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
         LineSegment3D line1 = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, EPSILON);
         LineSegment3D line2 = new LineSegment3D(line1);
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
            double element = line1.getFirstEndpoint().getElement(j);
            line2.getFirstEndpoint().setElement(j, element + epsilon);
            assertFalse(line1.equals(line2));
            line2.getFirstEndpoint().setElement(j, element - epsilon);
            assertFalse(line1.equals(line2));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.equals(line2));
            double element = line1.getSecondEndpoint().getElement(j);
            line2.getSecondEndpoint().setElement(j, element + epsilon);
            assertFalse(line1.equals(line2));
            line2.getSecondEndpoint().setElement(j, element - epsilon);
            assertFalse(line1.equals(line2));
         }
      }
   }

   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(19263L);
      double epsilon = 1e-7;
      Point3D segment1Point1, segment1Point2, segment2Point1, segment2Point2;
      LineSegment3D testSegment1, testSegment2;
      Vector3D translation;

      for (int i = 0; i < ITERATIONS; ++i) {
         segment1Point1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         segment1Point2 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         segment2Point1 = new Point3D(segment1Point1);
         segment2Point2 = new Point3D(segment1Point2);

         testSegment1 = new LineSegment3D(segment1Point1, segment1Point2);
         testSegment2 = new LineSegment3D(segment2Point1, segment2Point2);

         assertTrue(testSegment1.geometricallyEquals(testSegment1, epsilon));
         assertTrue(testSegment1.geometricallyEquals(testSegment2, epsilon));
         assertTrue(testSegment2.geometricallyEquals(testSegment1, epsilon));
         assertTrue(testSegment2.geometricallyEquals(testSegment2, epsilon));

         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 0.99 * epsilon);
         segment2Point2.add(translation);
         
         testSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         
         assertTrue(testSegment1.geometricallyEquals(testSegment2, epsilon));
         
         segment2Point2 = new Point3D(segment1Point2);
         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.01 * epsilon);
         segment2Point2.add(translation);
         
         testSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         
         assertFalse(testSegment1.geometricallyEquals(testSegment2, epsilon));

         segment2Point2 = new Point3D(segment1Point2);
         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 0.99 * epsilon);
         segment2Point1.add(translation);
         
         testSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         
         assertTrue(testSegment1.geometricallyEquals(testSegment2, epsilon));
         
         segment2Point1 = new Point3D(segment1Point1);
         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.01 * epsilon);
         segment2Point1.add(translation);
         
         testSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         
         assertFalse(testSegment1.geometricallyEquals(testSegment2, epsilon));
      }
   }
}

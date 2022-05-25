package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class LineSegment3DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(324324L);

      Point3D expectedFirstEndpoint = new Point3D();
      Point3D expectedSecondEndpoint = new Point3D();

      LineSegment3D lineSegment3D = new LineSegment3D();
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      lineSegment3D = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      lineSegment3D = new LineSegment3D(expectedFirstEndpoint.getX(),
                                        expectedFirstEndpoint.getY(),
                                        expectedFirstEndpoint.getZ(),
                                        expectedSecondEndpoint.getX(),
                                        expectedSecondEndpoint.getY(),
                                        expectedSecondEndpoint.getZ());
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

      Point3D expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

      LineSegment3D lineSegment3D = new LineSegment3D();
      lineSegment3D.setFirstEndpoint(expectedFirstEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      lineSegment3D.setFirstEndpoint(expectedFirstEndpoint.getX(), expectedFirstEndpoint.getY(), expectedFirstEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
   }

   @Test
   public void testSetSecondEndpoint() throws Exception
   {
      Random random = new Random(324324L);

      Point3D expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

      LineSegment3D lineSegment3D = new LineSegment3D();
      lineSegment3D.setSecondEndpoint(expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      lineSegment3D.setSecondEndpoint(expectedSecondEndpoint.getX(), expectedSecondEndpoint.getY(), expectedSecondEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(324324L);

      LineSegment3D lineSegment3D = new LineSegment3D();
      Point3D expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      Point3D expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

      lineSegment3D.set(expectedFirstEndpoint.getX(),
                        expectedFirstEndpoint.getY(),
                        expectedFirstEndpoint.getZ(),
                        expectedSecondEndpoint.getX(),
                        expectedSecondEndpoint.getY(),
                        expectedSecondEndpoint.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      lineSegment3D.set(expectedFirstEndpoint, expectedSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      Vector3D fromFirstToSecondEndpoint = new Vector3D();
      fromFirstToSecondEndpoint.sub(expectedSecondEndpoint, expectedFirstEndpoint);
      lineSegment3D.set(expectedFirstEndpoint, fromFirstToSecondEndpoint);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);

      expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      LineSegment3D other = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
      lineSegment3D.set(other);
      EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(32423L);

      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random);
      lineSegment3D.setToZero();
      EuclidCoreTestTools.assertTuple3DIsSetToZero(lineSegment3D.getFirstEndpoint());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(lineSegment3D.getSecondEndpoint());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(32423L);

      LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random);
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

         Point3D firstEndpoint = new Point3D();
         firstEndpoint.setElement(i, Double.NaN);
         lineSegment3D.setFirstEndpoint(firstEndpoint);

         assertTrue(lineSegment3D.containsNaN());
         assertTrue(lineSegment3D.firstEndpointContainsNaN());
         assertFalse(lineSegment3D.secondEndpointContainsNaN());
      }

      for (int i = 0; i < 3; i++)
      {
         LineSegment3D lineSegment3D = new LineSegment3D();
         assertFalse(lineSegment3D.containsNaN());

         Point3D secondEndpoint = new Point3D();
         secondEndpoint.setElement(i, Double.NaN);
         lineSegment3D.setSecondEndpoint(secondEndpoint);

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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query,
                                                                                         lineSegment3D.getFirstEndpoint(),
                                                                                         lineSegment3D.getSecondEndpoint());
         double actual = lineSegment3D.distanceSquared(query);
         assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // distance(Point3DReadOnly point)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         double actual = lineSegment3D.distance(query);
         assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // distance(LineSegment3D otherLineSegment)
         LineSegment3D lineSegment1 = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         LineSegment3D lineSegment2 = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double expected = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegment1.getFirstEndpoint(),
                                                                                lineSegment1.getSecondEndpoint(),
                                                                                lineSegment2.getFirstEndpoint(),
                                                                                lineSegment2.getSecondEndpoint());
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D expected = new Point3D();
         Point3DBasics actual = new Point3D();

         expected = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(query, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         actual = lineSegment3D.orthogonalProjectionCopy(query);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         Point3D expected = new Point3D();
         expected.interpolate(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), percentage);
         Point3DBasics actual = new Point3D();
         actual = lineSegment3D.pointBetweenEndpointsGivenPercentage(percentage);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // pointBetweenEndPointsGivenPercentage(double percentage, Point3DBasics pointToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, 10.0);

         Point3D expected = new Point3D();
         expected.interpolate(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), percentage);
         Point3DBasics actual = new Point3D();
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Vector3D actualDirection = new Vector3D();
         lineSegment3D.getDirection(false, actualDirection);

         Point3DReadOnly firstEndpoint = lineSegment3D.getFirstEndpoint();
         Point3DReadOnly secondEndpoint = lineSegment3D.getSecondEndpoint();
         Vector3D expectedDirection = new Vector3D();
         expectedDirection.sub(secondEndpoint, firstEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
         assertEquals(lineSegment3D.length(), actualDirection.length(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(true, Vector3DBasics directionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Vector3DBasics actualDirection = lineSegment3D.getDirection(false);

         Vector3D expectedDirection = new Vector3D();
         lineSegment3D.getDirection(false, expectedDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedDirection, actualDirection, EPSILON);
         assertEquals(lineSegment3D.length(), actualDirection.length(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // getDirection(true, Vector3DBasics directionToPack)
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Vector3DBasics actualDirection = lineSegment3D.getDirection(true);

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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3DBasics pointInside;
         pointInside = lineSegment3D.pointBetweenEndpointsGivenPercentage(percentage);
         assertTrue(lineSegment3D.isBetweenEndpoints(pointInside));
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
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
         assertFalse(lineSegment3D.isBetweenEndpoints(lineSegment3D.getFirstEndpoint().getX(),
                                                      lineSegment3D.getFirstEndpoint().getY(),
                                                      lineSegment3D.getFirstEndpoint().getZ(),
                                                      Double.MIN_VALUE));
         assertFalse(lineSegment3D.isBetweenEndpoints(lineSegment3D.getSecondEndpoint().getX(),
                                                      lineSegment3D.getSecondEndpoint().getY(),
                                                      lineSegment3D.getSecondEndpoint().getZ(),
                                                      0.555555e-16));

         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointInside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointInside);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         Point3DBasics pointBefore;
         pointBefore = lineSegment3D.pointOnLineGivenPercentage(percentage);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore));
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore.getX(), pointBefore.getY(), pointBefore.getZ(), epsilon));

         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointBefore.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointBefore);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointBefore.getX(), pointBefore.getY(), pointBefore.getZ(), epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         double percentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         Point3DBasics pointAfter;
         pointAfter = lineSegment3D.pointOnLineGivenPercentage(percentage);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter));
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter, epsilon));
         assertFalse(lineSegment3D.isBetweenEndpoints(pointAfter.getX(), pointAfter.getY(), pointAfter.getZ(), epsilon));

         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegment3D.getDirection(true), true);
         pointAfter.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointAfter);
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
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         Point3D query = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

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
         Point3D expectedFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D expectedSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         LineSegment3D lineSegment3D = new LineSegment3D(expectedFirstEndpoint, expectedSecondEndpoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, lineSegment3D.getFirstEndpoint(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, lineSegment3D.getSecondEndpoint(), EPSILON);
         assertFalse(expectedFirstEndpoint == lineSegment3D.getFirstEndpoint());
         assertFalse(expectedSecondEndpoint == lineSegment3D.getSecondEndpoint());

         Point3D actualFirstEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D actualSecondEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         actualFirstEndpoint.set(lineSegment3D.getFirstEndpoint());
         EuclidCoreTestTools.assertTuple3DEquals(expectedFirstEndpoint, actualFirstEndpoint, EPSILON);
         actualSecondEndpoint.set(lineSegment3D.getSecondEndpoint());
         EuclidCoreTestTools.assertTuple3DEquals(expectedSecondEndpoint, actualSecondEndpoint, EPSILON);
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3d = EuclidGeometryRandomTools.nextLineSegment3D(random, EPSILON);
         Transform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         Point3D expectedFirstEndPoint = new Point3D(lineSegment3d.getFirstEndpoint());
         Point3D expectedSecondEndpoint = new Point3D(lineSegment3d.getSecondEndpoint());

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
         LineSegment3D line1 = EuclidGeometryRandomTools.nextLineSegment3D(random, EPSILON);
         LineSegment3D line2 = new LineSegment3D(line1);
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         assertTrue(line1.epsilonEquals(line2, epsilon));

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getFirstEndpoint().getElement(j);

            Point3D firstEndpoint = new Point3D(line2.getFirstEndpoint());
            firstEndpoint.setElement(j, element + 0.999 * epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertTrue(line1.epsilonEquals(line2, epsilon));

            firstEndpoint.setElement(j, element - 0.999 * epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertTrue(line1.epsilonEquals(line2, epsilon));

            firstEndpoint.setElement(j, element + 1.001 * epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertFalse(line1.epsilonEquals(line2, epsilon));

            firstEndpoint.setElement(j, element - 1.001 * epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertFalse(line1.epsilonEquals(line2, epsilon));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.epsilonEquals(line2, epsilon));
            double element = line1.getSecondEndpoint().getElement(j);

            Point3D secondEndpoint = new Point3D(line2.getSecondEndpoint());
            secondEndpoint.setElement(j, element + 0.999 * epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertTrue(line1.epsilonEquals(line2, epsilon));

            secondEndpoint.setElement(j, element - 0.999 * epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertTrue(line1.epsilonEquals(line2, epsilon));

            secondEndpoint.setElement(j, element + 1.001 * epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertFalse(line1.epsilonEquals(line2, epsilon));

            secondEndpoint.setElement(j, element - 1.001 * epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertFalse(line1.epsilonEquals(line2, epsilon));
         }
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D line1 = EuclidGeometryRandomTools.nextLineSegment3D(random, EPSILON);
         LineSegment3D line2 = new LineSegment3D(line1);
         double epsilon = 1.0e-15;
         assertTrue(line1.equals(line2));
         Object line2AsObject = line2;
         assertTrue(line1.equals(line2AsObject));

         LineSegment3D nullAsLineSegment3D = null;
         assertFalse(line1.equals(nullAsLineSegment3D));
         Object nullAsObject = null;
         assertFalse(line1.equals(nullAsObject));
         assertFalse(line1.equals(new double[3]));

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.equals(line2));
            double element = line1.getFirstEndpoint().getElement(j);

            Point3D firstEndpoint = new Point3D(line2.getFirstEndpoint());
            firstEndpoint.setElement(j, element + epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertFalse(line1.equals(line2));

            firstEndpoint.setElement(j, element - epsilon);
            line2.setFirstEndpoint(firstEndpoint);

            assertFalse(line1.equals(line2));
         }

         for (int j = 0; j < 3; j++)
         {
            line2.set(line1);
            assertTrue(line1.equals(line2));
            double element = line1.getSecondEndpoint().getElement(j);

            Point3D secondEndpoint = new Point3D(line2.getSecondEndpoint());
            secondEndpoint.setElement(j, element + epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertFalse(line1.equals(line2));

            secondEndpoint.setElement(j, element - epsilon);
            line2.setSecondEndpoint(secondEndpoint);

            assertFalse(line1.equals(line2));
         }
      }
   }

   @Test
   public void testTranslate() throws Exception
   {
      Random random = new Random(3653);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test translate(double x, double y)
         LineSegment3D originalLineSegment = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         LineSegment3D translatedLineSegment = new LineSegment3D(originalLineSegment);
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, 10.0);
         translatedLineSegment.translate(x, y, z);

         assertEquals(originalLineSegment.length(), translatedLineSegment.length(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalLineSegment.getDirection(false), translatedLineSegment.getDirection(false), EPSILON);

         LineSegment3D expectedLineSegment = new LineSegment3D(originalLineSegment);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getTranslation().set(x, y, z);
         expectedLineSegment.applyTransform(transform);
         EuclidGeometryTestTools.assertLineSegment3DEquals(expectedLineSegment, translatedLineSegment, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test translate(Tuple3DReadOnly translation)
         LineSegment3D originalLineSegment = EuclidGeometryRandomTools.nextLineSegment3D(random, 10.0);
         LineSegment3D translatedLineSegment = new LineSegment3D(originalLineSegment);
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         translatedLineSegment.translate(translation);

         assertEquals(originalLineSegment.length(), translatedLineSegment.length(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(originalLineSegment.getDirection(false), translatedLineSegment.getDirection(false), EPSILON);

         LineSegment3D expectedLineSegment = new LineSegment3D(originalLineSegment);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getTranslation().set(translation);
         expectedLineSegment.applyTransform(transform);
         EuclidGeometryTestTools.assertLineSegment3DEquals(expectedLineSegment, translatedLineSegment, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(19263L);
      Point3D segment1Point1, segment1Point2, segment2Point1, segment2Point2;
      LineSegment3D lineSegment1, lineSegment2;

      segment1Point1 = EuclidCoreRandomTools.nextPoint3D(random);
      segment1Point2 = EuclidCoreRandomTools.nextPoint3D(random);
      segment2Point1 = new Point3D(segment1Point1);
      segment2Point2 = new Point3D(segment1Point2);

      lineSegment1 = new LineSegment3D(segment1Point1, segment1Point2);
      lineSegment2 = new LineSegment3D(segment2Point1, segment2Point2);

      assertTrue(lineSegment1.geometricallyEquals(lineSegment1, EPSILON));
      assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
      assertTrue(lineSegment2.geometricallyEquals(lineSegment1, EPSILON));
      assertTrue(lineSegment2.geometricallyEquals(lineSegment2, EPSILON));

      for (int i = 0; i < ITERATIONS; ++i)
      {
         segment1Point1 = EuclidCoreRandomTools.nextPoint3D(random);
         segment1Point2 = EuclidCoreRandomTools.nextPoint3D(random);
         segment2Point1 = new Point3D();
         segment2Point2 = new Point3D();

         lineSegment1 = new LineSegment3D(segment1Point1, segment1Point2);

         segment2Point1.set(segment1Point1);
         segment2Point2.add(segment1Point2, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point2, segment2Point1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.set(segment1Point1);
         segment2Point2.add(segment1Point2, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point2, segment2Point1);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.add(segment1Point1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON));
         segment2Point2.set(segment1Point2);
         lineSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point2, segment2Point1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         segment2Point1.add(segment1Point1, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON));
         segment2Point2.set(segment1Point2);
         lineSegment2 = new LineSegment3D(segment2Point1, segment2Point2);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2 = new LineSegment3D(segment2Point2, segment2Point1);
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         lineSegment2.set(lineSegment1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.translate(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * EPSILON));
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.flipDirection();
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));

         lineSegment2.set(lineSegment1);
         assertTrue(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.translate(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * EPSILON));
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
         lineSegment2.flipDirection();
         assertFalse(lineSegment1.geometricallyEquals(lineSegment2, EPSILON));
      }
   }
}

package us.ihmc.euclid.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class Pose2DTest
{
   private double epsilon = 1e-7;
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(52942L);
      double x, y, yaw;
      Vector2D tuple;
      Orientation2D orientation;
      Pose2D allDoubles, toCopy, copyPose, fromComponents;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = Math.PI - (2.0 * Math.PI * random.nextDouble());

         allDoubles = new Pose2D(x, y, yaw);
         assertEquals(x, allDoubles.getX(), epsilon);
         assertEquals(y, allDoubles.getY(), epsilon);
         assertEquals(yaw, allDoubles.getYaw(), epsilon);

         toCopy = new Pose2D(x, y, yaw);
         copyPose = new Pose2D(toCopy);

         assertEquals(x, copyPose.getX(), epsilon);
         assertEquals(y, copyPose.getY(), epsilon);
         assertEquals(yaw, copyPose.getYaw(), epsilon);

         tuple = new Vector2D(x, y);
         orientation = new Orientation2D(yaw);
         fromComponents = new Pose2D(tuple, orientation);

         assertEquals(x, fromComponents.getX(), epsilon);
         assertEquals(y, fromComponents.getY(), epsilon);
         assertEquals(yaw, fromComponents.getYaw(), epsilon);
      }
   }

   @Test
   public void testSetToNaN()
   {
      Random random = new Random(70324L);
      Pose2D toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.setToNaN();

         assertEquals(Double.NaN, toSet.getX(), epsilon);
         assertEquals(Double.NaN, toSet.getY(), epsilon);
         assertEquals(Double.NaN, toSet.getYaw(), epsilon);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(70924L);
      Pose2D toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.setToZero();

         assertEquals(0, toSet.getX(), epsilon);
         assertEquals(0, toSet.getY(), epsilon);
         assertEquals(0, toSet.getYaw(), epsilon);
      }
   }

   @Test
   public void testSetComponents()
   {
      Random random = new Random(71484L);
      double x, y, yaw;
      Vector2D tuple;
      Orientation2D orientation;
      Pose2D toSet, toCopy;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.setX(x);
         toSet.setY(y);
         toSet.setYaw(yaw);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.setPosition(x, y);
         toSet.setYaw(yaw);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.set(x, y, yaw);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         tuple = new Vector2D(x, y);
         orientation = new Orientation2D(yaw);
         toSet.set(tuple, orientation);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toSet.setPosition(tuple);
         toSet.setOrientation(orientation);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);

         toSet = EuclidGeometryRandomTools.generateRandomPose2D(random);

         toCopy = new Pose2D(x, y, yaw);

         toSet.set(toCopy);

         assertEquals(x, toSet.getX(), epsilon);
         assertEquals(y, toSet.getY(), epsilon);
         assertEquals(yaw, toSet.getYaw(), epsilon);
      }
   }

   @Test
   public void testPointDistance()
   {
      Random random = new Random(41133L);
      Pose2D firstPose, secondPose;
      Point2D point = new Point2D();
      Vector2D translation;
      double length;

      for (int i = 0; i < ITERATIONS; i++)
      {
         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         translation = EuclidCoreRandomTools.generateRandomVector2D(random);
         length = translation.length();

         secondPose.appendTranslation(translation);

         assertEquals(length, firstPose.getPositionDistance(secondPose), epsilon);

         firstPose.getPosition(point);
         point.add(translation);

         assertEquals(length, firstPose.getPositionDistance(point), epsilon);
      }
   }

   @Test
   public void testOrientationDistance()
   {
      Random random = new Random(59886L);
      Pose2D firstPose, secondPose;
      Orientation2D orientation = new Orientation2D();
      double angleDiff;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         angleDiff = Math.PI - (2.0 * Math.PI * random.nextDouble());

         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         secondPose.appendRotation(angleDiff);

         assertEquals(Math.abs(angleDiff), firstPose.getOrientationDistance(secondPose), epsilon);

         firstPose.getOrientation(orientation);
         orientation.add(angleDiff);

         assertEquals(Math.abs(angleDiff), firstPose.getOrientationDistance(orientation), epsilon);
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(19732L);
      Pose2D firstPose, secondPose;
      double epsilon = 1e-7;
      Vector2D perturb;

      firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
      secondPose = new Pose2D(firstPose);

      assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));
      assertTrue(secondPose.geometricallyEquals(firstPose, epsilon));
      assertTrue(firstPose.geometricallyEquals(firstPose, epsilon));
      assertTrue(secondPose.geometricallyEquals(secondPose, epsilon));

      // Orientation
      for (int i = 0; i < ITERATIONS; ++i)
      { // Poses are equal if orientations are equal within +- epsilon points are the same
         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         secondPose.setYaw(firstPose.getYaw() + 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() - 0.99 * epsilon);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() + 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose.setYaw(firstPose.getYaw() - 1.01 * epsilon);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }

      // Point
      for (int i = 0; i < ITERATIONS; ++i)
      { // Poses are equal if points are equal within +- epsilon and orientations are the same
         firstPose = EuclidGeometryRandomTools.generateRandomPose2D(random);
         secondPose = new Pose2D(firstPose);

         perturb = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 0.99 * epsilon);
         perturb.add(secondPose.getPosition());
         secondPose.setPosition(perturb);

         assertTrue(firstPose.geometricallyEquals(secondPose, epsilon));

         secondPose = new Pose2D(firstPose);
         perturb = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.01 * epsilon);
         perturb.add(secondPose.getPosition());
         secondPose.setPosition(perturb);

         assertFalse(firstPose.geometricallyEquals(secondPose, epsilon));
      }
   }
}

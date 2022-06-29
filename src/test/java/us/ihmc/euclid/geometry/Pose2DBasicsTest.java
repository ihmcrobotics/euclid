package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Vector2D;

public abstract class Pose2DBasicsTest<T extends Pose2DBasics>
{
   private static final double EPSILON = 1e-7;

   public abstract T createEmptyPose2D();

   public abstract T createRandomPose2D(Random random);

   public T copy(Pose2DReadOnly source)
   {
      T copy = createEmptyPose2D();
      copy.set(source);
      return copy;
   }

   @Test
   public void testSetToNaN()
   {
      Random random = new Random(70324L);
      T toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = createRandomPose2D(random);

         toSet.setToNaN();

         assertEquals(Double.NaN, toSet.getX(), EPSILON);
         assertEquals(Double.NaN, toSet.getY(), EPSILON);
         assertEquals(Double.NaN, toSet.getYaw(), EPSILON);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(70924L);
      T toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = createRandomPose2D(random);

         toSet.setToZero();

         assertEquals(0, toSet.getX(), EPSILON);
         assertEquals(0, toSet.getY(), EPSILON);
         assertEquals(0, toSet.getYaw(), EPSILON);
      }
   }

   @Test
   public void testSetComponents()
   {
      Random random = new Random(71484L);
      double x, y, yaw;
      Vector2D tuple;
      Orientation2D orientation;
      T toSet, toCopy;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         toSet = createRandomPose2D(random);

         toSet.setX(x);
         toSet.setY(y);
         toSet.setYaw(yaw);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);

         toSet = createRandomPose2D(random);

         toSet.getPosition().set(x, y);
         toSet.setYaw(yaw);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);

         toSet = createRandomPose2D(random);

         toSet.set(x, y, yaw);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);

         toSet = createRandomPose2D(random);

         tuple = new Vector2D(x, y);
         orientation = new Orientation2D(yaw);
         toSet.set(tuple, orientation);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);

         toSet = createRandomPose2D(random);

         toSet.getPosition().set(tuple);
         toSet.getOrientation().set(orientation);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);

         toSet = createRandomPose2D(random);

         toCopy = createEmptyPose2D();
         toCopy.set(x, y, yaw);

         toSet.set(toCopy);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(yaw, toSet.getYaw(), EPSILON);
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(9827L);
      T firstPose, secondPose;
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double yaw = random.nextDouble() - random.nextDouble();
      double angleDiff;
      Vector2D translation;

      firstPose = createEmptyPose2D();
      firstPose.set(x, y, yaw);
      secondPose = createEmptyPose2D();
      secondPose.set(x, y, yaw);

      // Sanity checks
      assertTrue(firstPose.equals(secondPose));
      assertTrue(secondPose.equals(secondPose));
      assertTrue(firstPose.equals(firstPose));
      assertTrue(secondPose.equals(secondPose));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if point components are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         firstPose = createEmptyPose2D();
         firstPose.set(x, y, yaw);
         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EPSILON);
         secondPose.appendTranslation(translation);

         assertFalse(firstPose.equals(secondPose));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if orientation angles are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         firstPose = createEmptyPose2D();
         firstPose.set(x, y, yaw);
         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         angleDiff = (random.nextBoolean() ? 1 : -1) * EPSILON;

         secondPose.appendRotation(angleDiff);

         assertFalse(firstPose.equals(secondPose));
      }
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(9827L);
      T firstPose, secondPose;
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double yaw = random.nextDouble() - random.nextDouble();
      double angleDiff;

      firstPose = createEmptyPose2D();
      firstPose.set(x, y, yaw);
      secondPose = createEmptyPose2D();
      secondPose.set(x, y, yaw);

      // Sanity checks
      assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(firstPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(secondPose, EPSILON));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when distance between point components is <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         firstPose = createEmptyPose2D();
         firstPose.set(x, y, yaw);
         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when angle between orientations is <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         yaw = random.nextDouble() - random.nextDouble();

         firstPose = createEmptyPose2D();
         firstPose.set(x, y, yaw);
         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         angleDiff = (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON;

         secondPose.appendRotation(angleDiff);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = createEmptyPose2D();
         secondPose.set(x, y, yaw);

         angleDiff = (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON;

         secondPose.appendRotation(angleDiff);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }
   }

}

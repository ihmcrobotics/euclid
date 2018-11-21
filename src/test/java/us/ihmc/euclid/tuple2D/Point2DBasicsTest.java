package us.ihmc.euclid.tuple2D;

import static org.junit.Assert.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;

public abstract class Point2DBasicsTest<T extends Point2DBasics> extends Tuple2DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistance = p1.distance(p2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         T p2 = createTuple(p1.getX() + translation.getX(), p1.getY() + translation.getY());
         double actualDistanceSquared = p1.distanceSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistanceSquared, getEpsilon());
      }
   }

   @Test
   public void testDistanceXY()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p1 = createRandomTuple(random);
         Point3D p2 = new Point3D(p1.getX() + translation.getX(), p1.getY() + translation.getY(), random.nextDouble());
         double actualDistance = p1.distanceXY(p2);
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceXYSquared()
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p1 = createRandomTuple(random);
         Point3D p2 = new Point3D(p1.getX() + translation.getX(), p1.getY() + translation.getY(), random.nextDouble());
         double actualDistance = p1.distanceXYSquared(p2);
         assertEquals(expectedDistanceSquared, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOrigin() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(expectedDistance);
         T p = createTuple(translation.getX(), translation.getY());
         double actualDistance = p.distanceFromOrigin();
         assertEquals(expectedDistance, actualDistance, getEpsilon());
      }
   }

   @Test
   public void testDistanceFromOriginSquared() throws Exception
   {
      Random random = new Random(654135L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D translation = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         double expectedDistanceSquared = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         translation.scale(Math.sqrt(expectedDistanceSquared));
         T p = createTuple(translation.getX(), translation.getY());
         double actualDistance = p.distanceFromOriginSquared();
         assertEquals(expectedDistanceSquared, actualDistance, getEpsilon());
      }
   }

   // Basics part
   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomTuple(random);
         T actual = createEmptyTuple();
         T expected = createEmptyTuple();

         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         rigidBodyTransform.setTranslation(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         rigidBodyTransform.transform(expected);
         actual.set(original);
         actual.applyTransform(rigidBodyTransform);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         try
         {
            actual.applyTransform(rigidBodyTransform);
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         catch (NotAMatrix2DException | NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }

         try
         {
            actual.applyTransform(rigidBodyTransform, true);
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         catch (NotAMatrix2DException | NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         actual.applyTransform(rigidBodyTransform, false);
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T original = createRandomTuple(random);
         T actual = createEmptyTuple();
         T expected = createEmptyTuple();

         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setRotationYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         rigidBodyTransform.setTranslation(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         actual.set(original);
         actual.applyTransform(rigidBodyTransform);
         actual.applyInverseTransform(rigidBodyTransform);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, false);
         actual.applyInverseTransform(rigidBodyTransform, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, true);
         actual.applyInverseTransform(rigidBodyTransform, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         try
         {
            actual.applyInverseTransform(rigidBodyTransform);
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         catch (NotAMatrix2DException | NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }

         try
         {
            actual.applyInverseTransform(rigidBodyTransform, true);
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         catch (NotAMatrix2DException | NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException or NotAnOrientation2DException.");
         }
         actual.applyInverseTransform(rigidBodyTransform, false);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Point2DBasics pointA;
      Point2DBasics pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         pointA = EuclidCoreRandomTools.nextPoint2D(random);
         pointB = EuclidCoreRandomTools.nextPoint2D(random);

         if (pointA.epsilonEquals(pointB, getEpsilon()))
         {
            assertTrue(pointA.geometricallyEquals(pointB, Math.sqrt(3) * getEpsilon()));
         }
         else
         {
            if (Math.sqrt(EuclidCoreTools.normSquared(pointA.getX() - pointB.getX(), pointA.getY() - pointB.getY())) <= getEpsilon())
            {
               assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
            else
            {
               assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
         }

         pointA = EuclidCoreRandomTools.nextPoint2D(random);
         pointB = new Point2D(pointA);

         assertTrue(pointA.geometricallyEquals(pointB, 0));

         pointB.set(pointA.getX() + 0.9d * getEpsilon(), pointA.getY());

         assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));

         pointB.set(pointA.getX() + 1.1d * getEpsilon(), pointA.getY());

         assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));

         pointB.set(pointA.getX(), pointA.getY() + 0.9d * getEpsilon());

         assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));

         pointB.set(pointA.getX(), pointA.getY() + 1.1d * getEpsilon());

         assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
      }
   }
}

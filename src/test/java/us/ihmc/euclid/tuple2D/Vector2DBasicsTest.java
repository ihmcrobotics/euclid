package us.ihmc.euclid.tuple2D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public abstract class Vector2DBasicsTest<T extends Vector2DBasics> extends Tuple2DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = EuclidCoreTools.cos(expectedAngle) * vector1.getX() - EuclidCoreTools.sin(expectedAngle) * vector1.getY();
         double y = EuclidCoreTools.sin(expectedAngle) * vector1.getX() + EuclidCoreTools.cos(expectedAngle) * vector1.getY();
         T vector2 = createTuple(x, y);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         double actualAngle = vector1.angle(vector2);

         assertEquals(expectedAngle, actualAngle, getEpsilon());
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Vector2DReadOnly other)
         T vector1 = createRandomTuple(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = EuclidCoreTools.cos(angle) * vector1.getX() - EuclidCoreTools.sin(angle) * vector1.getY();
         double y = EuclidCoreTools.sin(angle) * vector1.getX() + EuclidCoreTools.cos(angle) * vector1.getY();
         T vector2 = createTuple(x, y);

         double expectedDot = vector1.norm() * vector2.norm() * EuclidCoreTools.sin(angle);
         double actualDot = vector1.cross(vector2);
         assertEquals(expectedDot, actualDot, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Vector2DReadOnly v1, Vector2DReadOnly v2)
         T vector1 = createRandomTuple(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = EuclidCoreTools.cos(angle) * vector1.getX() - EuclidCoreTools.sin(angle) * vector1.getY();
         double y = EuclidCoreTools.sin(angle) * vector1.getX() + EuclidCoreTools.cos(angle) * vector1.getY();
         T vector2 = createTuple(x, y);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         double expectedCross = vector1.norm() * vector2.norm() * EuclidCoreTools.sin(angle);
         double actualCross = Vector2DReadOnly.cross(vector1, vector2);
         assertEquals(expectedCross, actualCross, getEpsilon());
      }
   }

   // Basics part
   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test normalize()
         T vector1 = createRandomTuple(random);
         vector1.normalize();

         double expectedLength = 1.0;
         double actualLength = vector1.norm();
         assertEquals(expectedLength, actualLength, getEpsilon());

         T vector2 = createRandomTuple(random);
         vector2.normalize();
         vector1.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), vector2);
         vector1.normalize();
         EuclidCoreTestTools.assertEquals(vector1, vector2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test normalize(Vector2D vector)
         T vector1 = createRandomTuple(random);
         T vector2 = createEmptyTuple();

         vector2.setAndNormalize(vector1);

         double expectedLength = 1.0;
         double actualLength = vector2.norm();
         assertEquals(expectedLength, actualLength, getEpsilon());

         vector2 = createRandomTuple(random);
         vector2.normalize();
         T vector3 = createEmptyTuple();
         vector3.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), vector2);
         vector1.setAndNormalize(vector3);
         EuclidCoreTestTools.assertEquals(vector1, vector2, getEpsilon());
      }
   }

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
         rigidBodyTransform.getRotation().setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         rigidBodyTransform.getTranslation().set(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         rigidBodyTransform.transform(expected);
         actual.set(original);
         actual.applyTransform(rigidBodyTransform);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, false);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, true);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         try
         {
            actual.applyTransform(rigidBodyTransform);
            fail("Should have thrown a NotAMatrix2DException.");
         }
         catch (NotAMatrix2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException.");
         }

         try
         {
            actual.applyTransform(rigidBodyTransform, true);
            fail("Should have thrown a NotAMatrix2DException.");
         }
         catch (NotAMatrix2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAMatrix2DException.");
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
         rigidBodyTransform.getRotation().setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         rigidBodyTransform.getTranslation().set(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         actual.set(original);
         actual.applyTransform(rigidBodyTransform);
         actual.applyInverseTransform(rigidBodyTransform);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, false);
         actual.applyInverseTransform(rigidBodyTransform, false);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         actual.set(original);
         actual.applyTransform(rigidBodyTransform, true);
         actual.applyInverseTransform(rigidBodyTransform, true);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());

         rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         try
         {
            actual.applyInverseTransform(rigidBodyTransform);
            fail("Should have thrown a NotAnOrientation2DException.");
         }
         catch (NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAnOrientation2DException.");
         }

         try
         {
            actual.applyInverseTransform(rigidBodyTransform, true);
            fail("Should have thrown a NotAnOrientation2DException.");
         }
         catch (NotAnOrientation2DException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a NotAnOrientation2DException.");
         }
         actual.applyInverseTransform(rigidBodyTransform, false);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Vector2DBasics vectorA;
      Vector2DBasics vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector2D(random);
         vectorB = EuclidCoreRandomTools.nextVector2D(random);

         if (vectorA.epsilonEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, EuclidCoreTools.squareRoot(3) * getEpsilon()));
         }
         else
         {
            if (EuclidCoreTools.norm(vectorA.getX() - vectorB.getX(), vectorA.getY() - vectorB.getY()) <= getEpsilon())
            {
               assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
            else
            {
               assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
         }

         vectorA = EuclidCoreRandomTools.nextVector2D(random);
         vectorB = new Vector2D(vectorA);

         assertTrue(vectorA.geometricallyEquals(vectorB, 0));

         vectorB.set(vectorA.getX() + 0.9d * getEpsilon(), vectorA.getY());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX() + 1.1d * getEpsilon(), vectorA.getY());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 0.9d * getEpsilon());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 1.1d * getEpsilon());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
      }
   }
}

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
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public abstract class Vector2DBasicsTest<T extends Vector2DBasics> extends Tuple2DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testLength()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.length();
         assertEquals(expectedLength2, actualLength2, 2.0 * getEpsilon());
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.lengthSquared();
         assertEquals(expectedLength2, EuclidCoreTools.squareRoot(actualLength2), 2.0 * getEpsilon());
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = Math.cos(angle) * vector1.getX() - Math.sin(angle) * vector1.getY();
         double y = Math.sin(angle) * vector1.getX() + Math.cos(angle) * vector1.getY();
         T vector2 = createTuple(x, y);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         double expectedDot = vector1.length() * vector2.length() * Math.cos(angle);
         double actualDot = vector1.dot(vector2);
         assertEquals(expectedDot, actualDot, getEpsilon());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = Math.cos(expectedAngle) * vector1.getX() - Math.sin(expectedAngle) * vector1.getY();
         double y = Math.sin(expectedAngle) * vector1.getX() + Math.cos(expectedAngle) * vector1.getY();
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
         double x = Math.cos(angle) * vector1.getX() - Math.sin(angle) * vector1.getY();
         double y = Math.sin(angle) * vector1.getX() + Math.cos(angle) * vector1.getY();
         T vector2 = createTuple(x, y);

         double expectedDot = vector1.length() * vector2.length() * Math.sin(angle);
         double actualDot = vector1.cross(vector2);
         assertEquals(expectedDot, actualDot, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Vector2DReadOnly v1, Vector2DReadOnly v2)
         T vector1 = createRandomTuple(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double x = Math.cos(angle) * vector1.getX() - Math.sin(angle) * vector1.getY();
         double y = Math.sin(angle) * vector1.getX() + Math.cos(angle) * vector1.getY();
         T vector2 = createTuple(x, y);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         double expectedCross = vector1.length() * vector2.length() * Math.sin(angle);
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
         double actualLength = vector1.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         T vector2 = createRandomTuple(random);
         vector2.normalize();
         vector1.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), vector2);
         vector1.normalize();
         EuclidCoreTestTools.assertTuple2DEquals(vector1, vector2, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test normalize(Vector2D vector)
         T vector1 = createRandomTuple(random);
         T vector2 = createEmptyTuple();

         vector2.setAndNormalize(vector1);

         double expectedLength = 1.0;
         double actualLength = vector2.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         vector2 = createRandomTuple(random);
         vector2.normalize();
         T vector3 = createEmptyTuple();
         vector3.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), vector2);
         vector1.setAndNormalize(vector3);
         EuclidCoreTestTools.assertTuple2DEquals(vector1, vector2, getEpsilon());
      }
   }

   @Test
   public void testClipToMaxLength() throws Exception
   {
      Random random = new Random(234234);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with maxLength > EPS_MAX_LENGTH
         double maxLength = EuclidCoreRandomTools.nextDouble(random, Vector3DBasics.EPS_MAX_LENGTH, 10.0);
         double vectorLength = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T expectedVector = createTuple(1.0, 0.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         RotationMatrixTools.applyYawRotation(yaw, expectedVector, expectedVector);
         T actualVector = createEmptyTuple();
         actualVector.setAndScale(vectorLength, expectedVector);

         if (maxLength > vectorLength)
         {
            expectedVector.scale(vectorLength);
            assertFalse(actualVector.clipToMaxLength(maxLength));
         }
         else
         {
            expectedVector.scale(maxLength);
            assertTrue(actualVector.clipToMaxLength(maxLength));
         }

         EuclidCoreTestTools.assertTuple2DEquals("Iteration: " + i + ", maxLength: " + maxLength, expectedVector, actualVector, 5.0 * getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with maxLength < EPS_MAX_LENGTH
         double maxLength = EuclidCoreRandomTools.nextDouble(random, 0.0, Vector3DBasics.EPS_MAX_LENGTH);
         double vectorLength = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         T actualVector = createTuple(vectorLength, 0.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         RotationMatrixTools.applyYawRotation(yaw, actualVector, actualVector);

         assertTrue(actualVector.clipToMaxLength(maxLength));

         EuclidCoreTestTools.assertTuple2DIsSetToZero("Iteration: " + i + ", maxLength: " + maxLength, actualVector);
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

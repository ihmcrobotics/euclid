package us.ihmc.euclid.tuple3D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public abstract class Vector3DBasicsTest<T extends Vector3DBasics> extends Tuple3DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.nextDouble(random, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, vector1, true);
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, 0.01, Math.PI - 0.01);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, expectedAngle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         double actualAngle = vector1.angle(vector2);
         assertEquals(Math.abs(expectedAngle), actualAngle, getEpsilon());
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, vector1, true);
         double angle = EuclidCoreRandomTools.nextDouble(random, 0.01, Math.PI - 0.01);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.normalize();
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 2.0));

         T vector3 = createEmptyTuple();
         vector3.cross(vector1, vector2);

         double expectedCrossMagnitude = vector1.norm() * vector2.norm() * EuclidCoreTools.sin(angle);
         double actualCrossMagnitude = vector3.norm();
         assertEquals(expectedCrossMagnitude, actualCrossMagnitude, 10.0 * getEpsilon());

         assertEquals(0.0, vector1.dot(vector3), 10.0 * getEpsilon());
         assertEquals(0.0, vector2.dot(vector3), 10.0 * getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // cross(Tuple3DReadOnly other)
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.nextDouble(random, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, vector1, true);
         double angle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));

         T expectedVector = createEmptyTuple();
         expectedVector.cross(vector1, vector2);
         T actualVector = createEmptyTuple();
         actualVector.set(vector1);
         actualVector.cross(vector2);

         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, getEpsilon());
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

         vector1.setToNaN();
         vector1.normalize();
         for (int index = 0; index < 3; index++)
            assertTrue(Double.isNaN(vector1.getElement(index)));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setAndNormalize(Tuple3DReadOnly other)
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

         vector3.setToNaN();
         vector1.setToZero();
         vector1.setAndNormalize(vector3);
         for (int index = 0; index < 3; index++)
            assertTrue(Double.isNaN(vector1.getElement(index)));
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, 10.0 * getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, 10.0 * getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertEquals(expected, actual, 100.0 * getEpsilon());
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Vector3DBasics vectorA;
      Vector3DBasics vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector3D(random);
         vectorB = EuclidCoreRandomTools.nextVector3D(random);

         if (vectorA.epsilonEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, EuclidCoreTools.squareRoot(3) * getEpsilon()));
         }
         else
         {
            if (EuclidCoreTools.norm(vectorA.getX() - vectorB.getX(), vectorA.getY() - vectorB.getY(), vectorA.getZ() - vectorB.getZ()) <= getEpsilon())
            {
               assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
            else
            {
               assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
         }

         vectorA = EuclidCoreRandomTools.nextVector3D(random);
         vectorB = new Vector3D(vectorA);

         assertTrue(vectorA.geometricallyEquals(vectorB, 0));

         vectorB.set(vectorA.getX() + 0.9d * getEpsilon(), vectorA.getY(), vectorA.getZ());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX() + 1.1d * getEpsilon(), vectorA.getY(), vectorA.getZ());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 0.9d * getEpsilon(), vectorA.getZ());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 1.1d * getEpsilon(), vectorA.getZ());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 0.9d * getEpsilon());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 1.1d * getEpsilon());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
      }
   }
}

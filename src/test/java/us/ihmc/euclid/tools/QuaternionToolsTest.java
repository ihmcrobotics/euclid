package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;

public class QuaternionToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(394865L);
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      // Test that q times the neutral quaternion equals q
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qNeutral = new Quaternion();
         Quaternion qNeutralCopy = new Quaternion();

         qExpected.set(q);
         QuaternionTools.multiply(q, qNeutral, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         assertTrue(qNeutral.equals(qNeutralCopy));
         QuaternionTools.multiply(qNeutral, q, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Test that q * q^-1 = qNeutral
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);

         Quaternion qInv = new Quaternion(q);
         qInv.conjugate();
         qExpected.setToZero();
         QuaternionTools.multiply(q, qInv, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         QuaternionTools.multiply(qInv, q, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Simplify the multiplication by making q1 and q2 describe rotation of different angle but around the same axis.
      // So the multiplication basically adds up the two angles and is also commutable.
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double angle2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Quaternion q1 = new Quaternion();
         Quaternion q2 = new Quaternion();

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle1, q1);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle2, q2);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle1 + angle2, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q1, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q2, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);

         QuaternionTools.multiply(q1, q2, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);

         QuaternionTools.multiply(q2, q1, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Check that we can do in-place multiplication
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle, qActual);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), 2.0 * angle, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qActual, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);

         QuaternionTools.multiply(qActual, qActual, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }
   }

   @Test
   public void testMultiplyConjugateLeft() throws Exception
   {
      Random random = new Random(394865L);
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      // Test that: conj(qNeutral) * q = q and that: conj(q) * qNeutral = conj(q)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qNeutral = new Quaternion();
         Quaternion qNeutralCopy = new Quaternion();

         qExpected.set(q);

         QuaternionTools.multiplyConjugateLeft(qNeutral, q, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         assertTrue(qNeutral.equals(qNeutralCopy));

         qExpected.conjugate();
         QuaternionTools.multiplyConjugateLeft(q, qNeutral, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Test that conj(q^-1) * q = q * q and that conj(q) * q^-1 = (q * q)^-1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = new Quaternion();
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle, q);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), 2.0 * angle, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);

         Quaternion qInv = new Quaternion(q);
         qInv.conjugate();

         QuaternionTools.multiplyConjugateLeft(qInv, q, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         QuaternionTools.multiplyConjugateLeft(q, qInv, qActual);

         qExpected.conjugate();
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Simplify the multiplication by making q1 and q2 describe rotation of different angle but around the same axis.
      // So the multiplication (with left term conjugated) basically subtracts up the two angles.
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double angle2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Quaternion q1 = new Quaternion();
         Quaternion q2 = new Quaternion();

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle1, q1);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle2, q2);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle2 - angle1, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q1, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q2, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);

         QuaternionTools.multiplyConjugateLeft(q1, q2, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);

         qExpected.conjugate();
         QuaternionTools.multiplyConjugateLeft(q2, q1, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Check that we can do in-place multiplication, so we test that: conj(q) * q = qNeutral
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle, qActual);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qActual, EPSILON);
         qExpected.setToZero();

         QuaternionTools.multiplyConjugateLeft(qActual, qActual, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }
   }

   @Test
   public void testMultiplyConjugateRight() throws Exception
   {
      Random random = new Random(394865L);
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      // Test that: qNeutral * conj(q) = conj(q) and that: q * conj(qNeutral) = q
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qNeutral = new Quaternion();
         Quaternion qNeutralCopy = new Quaternion();

         qExpected.set(q);

         QuaternionTools.multiplyConjugateRight(q, qNeutral, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         assertTrue(qNeutral.equals(qNeutralCopy));

         qExpected.conjugate();
         QuaternionTools.multiplyConjugateRight(qNeutral, q, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Test that q^-1 * conj(q) = (q * q)^-1 and that q * conj(q^-1) = q * q
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = new Quaternion();
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle, q);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), 2.0 * angle, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);
         // Fill some random data in qActual
         qActual = EuclidCoreRandomTools.nextQuaternion(random);

         Quaternion qInv = new Quaternion(q);
         qInv.conjugate();

         QuaternionTools.multiplyConjugateRight(q, qInv, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
         QuaternionTools.multiplyConjugateRight(qInv, q, qActual);
         qExpected.conjugate();
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Simplify the multiplication by making q1 and q2 describe rotation of different angle but around the same axis.
      // So the multiplication (with right term conjugated) basically subtracts up the two angles.
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double angle2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Quaternion q1 = new Quaternion();
         Quaternion q2 = new Quaternion();

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle1, q1);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle2, q2);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle1 - angle2, qExpected);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q1, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(q2, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qExpected, EPSILON);

         QuaternionTools.multiplyConjugateRight(q1, q2, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);

         qExpected.conjugate();
         QuaternionTools.multiplyConjugateRight(q2, q1, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Check that we can do in-place multiplication, so we test that: q * conj(q) = qNeutral
      for (int i = 0; i < ITERATIONS; i++)
      {
         double angle = EuclidCoreRandomTools.nextDouble(random, 1.0);
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle, qActual);
         EuclidCoreTestTools.assertQuaternionIsUnitary(qActual, EPSILON);
         qExpected.setToZero();

         QuaternionTools.multiplyConjugateRight(qActual, qActual, qActual);
         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }
   }

   @Test
   public void testMultiplyWithVector4D() throws Exception
   {
      Random random = new Random(394865L);
      Vector4D vectorExpected = new Vector4D();
      Vector4D vectorActual = new Vector4D();

      // multiply(Tuple4DReadOnly q, Tuple4DReadOnly v, Vector4DBasics vectorToPack)
      // Test against the multiplication with quaternions
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q1 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion q2 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qResult = new Quaternion();
         QuaternionTools.multiply(q1, q2, qResult);
         vectorExpected.set(qResult);

         QuaternionTools.multiply(q1, q2, vectorActual);
         if (vectorActual.dot(vectorExpected) < 0.0)
            vectorActual.negate();
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);
      }

      // multiplyConjugateLeft(Tuple4DReadOnly q, Tuple4DReadOnly v, Vector4DBasics vectorToPack)
      // Test against the multiplication with quaternions
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q1 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion q2 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qResult = new Quaternion();
         QuaternionTools.multiplyConjugateLeft(q1, q2, qResult);
         vectorExpected.set(qResult);

         QuaternionTools.multiplyConjugateLeft(q1, q2, vectorActual);
         if (vectorActual.dot(vectorExpected) < 0.0)
            vectorActual.negate();
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);
      }

      // multiplyConjugateRight(Tuple4DReadOnly q, Tuple4DReadOnly v, Vector4DBasics vectorToPack)
      // Test against the multiplication with quaternions
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q1 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion q2 = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion qResult = new Quaternion();
         QuaternionTools.multiplyConjugateRight(q1, q2, qResult);
         vectorExpected.set(qResult);

         QuaternionTools.multiplyConjugateRight(q1, q2, vectorActual);
         if (vectorActual.dot(vectorExpected) < 0.0)
            vectorActual.negate();
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testNormalizeAndLimitToPi() throws Exception
   {
      Random random = new Random(3294862L);
      Quaternion qExpected = new Quaternion();
      Quaternion qActual = new Quaternion();

      // Test that in the range [-Pi, Pi] normalizeAndLimitToMinusPiToPi() is the same as normalize().
      for (int i = 0; i < 10 * ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, Math.PI);
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, qExpected);
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);

         qExpected.setUnsafe(qExpected.getX() * scale, qExpected.getY() * scale, qExpected.getZ() * scale, qExpected.getS() * scale);
         qActual.set(qExpected);

         qExpected.normalize();
         qActual.normalizeAndLimitToPi();

         EuclidCoreTestTools.assertQuaternionEquals(qExpected, qActual, EPSILON);
      }

      // Test that outside the range [-Pi, Pi] normalizedAndLimitToMinusPiToPi actually limits the angle described by the quaternion to interval [-Pi, Pi].
      for (int i = 0; i < 10 * ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         double sign = random.nextBoolean() ? -1.0 : 1.0;
         axisAngle.setAngle(sign * EuclidCoreRandomTools.nextDouble(random, Math.PI + EPSILON, 2.0 * Math.PI));
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, qExpected);

         assertTrue(qExpected.getS() < 0.0);

         double scale = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);

         qExpected.setUnsafe(qExpected.getX() * scale, qExpected.getY() * scale, qExpected.getZ() * scale, qExpected.getS() * scale);
         qActual.set(qExpected);

         qExpected.normalize();
         qActual.normalizeAndLimitToPi();

         assertFalse(qExpected.epsilonEquals(qActual, EPSILON));
         assertTrue(qExpected.dot(qActual) < 0.0);
         assertTrue(qActual.getS() > 0.0);
         AxisAngleConversion.convertQuaternionToAxisAngle(qActual, axisAngle);
         assertTrue(axisAngle.getAngle() < Math.PI && axisAngle.getAngle() > -Math.PI);
         AxisAngleConversion.convertQuaternionToAxisAngle(qExpected, axisAngle);
         assertTrue(axisAngle.getAngle() > Math.PI || axisAngle.getAngle() < -Math.PI);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(qExpected, qActual, EPSILON);
      }
   }

   @Test
   public void testTransformATuple() throws Exception
   {
      Random random = new Random(4536L);
      Quaternion quaternion = new Quaternion();
      Vector3D tupleExpected = new Vector3D();
      Vector3D tupleActual = new Vector3D();
      Vector3D tupleOriginal = new Vector3D();

      // Test that qNeutral does not change the tuple
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      quaternion.setToZero();
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      // Test trivial cases
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      quaternion.set(0.0, 0.0, 0.0, -1.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setY(-tupleExpected.getY());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(1.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setY(-tupleExpected.getY());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(-1.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(0.0, 1.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(0.0, -1.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.set(0.0, 0.0, 1.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.set(0.0, 0.0, -1.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      // Test against a second way of transforming, by using multiply with a pure quaternion
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         tupleOriginal = EuclidCoreRandomTools.nextRotationVector(random);
         tupleActual.set(tupleOriginal);

         Vector4D pureQuaternion = new Vector4D();
         pureQuaternion.set(tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ(), 0.0);

         QuaternionTools.multiply(quaternion, pureQuaternion, pureQuaternion);
         QuaternionTools.multiplyConjugateRight(pureQuaternion, quaternion, pureQuaternion);
         tupleExpected.setX(pureQuaternion.getX());
         tupleExpected.setY(pureQuaternion.getY());
         tupleExpected.setZ(pureQuaternion.getZ());
         QuaternionTools.transform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
         QuaternionTools.transform(quaternion, tupleOriginal, tupleActual);
      }

      // Test that the quaternion values are normalized before transforming
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.setUnsafe(10.0, 10.0, 10.0, 10.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual);
      assertEquals(tupleExpected.length(), tupleActual.length(), EPSILON);
   }

   @Test
   public void testInverseTransformATuple() throws Exception
   {
      Random random = new Random(4536L);
      Quaternion quaternion = new Quaternion();
      Vector3D tupleExpected = new Vector3D();
      Vector3D tupleActual = new Vector3D();
      Vector3D tupleOriginal = new Vector3D();

      // Test that qNeutral does not change the tuple
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      quaternion.setToZero();
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      // Test trivial cases
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      quaternion.set(0.0, 0.0, 0.0, -1.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setY(-tupleExpected.getY());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(1.0, 0.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setY(-tupleExpected.getY());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(-1.0, 0.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(0.0, 1.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setZ(-tupleExpected.getZ());
      quaternion.set(0.0, -1.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.set(0.0, 0.0, 1.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.set(0.0, 0.0, -1.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

      // Test against a second way of transforming, by using multiply with a pure quaternion
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         tupleOriginal = EuclidCoreRandomTools.nextRotationVector(random);
         tupleActual.set(tupleOriginal);

         Vector4D pureQuaternion = new Vector4D();
         pureQuaternion.set(tupleOriginal.getX(), tupleOriginal.getY(), tupleOriginal.getZ(), 0.0);

         QuaternionTools.multiplyConjugateLeft(quaternion, pureQuaternion, pureQuaternion);
         QuaternionTools.multiply(pureQuaternion, quaternion, pureQuaternion);
         tupleExpected.setX(pureQuaternion.getX());
         tupleExpected.setY(pureQuaternion.getY());
         tupleExpected.setZ(pureQuaternion.getZ());
         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
         QuaternionTools.inverseTransform(quaternion, tupleOriginal, tupleActual);
      }

      // Test that inverseTransform(transform(tuple)) == tuple and transform(inverseTransform(tuple)) == tuple
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
         tupleActual.set(tupleExpected);

         QuaternionTools.transform(quaternion, tupleActual, tupleActual);
         assertFalse(TupleTools.epsilonEquals(tupleExpected, tupleActual, EPSILON));
         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
         assertFalse(TupleTools.epsilonEquals(tupleExpected, tupleActual, EPSILON));
         QuaternionTools.transform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }

      // Test that the quaternion values are normalized before transforming
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      tupleExpected.setX(-tupleExpected.getX());
      tupleExpected.setY(-tupleExpected.getY());
      quaternion.setUnsafe(10.0, 10.0, 10.0, 10.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual);
      assertEquals(tupleExpected.length(), tupleActual.length(), EPSILON);

      // Test that a quaternion with zeros does not do anything
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleExpected, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      QuaternionTools.transform(quaternion, tupleExpected, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
   }

   @Test
   public void testAddTransformATuple() throws Exception
   {
      Random random = new Random(28346L);
      Tuple3DBasics tupleExpected = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();

      // Test that addTransform(tupleOriginal, tupleTransformed) == tupleTransformed + transform(tupleOriginal)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationVector(random);
         tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
         tupleActual.set(tupleExpected);
         Tuple3DBasics tupleTransformed = new Vector3D();
         QuaternionTools.transform(quaternion, tupleOriginal, tupleTransformed);
         tupleExpected.add(tupleTransformed);

         double corrupt = random.nextDouble() + 0.5;
         double qx = corrupt * quaternion.getX();
         double qy = corrupt * quaternion.getY();
         double qz = corrupt * quaternion.getZ();
         double qs = corrupt * quaternion.getS();
         quaternion.setUnsafe(qx, qy, qz, qs);

         QuaternionTools.addTransform(quaternion, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleActual);

         QuaternionTools.addTransform(quaternion, tupleActual, tupleExpected);
         QuaternionTools.addTransform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }

      // Test that a quaternion with zeros does not do anything
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.addTransform(quaternion, tupleExpected, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
   }

   @Test
   public void testSubTransformATuple() throws Exception
   {
      Random random = new Random(28346L);
      Tuple3DBasics tupleExpected = new Vector3D();
      Tuple3DBasics tupleActual = new Vector3D();

      // Test that addTransform(tupleOriginal, tupleTransformed) == tupleTransformed + transform(tupleOriginal)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationVector(random);
         tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
         tupleActual.set(tupleExpected);
         Tuple3DBasics tupleTransformed = new Vector3D();
         QuaternionTools.transform(quaternion, tupleOriginal, tupleTransformed);
         tupleExpected.sub(tupleTransformed);

         double corrupt = random.nextDouble() + 0.5;
         double qx = corrupt * quaternion.getX();
         double qy = corrupt * quaternion.getY();
         double qz = corrupt * quaternion.getZ();
         double qs = corrupt * quaternion.getS();
         quaternion.setUnsafe(qx, qy, qz, qs);

         QuaternionTools.subTransform(quaternion, tupleOriginal, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }

      // Test transforming in-place
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         tupleActual = EuclidCoreRandomTools.nextVector3D(random);
         tupleExpected.set(tupleActual);

         QuaternionTools.subTransform(quaternion, tupleActual, tupleExpected);
         QuaternionTools.subTransform(quaternion, tupleActual, tupleActual);
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }

      // Test that a quaternion with zeros does not do anything
      tupleExpected = EuclidCoreRandomTools.nextRotationVector(random);
      tupleActual.set(tupleExpected);
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.subTransform(quaternion, tupleExpected, tupleActual);
      EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
   }

   @Test
   public void testTransformATuple2D() throws Exception
   {
      Random random = new Random(2356L);
      Quaternion quaternion = new Quaternion();
      Tuple2DReadOnly tupleOriginal = new Vector2D();
      Tuple2DBasics tupleExpected = new Vector2D();
      Tuple2DBasics tupleActual = new Vector2D();

      // Test the exception is thrown when checkIfTransformInXYPlane and the transform is not in XY plane
      quaternion.set(1.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual, false);
      try
      {
         QuaternionTools.transform(quaternion, tupleActual, tupleActual, true);
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
      QuaternionTools.transform(quaternion, tupleOriginal, tupleActual, false);
      try
      {
         QuaternionTools.transform(quaternion, tupleOriginal, tupleActual, true);
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
      quaternion.set(0.0, 1.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleActual, tupleActual, false);
      try
      {
         QuaternionTools.transform(quaternion, tupleActual, tupleActual, true);
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
      QuaternionTools.transform(quaternion, tupleOriginal, tupleActual, false);
      try
      {
         QuaternionTools.transform(quaternion, tupleOriginal, tupleActual, true);
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

      for (int i = 0; i < ITERATIONS; i++)
      {
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DReadOnly tupleOriginalCopy = new Vector2D(tupleOriginal);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         quaternion.setToZero();
         double corrupt = random.nextDouble() + 0.5;
         quaternion.setUnsafe(0.0, 0.0, Math.sin(yaw / 2.0) * corrupt, Math.cos(yaw / 2.0) * corrupt);

         tupleExpected.setX(Math.cos(yaw) * tupleOriginal.getX() - Math.sin(yaw) * tupleOriginal.getY());
         tupleExpected.setY(Math.sin(yaw) * tupleOriginal.getX() + Math.cos(yaw) * tupleOriginal.getY());

         tupleActual.set(tupleOriginal);
         QuaternionTools.transform(quaternion, tupleActual, tupleActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPSILON);
         tupleActual.setToZero();
         QuaternionTools.transform(quaternion, tupleOriginal, tupleActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPSILON);
         assertTrue(tupleOriginal.equals(tupleOriginalCopy));
      }

      // Test that a quaternion with zeros does not do anything
      tupleExpected = EuclidCoreRandomTools.nextVector2D(random);
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, tupleExpected, tupleActual, false);
      EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPSILON);
   }

   @Test
   public void testInverseTransformATuple2D() throws Exception
   {
      Random random = new Random(2356L);
      Quaternion quaternion = new Quaternion();
      Tuple2DReadOnly tupleOriginal = new Vector2D();
      Tuple2DBasics tupleExpected = new Vector2D();
      Tuple2DBasics tupleActual = new Vector2D();

      // Test the exception is thrown when checkIfTransformInXYPlane and the transform is not in XY plane
      quaternion.set(1.0, 0.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual, false);
      try
      {
         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual, true);
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
      QuaternionTools.inverseTransform(quaternion, tupleOriginal, tupleActual, false);
      try
      {
         QuaternionTools.inverseTransform(quaternion, tupleOriginal, tupleActual, true);
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
      quaternion.set(0.0, 1.0, 0.0, 0.0);
      QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual, false);
      try
      {
         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual, true);
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
      QuaternionTools.inverseTransform(quaternion, tupleOriginal, tupleActual, false);
      try
      {
         QuaternionTools.inverseTransform(quaternion, tupleOriginal, tupleActual, true);
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

      // Simply check that inverseTransform(transform(tuple2D)) == tuple2D and that transform(inverseTransform(tuple2D)) == tuple2D
      for (int i = 0; i < ITERATIONS; i++)
      {
         tupleOriginal = EuclidCoreRandomTools.nextVector2D(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         quaternion.setToZero();
         quaternion.set(0.0, 0.0, Math.sin(yaw / 2.0), Math.cos(yaw / 2.0));

         tupleExpected.set(tupleOriginal);
         tupleActual.set(tupleOriginal);
         QuaternionTools.transform(quaternion, tupleActual, tupleActual, true);
         QuaternionTools.inverseTransform(quaternion, tupleActual, tupleActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPSILON);

         Tuple2DBasics tupleTransformed = new Vector2D();
         Tuple2DBasics tupleTransformedCopy = new Vector2D();
         tupleActual.setToZero();
         QuaternionTools.transform(quaternion, tupleOriginal, tupleTransformed, true);
         tupleTransformedCopy.set(tupleTransformed);
         assertFalse(TupleTools.epsilonEquals(tupleOriginal, tupleTransformed, EPSILON));
         QuaternionTools.inverseTransform(quaternion, tupleTransformed, tupleActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(tupleExpected, tupleActual, EPSILON);
         assertTrue(tupleTransformed.equals(tupleTransformedCopy));
      }
   }

   @Test
   public void testTransformAVector4D() throws Exception
   {
      Random random = new Random(35656L);
      Vector4D vectorOriginal = new Vector4D();
      Vector4D vectorActual = new Vector4D();
      Vector4D vectorExpected = new Vector4D();

      // Test against transform with Tuple and check that v = inverseTransform(transform(v))
      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector3D vector3D = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), vectorOriginal.getZ());
         QuaternionTools.transform(quaternion, vector3D, vector3D);
         vectorExpected.set(vector3D);
         vectorExpected.setS(vectorOriginal.getS());

         double corrupt = random.nextDouble() + 0.5;
         double qx = corrupt * quaternion.getX();
         double qy = corrupt * quaternion.getY();
         double qz = corrupt * quaternion.getZ();
         double qs = corrupt * quaternion.getS();
         quaternion.setUnsafe(qx, qy, qz, qs);

         QuaternionTools.transform(quaternion, vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);

         vectorActual.set(vectorOriginal);
         QuaternionTools.transform(quaternion, vectorActual, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);

         QuaternionTools.inverseTransform(quaternion, vectorActual, vectorActual);
         EuclidCoreTestTools.assertTuple4DEquals(vectorOriginal, vectorActual, EPSILON);
      }

      // Test that a quaternion with zeros does not do anything
      vectorExpected = EuclidCoreRandomTools.nextVector4D(random);
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, vectorExpected, vectorActual);
      EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPSILON);
   }

   @Test
   public void testTransformAQuaternion() throws Exception
   {
      Random random = new Random(2352L);
      Quaternion quaternion = new Quaternion();
      Quaternion quaternionOriginal = new Quaternion();
      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      // Test with the multiply: qTransformed = q * qOriginal and check that  q = inverseTransform(transform(q))
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         quaternionOriginal = EuclidCoreRandomTools.nextQuaternion(random);

         QuaternionTools.multiply(quaternion, quaternionOriginal, quaternionExpected);
         assertFalse(quaternionOriginal.epsilonEquals(quaternionExpected, EPSILON));

         quaternionActual.set(quaternionOriginal);
         QuaternionTools.transform(quaternion, quaternionActual, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);

         quaternionActual.setToNaN();
         QuaternionTools.transform(quaternion, quaternionOriginal, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);

         QuaternionTools.inverseTransform(quaternion, quaternionActual, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionOriginal, quaternionActual, EPSILON);
      }
   }

   @Test
   public void testTransformAMatrix() throws Exception
   {
      Random random = new Random(2352L);
      Quaternion quaternion = new Quaternion();
      RotationMatrix matrix = new RotationMatrix();

      Matrix3D matrixOriginal = new Matrix3D();
      Matrix3D matrixOriginalCopy = new Matrix3D();
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      // Test against the matrix multiplication: mTransformed = R * mOriginal * R^T
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix.set(quaternion);
         matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         matrixOriginalCopy.set(matrixOriginal);

         Matrix3DTools.multiply(matrix, matrixOriginal, matrixExpected);
         assertFalse(matrixExpected.epsilonEquals(matrixOriginal, EPSILON));
         Matrix3DTools.multiplyTransposeRight(matrixExpected, matrix, matrixExpected);
         assertFalse(matrixExpected.epsilonEquals(matrixOriginal, EPSILON));

         double corrupt = random.nextDouble() + 0.5;
         double qx = corrupt * quaternion.getX();
         double qy = corrupt * quaternion.getY();
         double qz = corrupt * quaternion.getZ();
         double qs = corrupt * quaternion.getS();
         quaternion.setUnsafe(qx, qy, qz, qs);

         matrixActual.set(matrixOriginal);
         QuaternionTools.transform(quaternion, matrixActual, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPSILON);

         matrixActual.setToNaN();
         QuaternionTools.transform(quaternion, matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPSILON);
         assertTrue(matrixOriginal.equals(matrixOriginalCopy));

         QuaternionTools.inverseTransform(quaternion, matrixActual, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, EPSILON);
      }

      // Test that a quaternion with zeros does not do anything
      matrixExpected = EuclidCoreRandomTools.nextMatrix3D(random);
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      QuaternionTools.transform(quaternion, matrixExpected, matrixActual);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPSILON);
   }

   @Test
   public void testTransformARotationMatrix() throws Exception
   {
      Random random = new Random(345L);
      Quaternion quaternion = new Quaternion();
      RotationMatrix matrix = new RotationMatrix();

      RotationMatrix matrixOriginal = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      // Test against the matrix multiplication: mTransformed = R * mOriginal
      for (int i = 0; i < ITERATIONS; i++)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix.set(quaternion);
         matrixOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);

         RotationMatrixTools.multiply(matrix, matrixOriginal, matrixExpected);
         assertFalse(matrixExpected.epsilonEquals(matrixOriginal, EPSILON));

         double corrupt = random.nextDouble() + 0.5;
         double qx = corrupt * quaternion.getX();
         double qy = corrupt * quaternion.getY();
         double qz = corrupt * quaternion.getZ();
         double qs = corrupt * quaternion.getS();
         quaternion.setUnsafe(qx, qy, qz, qs);

         matrixActual.set(matrixOriginal);
         QuaternionTools.transform(quaternion, matrixActual, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPSILON);

         matrixActual.setToNaN();
         QuaternionTools.transform(quaternion, matrixOriginal, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPSILON);

         QuaternionTools.inverseTransform(quaternion, matrixActual, matrixActual);
         EuclidCoreTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, EPSILON);
      }
   }

   @Test
   public void testMultiplyQuaternionMatrixResultPutInQuaternion() throws Exception
   {
      Random random = new Random(3466L);
      Quaternion quaternion = new Quaternion();
      RotationMatrix matrix = new RotationMatrix();

      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(QuaternionReadOnly quaternion, false, RotationMatrixReadOnly matrix, false, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix matrixCopy = new RotationMatrix(matrix);

         QuaternionTools.multiply(quaternion, new Quaternion(matrix), quaternionExpected);
         QuaternionTools.multiply(quaternion, false, matrix, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
         assertTrue(matrix.equals(matrixCopy));

         // Check that is works even the two quaternion arguments are the same object
         quaternionActual.set(quaternion);
         QuaternionTools.multiply(quaternionActual, false, matrix, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
         assertTrue(matrix.equals(matrixCopy));
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(QuaternionReadOnly quaternion, true, RotationMatrixReadOnly matrix, false, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion conjugate = new Quaternion(quaternion);
         conjugate.conjugate();
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);

         QuaternionTools.multiply(conjugate, new Quaternion(matrix), quaternionExpected);
         QuaternionTools.multiply(quaternion, true, matrix, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(QuaternionReadOnly quaternion, false, RotationMatrixReadOnly matrix, true, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix transposed = new RotationMatrix(matrix);
         transposed.transpose();

         QuaternionTools.multiply(quaternion, new Quaternion(transposed), quaternionExpected);
         QuaternionTools.multiply(quaternion, false, matrix, true, quaternionActual);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(quaternionExpected, quaternionActual, EPSILON);
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(QuaternionReadOnly quaternion, true, RotationMatrixReadOnly matrix, true, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion conjugate = new Quaternion(quaternion);
         conjugate.conjugate();
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix transposed = new RotationMatrix(matrix);
         transposed.transpose();

         QuaternionTools.multiply(conjugate, new Quaternion(transposed), quaternionExpected);
         QuaternionTools.multiply(quaternion, true, matrix, true, quaternionActual);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(quaternionExpected, quaternionActual, EPSILON);
      }
   }

   @Test
   public void testMultiplyMatrixQuaternionResultPutInQuaternion() throws Exception
   {
      Random random = new Random(3466L);
      Quaternion quaternion = new Quaternion();
      RotationMatrix matrix = new RotationMatrix();

      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(RotationMatrixReadOnly matrix, false, QuaternionReadOnly quaternion, false, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix matrixCopy = new RotationMatrix(matrix);

         QuaternionTools.multiply(new Quaternion(matrix), quaternion, quaternionExpected);
         QuaternionTools.multiply(matrix, false, quaternion, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
         assertTrue(matrix.equals(matrixCopy));

         // Check that is works even the two quaternion arguments are the same object
         quaternionActual.set(quaternion);
         QuaternionTools.multiply(matrix, false, quaternionActual, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
         assertTrue(matrix.equals(matrixCopy));
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(RotationMatrixReadOnly matrix, false, QuaternionReadOnly quaternion, true, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion conjugate = new Quaternion(quaternion);
         conjugate.conjugate();
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);

         QuaternionTools.multiply(new Quaternion(matrix), conjugate, quaternionExpected);
         QuaternionTools.multiply(matrix, false, quaternion, true, quaternionActual);
         EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPSILON);
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(RotationMatrixReadOnly matrix, true, QuaternionReadOnly quaternion, false, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix transposed = new RotationMatrix(matrix);
         transposed.transpose();

         QuaternionTools.multiply(new Quaternion(transposed), quaternion, quaternionExpected);
         QuaternionTools.multiply(matrix, true, quaternion, false, quaternionActual);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(quaternionExpected, quaternionActual, EPSILON);
      }

      // Simply test against the multiply(quaternion, quaternion, quaternion)
      for (int i = 0; i < ITERATIONS; i++)
      { // multiply(RotationMatrixReadOnly matrix, true, QuaternionReadOnly quaternion, true, QuaternionBasics quaternionToPack)
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion conjugate = new Quaternion(quaternion);
         conjugate.conjugate();
         matrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix transposed = new RotationMatrix(matrix);
         transposed.transpose();

         QuaternionTools.multiply(new Quaternion(transposed), conjugate, quaternionExpected);
         QuaternionTools.multiply(matrix, true, quaternion, true, quaternionActual);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(quaternionExpected, quaternionActual, EPSILON);
      }
   }
}

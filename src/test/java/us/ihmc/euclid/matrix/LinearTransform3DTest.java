package us.ihmc.euclid.matrix;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class LinearTransform3DTest extends Matrix3DBasicsTest<LinearTransform3D>
{
   private static final double SMALL_EPSILON = 1.0e-12;
   private static final double MID_EPSILON = 1.0e-9;
   private static final double LARGE_EPSILON = 1.0e-7;
   private static final int ITERATIONS = 1000;

   @Override
   public LinearTransform3D createEmptyMatrix()
   {
      return new LinearTransform3D();
   }

   @Override
   public LinearTransform3D createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return new LinearTransform3D(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   @Override
   public LinearTransform3D createRandomMatrix(Random random)
   {
      return EuclidCoreRandomTools.nextLinearTransform3D(random, 0.1, 5.0);
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(762834);

      { // Test LinearTransform3D()
         LinearTransform3D linearTransform3D = new LinearTransform3D();
         EuclidCoreTestTools.assertMatrix3DEquals(EuclidCoreTools.identityMatrix3D, linearTransform3D, 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getAsQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPreScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPostScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 1, 1), linearTransform3D.getScaleVector(), 0);
         assertTrue(linearTransform3D.isIdentity());
         assertTrue(linearTransform3D.isRotationMatrix());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test LinearTransform3D(Matrix3DReadOnly matrix3D)
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         LinearTransform3D linearTransform3D = new LinearTransform3D(matrix3D);
         EuclidCoreTestTools.assertMatrix3DEquals(matrix3D, linearTransform3D, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test LinearTransform3D(DMatrix matrix)
         DMatrix matrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 3, 3, 10.0);
         LinearTransform3D linearTransform3D = new LinearTransform3D(matrix);
         EuclidCoreTestTools.assertMatrix3DEquals(new Matrix3D(matrix), linearTransform3D, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test LinearTransform3D(Orientation3DReadOnly orientation)
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         LinearTransform3D linearTransform3D = new LinearTransform3D(orientation);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(orientation), linearTransform3D, SMALL_EPSILON);
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      super.testSetIdentity();

      Random random = new Random(76435);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LinearTransform3D linearTransform3D = EuclidCoreRandomTools.nextLinearTransform3D(random, 10.0);
         linearTransform3D.setIdentity();
         EuclidCoreTestTools.assertMatrix3DEquals(EuclidCoreTools.identityMatrix3D, linearTransform3D, 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getAsQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPreScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPostScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1, 1, 1), linearTransform3D.getScaleVector(), 0);
         assertTrue(linearTransform3D.isIdentity());
         assertTrue(linearTransform3D.isRotationMatrix());
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(76435);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LinearTransform3D linearTransform3D = EuclidCoreRandomTools.nextLinearTransform3D(random, 10.0);
         linearTransform3D.setToZero();
         EuclidCoreTestTools.assertMatrix3DEquals(EuclidCoreTools.zeroMatrix3D, linearTransform3D, 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getAsQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPreScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), linearTransform3D.getPostScaleQuaternion(), 0);
         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0, 0, 0), linearTransform3D.getScaleVector(), 0);
         assertFalse(linearTransform3D.isIdentity());
         assertFalse(linearTransform3D.isRotationMatrix());
      }
   }

   @Test
   public void testSetToNaN()
   {
      Random random = new Random(76435);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LinearTransform3D linearTransform3D = EuclidCoreRandomTools.nextLinearTransform3D(random, 10.0);
         linearTransform3D.setToNaN();
         EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(linearTransform3D);
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(linearTransform3D.getAsQuaternion());
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(linearTransform3D.getPreScaleQuaternion());
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(linearTransform3D.getPostScaleQuaternion());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(linearTransform3D.getScaleVector());
         assertFalse(linearTransform3D.isIdentity());
         assertFalse(linearTransform3D.isRotationMatrix());
      }
   }

   @Test
   public void testResetScale()
   {
      Random random = new Random(485725);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LinearTransform3D linearTransform3D = EuclidCoreRandomTools.nextLinearTransform3D(random, 10.0);
         Quaternion expectedOrientation = new Quaternion(linearTransform3D.getAsQuaternion());

         linearTransform3D.resetScale();
         assertEquals(new Vector3D(1, 1, 1), linearTransform3D.getScaleVector());
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedOrientation, linearTransform3D.getAsQuaternion(), SMALL_EPSILON);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedOrientation, linearTransform3D.getPreScaleQuaternion(), SMALL_EPSILON);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(new Quaternion(), linearTransform3D.getPostScaleQuaternion(), SMALL_EPSILON);
      }
   }

   @Test
   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Matrix3DReadOnly other)
         Matrix3DReadOnly expectedMatrix = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         LinearTransform3D actualMatrix = createEmptyMatrix();
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Matrix3DReadOnly other)
         LinearTransform3D expectedMatrix = EuclidCoreRandomTools.nextLinearTransform3D(random, 10.0);
         LinearTransform3D actualMatrix = createEmptyMatrix();
         actualMatrix.set(expectedMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // set(Orientation3D orientation3D)
         RotationMatrix expectedMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         LinearTransform3D actualMatrix = createEmptyMatrix();
         actualMatrix.set(new Quaternion(expectedMatrix));
         EuclidCoreTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, SMALL_EPS);
      }
   }

   @Test
   public void testSetRotationVector()
   {
      Random random = new Random(32546);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         RotationMatrix expected = new RotationMatrix();
         expected.setRotationVector(rotationVector);
         LinearTransform3D actual = new LinearTransform3D();
         actual.setRotationVector(rotationVector);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testSetEuler()
   {
      Random random = new Random(32546);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D euler = EuclidCoreRandomTools.nextRotationVector(random);
         RotationMatrix expected = new RotationMatrix();
         expected.setEuler(euler);
         LinearTransform3D actual = new LinearTransform3D();
         actual.setEuler(euler);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testAppendRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new RotationMatrix(orientation), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendRotation(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendRotationInvertThis()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, true, new RotationMatrix(orientation), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendRotationInvertThis(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendRotationInvertOther()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new RotationMatrix(orientation), false, true, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendRotationInvertOther(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendYawRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new RotationMatrix(yaw, 0, 0), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendPitchRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new RotationMatrix(0, pitch, 0), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendRollRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new RotationMatrix(0, 0, roll), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testAppendScale()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double scale = EuclidCoreRandomTools.nextDouble(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, new Matrix3D(scale, 0, 0, 0, scale, 0, 0, 0, scale), false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendScale(scale);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Matrix3D scale = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, scale, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendScale(new Vector3D(scale.getM00(), scale.getM11(), scale.getM22()));

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Matrix3D scale = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(original, false, false, scale, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.appendScale(scale.getM00(), scale.getM11(), scale.getM22());

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(orientation), false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependRotation(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependRotationInvertThis()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(orientation), false, false, original, false, true, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependRotationInvertThis(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependRotationInvertOther()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextOrientation3D(random);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(orientation), false, true, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependRotationInvertOther(orientation);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependYawRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(yaw, 0, 0), false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependPitchRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(0, pitch, 0), false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependRollRotation()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new RotationMatrix(0, 0, roll), false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testPrependScale()
   {
      Random random = new Random(34676);

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         double scale = EuclidCoreRandomTools.nextDouble(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(new Matrix3D(scale, 0, 0, 0, scale, 0, 0, 0, scale), false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependScale(scale);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Matrix3D scale = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(scale, false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependScale(new Vector3D(scale.getM00(), scale.getM11(), scale.getM22()));

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         CommonMatrix3DBasics original = EuclidCoreRandomTools.nextCommonMatrix3DBasics(random);
         Matrix3D scale = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);

         Matrix3D expected = new Matrix3D(original);
         Matrix3DTools.multiply(scale, false, false, original, false, false, expected);

         LinearTransform3D actual = new LinearTransform3D(original);
         actual.prependScale(scale.getM00(), scale.getM11(), scale.getM22());

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPSILON);
      }
   }

   @Test
   public void testGetOrientation()
   {
      Random random = new Random(74534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix r1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.5, 10.0);
         RotationMatrix r2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         RotationMatrix expected = new RotationMatrix();
         expected.set(r1);
         expected.append(r2);

         LinearTransform3D linearTransform3D = new LinearTransform3D();
         linearTransform3D.set(r1);
         linearTransform3D.appendScale(scale);
         linearTransform3D.appendRotation(r2);

         { // RotationMatrix
            RotationMatrix actual = new RotationMatrix();
            linearTransform3D.getOrientation(actual);
            EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, MID_EPSILON);
         }

         { // Quaternion
            Quaternion actual = new Quaternion();
            linearTransform3D.getOrientation(actual);
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals(new Quaternion(expected), actual, MID_EPSILON);
         }

         { // YawPitchRoll
            YawPitchRoll actual = new YawPitchRoll();
            linearTransform3D.getOrientation(actual);
            EuclidCoreTestTools.assertYawPitchRollEquals(new YawPitchRoll(expected), actual, MID_EPSILON);
         }
      }
   }

   @Test
   public void testGetRotationVector()
   {
      Random random = new Random(74534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix r1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.5, 10.0);
         RotationMatrix r2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(r1);
         rotationMatrix.append(r2);
         Vector3D expected = new Vector3D();
         rotationMatrix.getRotationVector(expected);

         LinearTransform3D linearTransform3D = new LinearTransform3D();
         linearTransform3D.set(r1);
         linearTransform3D.appendScale(scale);
         linearTransform3D.appendRotation(r2);

         Vector3D actual = new Vector3D();
         linearTransform3D.getRotationVector(actual);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(expected, actual, MID_EPSILON);
      }
   }

   @Test
   public void testGetEuler()
   {
      Random random = new Random(74534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix r1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.5, 10.0);
         RotationMatrix r2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(r1);
         rotationMatrix.append(r2);
         Vector3D expected = new Vector3D();
         rotationMatrix.getEuler(expected);

         LinearTransform3D linearTransform3D = new LinearTransform3D();
         linearTransform3D.set(r1);
         linearTransform3D.appendScale(scale);
         linearTransform3D.appendRotation(r2);

         Vector3D actual = new Vector3D();
         linearTransform3D.getEuler(actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, MID_EPSILON);
      }
   }

   @Test
   public void testGetScaleComponents()
   {
      Random random = new Random(74534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix r1 = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.5, 10.0);
         double maxScale = EuclidCoreTools.max(scale.getX(), scale.getY(), scale.getZ());
         double medScale = EuclidCoreTools.med(scale.getX(), scale.getY(), scale.getZ());
         double minScale = EuclidCoreTools.min(scale.getX(), scale.getY(), scale.getZ());
         scale.set(maxScale, medScale, minScale);
         RotationMatrix r2 = EuclidCoreRandomTools.nextRotationMatrix(random);

         LinearTransform3D linearTransform3D = new LinearTransform3D();
         linearTransform3D.set(r1);
         linearTransform3D.appendScale(scale);
         linearTransform3D.appendRotation(r2);

         assertEquals(maxScale, linearTransform3D.getScaleX(), SMALL_EPS);
         assertEquals(medScale, linearTransform3D.getScaleY(), SMALL_EPS);
         assertEquals(minScale, linearTransform3D.getScaleZ(), SMALL_EPS);
      }
   }

   @Test
   public void testGetAsQuaternion()
   {
      Random random = new Random(34536);

      LinearTransform3D linearTransform3D = new LinearTransform3D();
      QuaternionReadOnly actual = linearTransform3D.getAsQuaternion();
      Quaternion expected = new Quaternion();

      EuclidCoreTestTools.assertQuaternionEquals(expected, actual, SMALL_EPSILON);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         linearTransform3D.appendRotation(rotationMatrix);
         expected.append(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Iteration: " + i, expected, actual, MID_EPSILON);

         linearTransform3D.appendScale(EuclidCoreRandomTools.nextVector3D(random, 0.75, 1.25));
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Iteration: " + i, expected, actual, MID_EPSILON);

         rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         linearTransform3D.appendRotation(rotationMatrix);
         expected.append(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Iteration: " + i, expected, actual, MID_EPSILON);

         linearTransform3D.resetScale();
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, MID_EPSILON);
         expected.set(actual);
      }
   }

   @Test
   public void testGetPrePostQuaternionAndScale()
   {
      Random random = new Random(678658);

      LinearTransform3D linearTransform3D = new LinearTransform3D();
      QuaternionReadOnly actualPreScale = linearTransform3D.getPreScaleQuaternion();
      Vector3DReadOnly actualScale = linearTransform3D.getScaleVector();
      QuaternionReadOnly actualPostScale = linearTransform3D.getPostScaleQuaternion();

      Quaternion expectedPreScale = new Quaternion();
      Vector3D expectedScale = new Vector3D(1, 1, 1);
      Quaternion expectedPostScale = new Quaternion();

      EuclidCoreTestTools.assertQuaternionEquals(expectedPreScale, actualPreScale, SMALL_EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(expectedScale, actualScale, SMALL_EPSILON);
      EuclidCoreTestTools.assertQuaternionEquals(expectedPostScale, actualPostScale, SMALL_EPSILON);

      for (int i = 0; i < ITERATIONS; i++)
      {
         expectedPreScale.set(EuclidCoreRandomTools.nextQuaternion(random));
         expectedScale.set(EuclidCoreRandomTools.nextVector3D(random, 0.5, 10.0));
         double maxScale = EuclidCoreTools.max(expectedScale.getX(), expectedScale.getY(), expectedScale.getZ());
         double midScale = EuclidCoreTools.med(expectedScale.getX(), expectedScale.getY(), expectedScale.getZ());
         double minScale = EuclidCoreTools.min(expectedScale.getX(), expectedScale.getY(), expectedScale.getZ());
         expectedScale.set(maxScale, midScale, random.nextBoolean() ? +minScale : -minScale);
         expectedPostScale.set(EuclidCoreRandomTools.nextQuaternion(random));

         linearTransform3D.set(expectedPreScale);
         linearTransform3D.appendScale(expectedScale);
         linearTransform3D.appendRotation(expectedPostScale);

         if (expectedPreScale.geometricallyEquals(actualPreScale, LARGE_EPSILON))
         {
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Iteration: " + i, expectedPostScale, actualPostScale, LARGE_EPSILON);
         }
         else
         {
            double distance = expectedPreScale.distancePrecise(actualPreScale);
            assertEquals(Math.PI, distance, LARGE_EPSILON);

            Quaternion difference = new Quaternion();
            difference.difference(expectedPreScale, actualPreScale);
            difference.conjugate();
            expectedPostScale.prepend(difference);
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Iteration: " + i, expectedPostScale, actualPostScale, LARGE_EPSILON);
         }
         EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedScale, actualScale, SMALL_EPSILON);
      }
   }
}

package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.Matrix3DBasicsTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class LinearTransform3DTest extends Matrix3DBasicsTest<LinearTransform3D>
{
   private static final double EPSILON = 1.0e-12;
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
         EuclidCoreTestTools.assertMatrix3DEquals(matrix3D, linearTransform3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test LinearTransform3D(DMatrix matrix)
         DMatrix matrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 3, 3, 10.0);
         LinearTransform3D linearTransform3D = new LinearTransform3D(matrix);
         EuclidCoreTestTools.assertMatrix3DEquals(new Matrix3D(matrix), linearTransform3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test LinearTransform3D(Orientation3DReadOnly orientation)
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         LinearTransform3D linearTransform3D = new LinearTransform3D(orientation);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(orientation), linearTransform3D, EPSILON);
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
         Quaternion expectedPreScaleOrientation = new Quaternion(linearTransform3D.getPreScaleQuaternion());
         Quaternion expectedPostScaleOrientation = new Quaternion(linearTransform3D.getPostScaleQuaternion());

         linearTransform3D.resetScale();
         assertEquals(new Vector3D(1, 1, 1), linearTransform3D.getScaleVector());
         assertEquals(expectedOrientation, linearTransform3D.getAsQuaternion());
         assertEquals(expectedPreScaleOrientation, linearTransform3D.getPreScaleQuaternion());
         assertEquals(expectedPostScaleOrientation, linearTransform3D.getPostScaleQuaternion());
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
}

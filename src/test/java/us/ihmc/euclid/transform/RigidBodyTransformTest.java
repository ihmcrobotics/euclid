package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Arrays;
import java.util.Random;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class RigidBodyTransformTest extends TransformTest<RigidBodyTransform>
{
   private static final double EPS = 1.0e-14;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345L);

      { // Test empty constructor
         RigidBodyTransform transform = new RigidBodyTransform();
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertTrue(transform.getElement(row, column) == 1.0);
               else
                  assertTrue(transform.getElement(row, column) == 0.0);
            }
         }
      }

      { // Test RigidBodyTransform(RigidBodyTransform other)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(expected);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
      }

      { // Test RigidBodyTransform(QuaternionBasedTransform quaternionBasedTransform)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(new QuaternionBasedTransform(expected));
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertEquals(expected.getElement(row, column), actual.getElement(row, column), EPS);
            }
         }
      }

      { // Test RigidBodyTransform(DMatrix matrix)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RigidBodyTransform actual = new RigidBodyTransform(denseMatrix);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
      }

      { // Test RigidBodyTransform(Matrix3DReadOnly rotationMatrix, TupleReadOnly translation)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(rotationMatrix, vector);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
      }

      { // Test RigidBodyTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(new Quaternion(rotationMatrix), vector);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test RigidBodyTransform(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(denseMatrix);
         Vector3D vector = new Vector3D();
         vector.set(0, 3, denseMatrix);

         RigidBodyTransform actual = new RigidBodyTransform(new AxisAngle(rotationMatrix), vector);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      { // Test RigidBodyTransform(double[] transformArray)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               transformArray[4 * row + column] = expected.getElement(row, column);
            }
         }

         RigidBodyTransform actual = new RigidBodyTransform(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();

         RigidBodyTransform actual = new RigidBodyTransform(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }
   }

   @Test
   public void testResetRotation() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      assertTrue(transform.hasRotation());
      transform.setRotationToZero();
      assertFalse(transform.hasRotation());

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column == 3)
               assertTrue(transform.getElement(row, column) == original.getElement(row, column));
            else if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      assertTrue(transform.hasTranslation());
      transform.setTranslationToZero();
      assertFalse(transform.hasTranslation());

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column < 3)
               assertTrue(transform.getElement(row, column) == original.getElement(row, column));
            else if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testNormalizeRotationPart() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      double corruptionFactor = 0.1;
      double m00 = original.getM00() + corruptionFactor * random.nextDouble();
      double m01 = original.getM01() + corruptionFactor * random.nextDouble();
      double m02 = original.getM02() + corruptionFactor * random.nextDouble();
      double m10 = original.getM10() + corruptionFactor * random.nextDouble();
      double m11 = original.getM11() + corruptionFactor * random.nextDouble();
      double m12 = original.getM12() + corruptionFactor * random.nextDouble();
      double m20 = original.getM20() + corruptionFactor * random.nextDouble();
      double m21 = original.getM21() + corruptionFactor * random.nextDouble();
      double m22 = original.getM22() + corruptionFactor * random.nextDouble();
      transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      transform.normalizeRotationPart();

      Matrix3D rotation = new Matrix3D();
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      transform.getRotation(rotation);

      // Test that each row & column vectors are unit-length
      for (int j = 0; j < 3; j++)
      {
         rotation.getRow(j, vector1);
         assertEquals(1.0, vector1.length(), EPS);

         rotation.getColumn(j, vector1);
         assertEquals(1.0, vector1.length(), EPS);
      }

      // Test that each pair of rows and each pair of columns are orthogonal
      for (int j = 0; j < 3; j++)
      {
         rotation.getRow(j, vector1);
         rotation.getRow((j + 1) % 3, vector2);
         assertEquals(0.0, vector1.dot(vector2), EPS);

         rotation.getColumn(j, vector1);
         rotation.getColumn((j + 1) % 3, vector2);
         assertEquals(0.0, vector1.dot(vector2), EPS);
      }

      corruptionFactor = 0.9e-12;
      m00 = 1.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m01 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m02 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m10 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m11 = 1.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m12 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m20 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m21 = 0.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      m22 = 1.0 + EuclidCoreRandomTools.nextDouble(random, corruptionFactor);
      transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      assertFalse(transform.hasRotation());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (row == column)
            {
               assertEquals(1.0, transform.getElement(row, column), corruptionFactor);
               assertFalse(1.0 == transform.getElement(row, column));
            }
            else
            {
               assertEquals(0.0, transform.getElement(row, column), corruptionFactor);
               assertFalse(0.0 == transform.getElement(row, column));
            }
         }
      }

      transform.normalizeRotationPart();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (row == column)
            {
               assertTrue(1.0 == transform.getElement(row, column));
            }
            else
            {
               assertTrue(0.0 == transform.getElement(row, column));
            }
         }
      }
   }

   @Test
   public void testDeterminantRotationPart() throws Exception
   {
      Random random = new Random(42353L);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transform = new RigidBodyTransform(original);

      double corruptionFactor = 0.1;
      double m00 = original.getM00() * corruptionFactor;
      double m01 = original.getM01() * corruptionFactor;
      double m02 = original.getM02() * corruptionFactor;
      double m10 = original.getM10() * corruptionFactor;
      double m11 = original.getM11() * corruptionFactor;
      double m12 = original.getM12() * corruptionFactor;
      double m20 = original.getM20() * corruptionFactor;
      double m21 = original.getM21() * corruptionFactor;
      double m22 = original.getM22() * corruptionFactor;
      transform.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      assertTrue(transform.determinantRotationPart() < corruptionFactor);
      transform.normalizeRotationPart();
      assertEquals(1.0, transform.determinantRotationPart(), EPS);
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(34534L);

      { // Test set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();
         actual.set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         m00 = expected.getM00();
         m01 = expected.getM01();
         m02 = expected.getM02();
         m03 = expected.getM03();
         m10 = expected.getM10();
         m11 = expected.getM11();
         m12 = expected.getM12();
         m13 = expected.getM13();
         m20 = expected.getM20();
         m21 = expected.getM21();
         m22 = expected.getM22();
         m23 = expected.getM23();
         actual.set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         m00 = expected.getM00();
         m01 = expected.getM01();
         m02 = expected.getM02();
         m03 = expected.getM03();
         m10 = expected.getM10();
         m11 = expected.getM11();
         m12 = expected.getM12();
         m13 = expected.getM13();
         m20 = expected.getM20();
         m21 = expected.getM21();
         m22 = expected.getM22();
         m23 = expected.getM23();
         actual.set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test setUnsafe(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         double m00 = expected.getM00();
         double m01 = expected.getM01();
         double m02 = expected.getM02();
         double m03 = expected.getM03();
         double m10 = expected.getM10();
         double m11 = expected.getM11();
         double m12 = expected.getM12();
         double m13 = expected.getM13();
         double m20 = expected.getM20();
         double m21 = expected.getM21();
         double m22 = expected.getM22();
         double m23 = expected.getM23();
         actual.setUnsafe(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         m00 = expected.getM00();
         m01 = expected.getM01();
         m02 = expected.getM02();
         m03 = expected.getM03();
         m10 = expected.getM10();
         m11 = expected.getM11();
         m12 = expected.getM12();
         m13 = expected.getM13();
         m20 = expected.getM20();
         m21 = expected.getM21();
         m22 = expected.getM22();
         m23 = expected.getM23();
         actual.setUnsafe(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         m00 = expected.getM00();
         m01 = expected.getM01();
         m02 = expected.getM02();
         m03 = expected.getM03();
         m10 = expected.getM10();
         m11 = expected.getM11();
         m12 = expected.getM12();
         m13 = expected.getM13();
         m20 = expected.getM20();
         m21 = expected.getM21();
         m22 = expected.getM22();
         m23 = expected.getM23();
         actual.setUnsafe(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(RigidBodyTransform other)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         actual.set(expected);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         actual.set(expected);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         actual.set(expected);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(QuaternionBasedTransform quaternionBasedTransform)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         actual.set(new QuaternionBasedTransform(expected));
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(DMatrix matrix)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               denseMatrix.set(row, column, expected.getElement(row, column));
         actual.set(denseMatrix);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               denseMatrix.set(row, column, expected.getElement(row, column));
         actual.set(denseMatrix);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(DMatrix matrix, int startRow, int startColumn)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DMatrix denseMatrix = new DMatrixRMaj(4 + startRow, 4 + startColumn);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
         actual.set(denseMatrix, startRow, startColumn);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
         actual.set(denseMatrix, startRow, startColumn);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
         actual.set(denseMatrix, startRow, startColumn);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(double[] transformArray)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(float[] transformArray)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         float[] transformArray = new float[16];
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(row, column);
         actual.set(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test setAsTranspose(double[] transformArray)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test setAsTranspose(float[] transformArray)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         float[] transformArray = new float[16];
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setTranslationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         expected.setRotationToZero();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               transformArray[4 * row + column] = (float) expected.getElement(column, row);
         actual.setAsTranspose(transformArray);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, 1.0e-7);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(Matrix3DReadOnly rotationMatrix, TupleReadOnly translation)
         RigidBodyTransform actual = new RigidBodyTransform();
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set((Matrix3DReadOnly) expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         expectedTranslation = new Vector3D();
         actual.set((Matrix3DReadOnly) expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expectedRotation = new RotationMatrix();
         expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set((Matrix3DReadOnly) expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(RotationMatrix rotationMatrix, TupleReadOnly translation)
         RigidBodyTransform actual = new RigidBodyTransform();
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         expectedTranslation = new Vector3D();
         actual.set(expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expectedRotation = new RotationMatrix();
         expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedRotation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(RotationScaleMatrix rotationMatrix, TupleReadOnly translation)
         RigidBodyTransform actual = new RigidBodyTransform();
         RotationScaleMatrix expectedRotationScale = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedRotationScale, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale.getRotationMatrix(), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expectedRotationScale = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         expectedTranslation = new Vector3D();
         actual.set(expectedRotationScale, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale.getRotationMatrix(), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expectedRotationScale = new RotationScaleMatrix();
         expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedRotationScale, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale.getRotationMatrix(), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }

      { // Test set(Orientation3DReadOnly orientation, TupleReadOnly translation)
         RigidBodyTransform actual = new RigidBodyTransform();
         Quaternion expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedOrientation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(expectedOrientation), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());

         expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         expectedTranslation = new Vector3D();
         actual.set(expectedOrientation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(expectedOrientation), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());

         expectedOrientation = new Quaternion();
         expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(expectedOrientation, expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(expectedOrientation), actual.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actual.getTranslation(), 0.0);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      assertTrue(transform.hasRotation());
      assertTrue(transform.hasTranslation());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertFalse(transform.getElement(row, column) == 1.0);
            else
               assertFalse(transform.getElement(row, column) == 0.0);
         }
      }

      transform.setIdentity();
      assertFalse(transform.hasRotation());
      assertFalse(transform.hasTranslation());

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      assertTrue(transform.hasRotation());
      assertTrue(transform.hasTranslation());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertFalse(transform.getElement(row, column) == 1.0);
            else
               assertFalse(transform.getElement(row, column) == 0.0);
         }
      }

      transform.setToZero();
      assertFalse(transform.hasRotation());
      assertFalse(transform.hasTranslation());

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (row == column)
               assertTrue(transform.getElement(row, column) == 1.0);
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      assertTrue(transform.hasRotation());
      assertTrue(transform.hasTranslation());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setToNaN();
      assertTrue(transform.hasRotation());
      assertTrue(transform.hasTranslation());

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertTrue(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setToZero();
      assertFalse(transform.hasRotation());
      assertFalse(transform.hasTranslation());
      EuclidCoreTestTools.assertIdentity(transform.getRotation(), EPS);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslation());

      transform.setRotationToNaN();
      assertTrue(transform.hasRotation());
      assertFalse(transform.hasTranslation());
      EuclidCoreTestTools.assertMatrix3DContainsOnlyNaN(transform.getRotation());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslation());

      transform.setToZero();
      assertFalse(transform.hasRotation());
      assertFalse(transform.hasTranslation());
      transform.setTranslationToNaN();
      assertFalse(transform.hasRotation());
      assertTrue(transform.hasTranslation());
      EuclidCoreTestTools.assertIdentity(transform.getRotation(), EPS);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(transform.getTranslation());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(transform.containsNaN());
      transform.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testIsMatrix2D() throws Exception
   {
      Random random = new Random(3242L);
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      RigidBodyTransform transform = new RigidBodyTransform();
      double d = EuclidCoreRandomTools.nextDouble(random, 5.0);
      transform.setRotationUnsafe(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      transform.setTranslation(5.0, 3.0, -2.0);
      assertTrue(transform.isRotation2D());

      transform.setRotationUnsafe(0.0, 0.0, d, 0.0, 0.0, d, d, d, d);
      transform.setTranslation(5.0, 3.0, -2.0);
      assertFalse(transform.isRotation2D());
      transform.setRotationUnsafe(d, d, 0.0, d, d, 0.0, 0.0, 0.0, 1.0);
      transform.setTranslation(5.0, 3.0, -2.0);
      assertTrue(transform.isRotation2D());
   }

   @Test
   public void testCheckIfMatrix2D() throws Exception
   {
      Random random = new Random(3242L);
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      RigidBodyTransform transform = new RigidBodyTransform();
      double d = EuclidCoreRandomTools.nextDouble(random, 5.0);
      transform.setRotationUnsafe(0.0, 0.0, d, 0.0, 0.0, d, d, d, d);
      transform.setTranslation(5.0, 3.0, -2.0);
      try
      {
         transform.checkIfRotation2D();
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAnOrientation2DException e)
      {
         // good
         assertTrue(e.getMessage().equals("The orientation is not in XY plane: \n" + transform.getRotation()));
      }

      transform.setRotationUnsafe(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      transform.setTranslation(5.0, 3.0, -2.0);
      transform.checkIfRotation2D();
      transform.setRotationUnsafe(d, d, 0.0, d, d, 0.0, 0.0, 0.0, 1.0);
      transform.setTranslation(5.0, 3.0, -2.0);
      transform.checkIfRotation2D();
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(2342L);

      { // Test setRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         double m00 = expectedRotation.getM00();
         double m01 = expectedRotation.getM01();
         double m02 = expectedRotation.getM02();
         double m10 = expectedRotation.getM10();
         double m11 = expectedRotation.getM11();
         double m12 = expectedRotation.getM12();
         double m20 = expectedRotation.getM20();
         double m21 = expectedRotation.getM21();
         double m22 = expectedRotation.getM22();
         actual.setRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());

         expectedRotation.setToZero();
         m00 = expectedRotation.getM00();
         m01 = expectedRotation.getM01();
         m02 = expectedRotation.getM02();
         m10 = expectedRotation.getM10();
         m11 = expectedRotation.getM11();
         m12 = expectedRotation.getM12();
         m20 = expectedRotation.getM20();
         m21 = expectedRotation.getM21();
         m22 = expectedRotation.getM22();
         actual.setRotation(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertFalse(actual.hasRotation());
      }

      { // Test setRotationUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         double m00 = expectedRotation.getM00();
         double m01 = expectedRotation.getM01();
         double m02 = expectedRotation.getM02();
         double m10 = expectedRotation.getM10();
         double m11 = expectedRotation.getM11();
         double m12 = expectedRotation.getM12();
         double m20 = expectedRotation.getM20();
         double m21 = expectedRotation.getM21();
         double m22 = expectedRotation.getM22();
         actual.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());

         expectedRotation.setToZero();
         m00 = expectedRotation.getM00();
         m01 = expectedRotation.getM01();
         m02 = expectedRotation.getM02();
         m10 = expectedRotation.getM10();
         m11 = expectedRotation.getM11();
         m12 = expectedRotation.getM12();
         m20 = expectedRotation.getM20();
         m21 = expectedRotation.getM21();
         m22 = expectedRotation.getM22();
         actual.setRotationUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertFalse(actual.hasRotation());
      }

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AxisAngle axisAngle = new AxisAngle(expectedRotation);
         actual.setRotation(axisAngle);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), EPS);
         assertTrue(actual.hasRotation());

         axisAngle.setToZero();
         expectedRotation.set(axisAngle);
         actual.setRotation(axisAngle);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertFalse(actual.hasRotation());
      }

      { // Test setRotation(DMatrix matrix)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrixRMaj denseMatrix = new DMatrixRMaj(3, 3);
         expectedRotation.get(denseMatrix);
         actual.setRotation(denseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());

         CommonOps_DDRM.setIdentity(denseMatrix);
         expectedRotation.set(denseMatrix);
         actual.setRotation(denseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertFalse(actual.hasRotation());
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Quaternion quaternion = new Quaternion(expectedRotation);
         actual.setRotation(quaternion);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), EPS);
         assertTrue(actual.hasRotation());

         quaternion.setToZero();
         expectedRotation.set(quaternion);
         actual.setRotation(quaternion);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertFalse(actual.hasRotation());
      }

      { // Test setRotation(RotationMatrixReadOnly rotationMatrix)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.setRotation(expectedRotation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());
      }

      { // Test setRotation(Matrix3DReadOnly rotationMatrix)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.setRotation((Matrix3DReadOnly) expectedRotation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());
      }

      { // Test setRotation(Vector3DReadOnly rotationVector)
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.setRotation(rotationVector);
         expectedRotation.setRotationVector(rotationVector);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actual.getRotation(), 0.0);
         assertTrue(actual.hasRotation());
      }
   }

   @Test
   public void testAppendTranslation() throws Exception
   {
      Random random = new Random(35454L);

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(double x, double y, double z)
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform translationTransform = new RigidBodyTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         assertTrue(expected.hasTranslation());
         expected.multiply(translationTransform);
         assertTrue(expected.hasTranslation());

         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.appendTranslation(x, y, z);
         assertTrue(actual.hasTranslation());

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         actual.set(original);
         assertFalse(actual.hasTranslation());
         actual.appendTranslation(x, y, z);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.appendTranslation(0.0, 0.0, 0.0);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D negateTranslation = new Vector3D(original.getTranslation());
         negateTranslation.negate();
         original.inverseTransform(negateTranslation);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.appendTranslation(negateTranslation.getX(), negateTranslation.getY(), negateTranslation.getZ());
         assertFalse(actual.hasTranslation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(Tuple3DReadOnly translation)
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform translationTransform = new RigidBodyTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         assertTrue(expected.hasTranslation());
         expected.multiply(translationTransform);
         assertTrue(expected.hasTranslation());

         actual.set(original);
         actual.appendTranslation(translation);

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         actual.set(original);
         assertFalse(actual.hasTranslation());
         actual.appendTranslation(translation);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.appendTranslation(0.0, 0.0, 0.0);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D negateTranslation = new Vector3D(original.getTranslation());
         negateTranslation.negate();
         original.inverseTransform(negateTranslation);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.appendTranslation(negateTranslation);
         assertFalse(actual.hasTranslation());
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         RotationMatrix expectedRotation = new RotationMatrix();

         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.set(original.getRotation());
         expectedRotation.appendYawRotation(yaw);
         expected.set(expectedRotation, original.getTranslation());
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendYawRotation(yaw);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.appendYawRotation(yaw);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         yaw = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendYawRotation(yaw);
         assertTrue(actual.hasRotation());

         yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationYaw(-yaw);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendYawRotation(yaw);
         assertFalse(actual.hasRotation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getRotation());
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendPitchRotation(pitch);
         expected.set(expectedRotation, original.getTranslation());
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendPitchRotation(pitch);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.appendPitchRotation(pitch);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         pitch = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendPitchRotation(pitch);
         assertTrue(actual.hasRotation());

         pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationPitch(-pitch);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendPitchRotation(pitch);
         assertFalse(actual.hasRotation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendRollRotation(double roll)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getRotation());
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendRollRotation(roll);
         expected.set(expectedRotation, original.getTranslation());
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendRollRotation(roll);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.appendRollRotation(roll);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         roll = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendRollRotation(roll);
         assertTrue(actual.hasRotation());

         roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationRoll(-roll);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.appendRollRotation(roll);
         assertFalse(actual.hasRotation());
      }
   }

   @Test
   public void testPrependTranslation() throws Exception
   {
      Random random = new Random(35454L);

      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(double x, double y, double z)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform translationTransform = new RigidBodyTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         assertTrue(expected.hasTranslation());
         expected.preMultiply(translationTransform);
         assertTrue(expected.hasTranslation());

         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(x, y, z);
         assertTrue(actual.hasTranslation());

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         actual.set(original);
         assertFalse(actual.hasTranslation());
         actual.prependTranslation(x, y, z);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(0.0, 0.0, 0.0);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D negateTranslation = new Vector3D(original.getTranslation());
         negateTranslation.negate();
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(negateTranslation.getX(), negateTranslation.getY(), negateTranslation.getZ());
         assertFalse(actual.hasTranslation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(Tuple3DReadOnly translation)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform translationTransform = new RigidBodyTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         assertTrue(expected.hasTranslation());
         expected.preMultiply(translationTransform);
         assertTrue(expected.hasTranslation());

         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(translation);
         assertTrue(actual.hasTranslation());

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         actual.set(original);
         assertFalse(actual.hasTranslation());
         actual.prependTranslation(translation);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(0.0, 0.0, 0.0);
         assertTrue(actual.hasTranslation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D negateTranslation = new Vector3D(original.getTranslation());
         negateTranslation.negate();
         actual.set(original);
         assertTrue(actual.hasTranslation());
         actual.prependTranslation(negateTranslation);
         assertFalse(actual.hasTranslation());
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform yawTransform = new RigidBodyTransform();
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         yawTransform.setRotationYaw(yaw);
         expected.set(original);
         assertTrue(expected.hasRotation());
         expected.preMultiply(yawTransform);
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependYawRotation(yaw);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.prependYawRotation(yaw);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         yaw = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependYawRotation(yaw);
         assertTrue(actual.hasRotation());

         yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationYaw(-yaw);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependYawRotation(yaw);
         assertFalse(actual.hasRotation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform pitchTransform = new RigidBodyTransform();
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         pitchTransform.setRotationPitch(pitch);
         expected.set(original);
         expected.preMultiply(pitchTransform);
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependPitchRotation(pitch);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.prependPitchRotation(pitch);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         pitch = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependPitchRotation(pitch);
         assertTrue(actual.hasRotation());

         pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationPitch(-pitch);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependPitchRotation(pitch);
         assertFalse(actual.hasRotation());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependRollRotation(double roll)
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform rollTransform = new RigidBodyTransform();
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rollTransform.setRotationRoll(roll);
         expected.set(original);
         assertTrue(expected.hasRotation());
         expected.preMultiply(rollTransform);
         assertTrue(expected.hasRotation());

         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependRollRotation(roll);
         assertTrue(actual.hasRotation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         original.setToZero();
         roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         actual.set(original);
         assertFalse(actual.hasRotation());
         actual.prependRollRotation(roll);
         assertTrue(actual.hasRotation());

         original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         roll = 0.0;
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependRollRotation(roll);
         assertTrue(actual.hasRotation());

         roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         original.setRotationRoll(-roll);
         actual.set(original);
         assertTrue(actual.hasRotation());
         actual.prependRollRotation(roll);
         assertFalse(actual.hasRotation());
      }
   }

   @Test
   public void testSetRotationYawPitchRoll() throws Exception
   {
      Random random = new Random(234L);
      RotationMatrix expectedRotation = new RotationMatrix();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);

      { // Test setRotationYawPitchRoll(double yaw, double pitch, double roll)
         expectedRotation.set(yawPitchRoll);
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D expectedTranslation = new Vector3D();
         actualTransform.getTranslation(expectedTranslation);
         actualTransform.setRotationYawPitchRoll(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);

         actualTransform.setRotationYawPitchRoll(0.0, 0.0, 0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationYawPitchRollAndZeroTranslation(double yaw, double pitch, double roll)
         expectedRotation.set(yawPitchRoll);
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationYawPitchRollAndZeroTranslation(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll());
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationYawPitchRollAndZeroTranslation(0.0, 0.0, 0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationEulerAndZeroTranslation(double rotX, double rotY, double rotZ)
         expectedRotation.set(yawPitchRoll);
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationEulerAndZeroTranslation(yawPitchRoll.getRoll(), yawPitchRoll.getPitch(), yawPitchRoll.getYaw());
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationYaw(double yaw)
         expectedRotation.setToYawOrientation(yawPitchRoll.getYaw());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D expectedTranslation = new Vector3D();
         actualTransform.getTranslation(expectedTranslation);
         actualTransform.setRotationYaw(yawPitchRoll.getYaw());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);

         actualTransform.setRotationYaw(0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationPitch(double pitch)
         expectedRotation.setToPitchOrientation(yawPitchRoll.getPitch());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D expectedTranslation = new Vector3D();
         actualTransform.getTranslation(expectedTranslation);
         actualTransform.setRotationPitch(yawPitchRoll.getPitch());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);

         actualTransform.setRotationPitch(0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationRoll(double roll)
         expectedRotation.setToRollOrientation(yawPitchRoll.getRoll());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D expectedTranslation = new Vector3D();
         actualTransform.getTranslation(expectedTranslation);
         actualTransform.setRotationRoll(yawPitchRoll.getRoll());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);

         actualTransform.setRotationRoll(0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationYawAndZeroTranslation(double yaw)
         expectedRotation.setToYawOrientation(yawPitchRoll.getYaw());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationYawAndZeroTranslation(yawPitchRoll.getYaw());
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationYawAndZeroTranslation(0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationPitchAndZeroTranslation(double pitch)
         expectedRotation.setToPitchOrientation(yawPitchRoll.getPitch());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationPitchAndZeroTranslation(yawPitchRoll.getPitch());
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationPitchAndZeroTranslation(0.0);
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationRollAndZeroTranslation(double roll)
         expectedRotation.setToRollOrientation(yawPitchRoll.getRoll());
         actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationRollAndZeroTranslation(yawPitchRoll.getRoll());
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationRollAndZeroTranslation(0.0);
         assertFalse(actualTransform.hasRotation());
      }
   }

   @Test
   public void testSetRotationAndZeroTranslation() throws Exception
   {
      Random random = new Random(2342L);
      RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);

      { // Test setRotationAndZeroTranslation(AxisAngleReadOnly axisAngle)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AxisAngle axisAngle = new AxisAngle(expectedRotation);
         actualTransform.setRotationAndZeroTranslation(axisAngle);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationAndZeroTranslation(new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 0.0));
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationAndZeroTranslation(DMatrix matrix)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrix denseMatrix = new DMatrixRMaj(3, 3);
         expectedRotation.get(denseMatrix);
         actualTransform.setRotationAndZeroTranslation(denseMatrix);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationAndZeroTranslation(CommonOps_DDRM.identity(3));
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationAndZeroTranslation(QuaternionReadOnly quaternion)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Quaternion quaternion = new Quaternion(expectedRotation);
         actualTransform.setRotationAndZeroTranslation(quaternion);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), EPS);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationAndZeroTranslation(new Quaternion());
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotationAndZeroTranslation(Matrix3DReadOnly rotationMatrix)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationAndZeroTranslation(expectedRotation);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotationAndZeroTranslation(new RotationMatrix());
         assertFalse(actualTransform.hasRotation());
      }

      { // Test setRotation(Vector3DReadOnly rotationVector)
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setRotationAndZeroTranslation(rotationVector);
         expectedRotation.setRotationVector(rotationVector);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());

         actualTransform.setRotation(new Vector3D());
         assertFalse(actualTransform.hasRotation());
      }
   }

   @Test
   public void testRotationEuler() throws Exception
   {
      Random random = new Random(42353L);
      RotationMatrix expectedRotation = new RotationMatrix();
      Vector3D expectedTranslation = new Vector3D();

      { // Test setRotationEuler(VectorReadOnly eulerAngles)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getTranslation(expectedTranslation);
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         expectedRotation.setEuler(eulerAngles);
         actualTransform.setRotationEuler(eulerAngles);
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }

      { // Test setRotationEulerAndZeroTranslation(VectorReadOnly eulerAngles)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getTranslation(expectedTranslation);
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         expectedRotation.setEuler(eulerAngles);
         actualTransform.setRotationEulerAndZeroTranslation(eulerAngles);
         assertTrue(actualTransform.hasRotation());
         assertFalse(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(actualTransform.getTranslation());
      }

      { // Test setRotationEuler(double rotX, double rotY, double rotZ)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getTranslation(expectedTranslation);
         double rotX = Math.PI * random.nextDouble();
         double rotY = Math.PI * random.nextDouble();
         double rotZ = Math.PI * random.nextDouble();
         expectedRotation.setEuler(rotX, rotY, rotZ);
         actualTransform.setRotationEuler(rotX, rotY, rotZ);
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(243L);
      RotationMatrix expectedRotation = new RotationMatrix();
      Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);

      { // Test individual setTranslation(X/Y/Z)(double)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getRotation(expectedRotation);
         actualTransform.setTranslationX(expectedTranslation.getX());
         actualTransform.setTranslationY(expectedTranslation.getY());
         actualTransform.setTranslationZ(expectedTranslation.getZ());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);

         actualTransform.setTranslationX(0.0);
         actualTransform.setTranslationY(0.0);
         actualTransform.setTranslationZ(0.0);
         assertFalse(actualTransform.hasTranslation());

         actualTransform.setTranslationX(1.0);
         assertTrue(actualTransform.hasTranslation());
         actualTransform.setTranslationX(0.0);
         assertFalse(actualTransform.hasTranslation());

         actualTransform.setTranslationY(1.0);
         assertTrue(actualTransform.hasTranslation());
         actualTransform.setTranslationY(0.0);
         assertFalse(actualTransform.hasTranslation());

         actualTransform.setTranslationZ(1.0);
         assertTrue(actualTransform.hasTranslation());
         actualTransform.setTranslationZ(0.0);
         assertFalse(actualTransform.hasTranslation());
      }

      { // Test setTranslation(TupleReadOnly translation)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getRotation(expectedRotation);
         actualTransform.setTranslation(expectedTranslation);
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }

      { // Test setTranslation(double x, double y, double z)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getRotation(expectedRotation);
         actualTransform.setTranslation(expectedTranslation.getX(), expectedTranslation.getY(), expectedTranslation.getZ());
         assertTrue(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotation, actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }

      { // Test setTranslationAndIdentityRotation(TupleReadOnly translation)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.setTranslationAndIdentityRotation(expectedTranslation);
         assertFalse(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertIdentity(actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }

      { // Test setTranslationAndIdentityRotation(double x, double y, double z)
         RigidBodyTransform actualTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actualTransform.getRotation(expectedRotation);
         actualTransform.setTranslationAndIdentityRotation(expectedTranslation.getX(), expectedTranslation.getY(), expectedTranslation.getZ());
         assertFalse(actualTransform.hasRotation());
         assertTrue(actualTransform.hasTranslation());
         EuclidCoreTestTools.assertIdentity(actualTransform.getRotation(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslation(), 0.0);
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(2345L);
      RotationMatrix rotationMatrix = new RotationMatrix();

      { // Test getRotation(Matrix3DBasics rotationMatrixToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         transform.getRotation((CommonMatrix3DBasics) rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         transform.getRotation(rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(RotationScaleMatrix rotationMatrixToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.getRotation(rotationScaleMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(DMatrix matrixToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrix denseMatrix = new DMatrixRMaj(3, 3);
         transform.getRotation(denseMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Quaternion quaternion = new Quaternion();
         transform.getRotation(quaternion);
         rotationMatrix.set(quaternion);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         rotationMatrix.set(axisAngle);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }

      { // Test getRotation(double[] rotationMatrixArrayToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double[] expected = new double[9];
         double[] actual = new double[9];
         transform.getRotation(actual);
         transform.getRotation().get(expected);
         assertTrue(Arrays.equals(expected, actual));
      }

      { // Test getRotation(Vector3DBasics rotationVectorToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3DBasics rotationVector = new Vector3D();
         transform.getRotation(rotationVector);
         rotationMatrix.setRotationVector(rotationVector);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }

      { // Test getRotationEuler(Vector3DBasics eulerAngles)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3DBasics eulerAngles = new Vector3D();
         transform.getRotationEuler(eulerAngles);
         rotationMatrix.setEuler(eulerAngles);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getElement(row, column), EPS);
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(2345L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);
      for (int row = 0; row < 3; row++)
         assertTrue(translation.getElement(row) == transform.getElement(row, 3));

      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslation(), EPS);

      translation.set(transform.getTranslationX(), transform.getTranslationY(), transform.getTranslationZ());
      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslation(), EPS);
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(2342L);

      { // Test get(DMatrix matrixToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrix denseMatrix = RandomMatrices_DDRM.rectangle(4, 4, random);
         transform.get(denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test get(DMatrix matrixToPack, int startRow, int startColumn)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DMatrix denseMatrix = RandomMatrices_DDRM.rectangle(4 + startRow, 4 + startColumn, random);
         transform.get(startRow, startColumn, denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row + startRow, column + startColumn) == transform.getElement(row, column));
      }

      { // Test get(double[] transformArrayToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         double[] transformArray = new double[16];
         transform.get(transformArray);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(transformArray[4 * row + column] == transform.getElement(row, column));
      }

      { // Test get(float[] transformArrayToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         float[] transformArray = new float[16];
         transform.get(transformArray);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(transformArray[4 * row + column], transform.getElement(row, column), 1.0e-7);
      }

      { // Test get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Quaternion expectedQuaternion = new Quaternion();
         Quaternion actualQuaternion = new Quaternion();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedQuaternion);
         transform.getTranslation(expectedTranslation);
         transform.get(actualQuaternion, actualTranslation);
         assertEquals(expectedQuaternion, actualQuaternion);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(AxisAngleBasics axisAngleToPack, Tuple3DBasics<Vector3D> translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AxisAngle expected = new AxisAngle();
         AxisAngle actual = new AxisAngle();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expected);
         transform.getTranslation(expectedTranslation);
         transform.get(actual, actualTranslation);
         assertEquals(expected, actual);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(Vector3DBasics rotationVectorToPack, Tuple3DBasics<Vector3D> translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expected);
         transform.getTranslation(expectedTranslation);
         transform.get(actual, actualTranslation);
         assertEquals(expected, actual);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(Matrix3DBasics rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Matrix3D expectedMatrix = new Matrix3D();
         Matrix3D actualMatrix = new Matrix3D();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix expectedMatrix = new RotationMatrix();
         RotationMatrix actualMatrix = new RotationMatrix();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationScaleMatrix expectedMatrix = new RotationScaleMatrix();
         RotationScaleMatrix actualMatrix = new RotationScaleMatrix();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotation(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(3453L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrixRMaj denseMatrix = new DMatrixRMaj(4, 4);
         transform.get(denseMatrix);
         CommonOps_DDRM.invert(denseMatrix);
         transform.invert();
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(denseMatrix.get(row, column), transform.getElement(row, column), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RotationMatrix expectedRotationPart = new RotationMatrix(transform.getRotation());
         Vector3D expectedTranslationPart = new Vector3D(transform.getTranslation());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationPart, transform.getRotation(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslationPart, transform.getTranslation(), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expectedTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actualTransform = new RigidBodyTransform();
         actualTransform.setAndInvert(expectedTransform);
         expectedTransform.invert();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverse = new RigidBodyTransform(transform);
         inverse.invert();

         assertTrue(transform.hasRotation());
         assertTrue(transform.hasTranslation());
         transform.multiply(inverse);
         assertFalse(transform.hasRotation());
         assertFalse(transform.hasTranslation());

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertEquals(transform.getElement(row, column), 1.0, EPS);
               else
                  assertEquals(transform.getElement(row, column), 0.0, EPS);
            }
         }
      }

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform t2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         checkMultiplyAgainstEJML(t1, t2);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2.setRotation(t1.getRotation());
         t2.invertRotation();
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t1.getTranslation());
         negateTranslation.negate();
         t1.inverseTransform(negateTranslation);
         t2.setTranslation(negateTranslation);
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
      }
   }

   private static void checkMultiplyAgainstEJML(RigidBodyTransform t1, RigidBodyTransform t2)
   {
      RigidBodyTransform t3 = new RigidBodyTransform(t1);
      t3.multiply(t2);

      DMatrixRMaj m1 = new DMatrixRMaj(4, 4);
      DMatrixRMaj m2 = new DMatrixRMaj(4, 4);
      DMatrixRMaj m3 = new DMatrixRMaj(4, 4);
      t1.get(m1);
      t2.get(m2);
      CommonOps_DDRM.mult(m1, m2, m3);

      for (int row = 0; row < 4; row++)
         for (int column = 0; column < 4; column++)
            assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
   }

   @Test
   public void testMultiplyWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.multiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiply(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         multipliedWith.set(actual);
         multipliedWith.invert();
         actual.multiply(multipliedWith);
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.setScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.multiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiply(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         multipliedWithRigidBody.set(actual);
         multipliedWithRigidBody.invert();
         actual.multiply(new AffineTransform(multipliedWithRigidBody));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.set(original);
         expected.invert();
         expected.multiply(multipliedWith);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertThis(new RigidBodyTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(t1.getRotation(), EuclidCoreRandomTools.nextVector3D(random));
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), t1.getTranslation());
         expected.setAndInvert(t1);
         expected.multiply(t2);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWith = new RigidBodyTransform(multipliedWith);
         inverseOfMultipliedWith.invert();

         expected.set(original);
         expected.multiply(inverseOfMultipliedWith);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertOther(new RigidBodyTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2Inverse = new RigidBodyTransform();
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(t1.getRotation(), EuclidCoreRandomTools.nextVector3D(random));
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t1.getTranslation());
         t1.inverseTransform(negateTranslation);
         t2.transform(negateTranslation);
         t2.setTranslation(negateTranslation);
         t2Inverse.setAndInvert(t2);
         expected.set(t1);
         expected.multiply(t2Inverse);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.invert();
         expected.multiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertThis(new QuaternionBasedTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWithRigidBody = new RigidBodyTransform(multipliedWithRigidBody);
         inverseOfMultipliedWithRigidBody.invert();
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.multiply(inverseOfMultipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertOther(new QuaternionBasedTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.setScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.invert();
         expected.multiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertThis(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertThis(new AffineTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWithRigidBody = new RigidBodyTransform(multipliedWithRigidBody);
         inverseOfMultipliedWithRigidBody.invert();
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.setScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.multiply(inverseOfMultipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.multiplyInvertOther(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.multiplyInvertOther(new AffineTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverse = new RigidBodyTransform(transform);
         inverse.invert();

         assertTrue(transform.hasRotation());
         assertTrue(transform.hasTranslation());
         transform.preMultiply(inverse);
         assertFalse(transform.hasRotation());
         assertFalse(transform.hasTranslation());

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               if (row == column)
                  assertEquals(transform.getElement(row, column), 1.0, EPS);
               else
                  assertEquals(transform.getElement(row, column), 0.0, EPS);
            }
         }
      }

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform t2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform t3 = new RigidBodyTransform(t2);
         t3.preMultiply(t1);

         DMatrixRMaj m1 = new DMatrixRMaj(4, 4);
         DMatrixRMaj m2 = new DMatrixRMaj(4, 4);
         DMatrixRMaj m3 = new DMatrixRMaj(4, 4);
         t1.get(m1);
         t2.get(m2);
         CommonOps_DDRM.mult(m1, m2, m3);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t1.setRotation(t2.getRotation());
         t1.invertRotation();
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t2.getTranslation());
         negateTranslation.negate();
         t2.inverseTransform(negateTranslation);
         t1.setTranslation(negateTranslation);
         expected.set(t2);
         expected.multiply(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.preMultiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         multipliedWith.set(actual);
         multipliedWith.invert();
         actual.preMultiply(multipliedWith);
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.getScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.preMultiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiply(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         multipliedWithRigidBody.set(actual);
         multipliedWithRigidBody.invert();
         actual.preMultiply(new AffineTransform(multipliedWithRigidBody));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.set(original);
         expected.invert();
         expected.preMultiply(multipliedWith);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.preMultiplyInvertThis(new RigidBodyTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t1.setRotation(t2.getRotation());
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t2.getTranslation());
         t2.inverseTransform(negateTranslation);
         t1.transform(negateTranslation);
         t1.setTranslation(negateTranslation);
         expected.set(t2);
         expected.multiplyInvertOther(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWith = new RigidBodyTransform(multipliedWith);
         inverseOfMultipliedWith.invert();

         expected.set(original);
         expected.preMultiply(inverseOfMultipliedWith);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.preMultiplyInvertOther(new RigidBodyTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform expected = new RigidBodyTransform();
         RigidBodyTransform actual = new RigidBodyTransform();

         RigidBodyTransform t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t1.setRotation(t2.getRotation());
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertFalse(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t1.setTranslation(t2.getTranslation());
         expected.set(t2);
         expected.multiplyInvertThis(t1);
         actual.set(t1);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertOther(t2);
         assertTrue(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.invert();
         expected.preMultiply(multipliedWithRigidBody);
         actual.set(original);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         actual.preMultiplyInvertThis(multipliedWith);
         assertTrue(actual.hasRotation());
         assertTrue(actual.hasTranslation());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.preMultiplyInvertThis(new QuaternionBasedTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWithRigidBody = new RigidBodyTransform(multipliedWithRigidBody);
         inverseOfMultipliedWithRigidBody.invert();
         QuaternionBasedTransform multipliedWith = new QuaternionBasedTransform(multipliedWithRigidBody);

         expected.set(original);
         expected.preMultiply(inverseOfMultipliedWithRigidBody);
         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.preMultiplyInvertOther(new QuaternionBasedTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.setScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.invert();
         expected.preMultiply(multipliedWithRigidBody);
         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.preMultiplyInvertThis(new AffineTransform(actual));
         assertFalse(actual.hasRotation());
         assertFalse(actual.hasTranslation());
         expected.setToZero();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(23542342L);

      RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform t0 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform tf = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      actual.interpolate(t0, tf, 0.0);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(t0, actual, EPS);
      actual.interpolate(tf, 0.0);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(t0, actual, EPS);
      actual.interpolate(tf, 1.0);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(tf, actual, EPS);

      actual.interpolate(t0, tf, 1.0);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(tf, actual, EPS);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double alpha = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Vector3D interpolatedVector = new Vector3D();
         RotationMatrix interpolatedRotation = new RotationMatrix();

         interpolatedVector.interpolate(t0.getTranslation(), tf.getTranslation(), alpha);
         interpolatedRotation.interpolate(t0.getRotation(), tf.getRotation(), alpha);

         expected.set(interpolatedRotation, interpolatedVector);
         actual.interpolate(t0, tf, alpha);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

         actual.set(t0);
         actual.interpolate(tf, alpha);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against multiply(RigidBodyTransform)
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform multipliedWithRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform inverseOfMultipliedWithRigidBody = new RigidBodyTransform(multipliedWithRigidBody);
         inverseOfMultipliedWithRigidBody.invert();
         AffineTransform multipliedWith = new AffineTransform(multipliedWithRigidBody);
         multipliedWith.setScale(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));

         expected.set(original);
         expected.preMultiply(inverseOfMultipliedWithRigidBody);
         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformWithTuple() throws Exception
   {
      Random random = new Random(432L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      DMatrixRMaj matrix = new DMatrixRMaj(4, 4);
      transform.get(matrix);

      { // Test transform(PointBasics pointToTransform)
         DMatrixRMaj ejmlPoint = new DMatrixRMaj(4, 1);
         DMatrixRMaj ejmlTransformedPoint = new DMatrixRMaj(4, 1);

         Point3D point = EuclidCoreRandomTools.nextPoint3D(random);
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point);
         CommonOps_DDRM.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), point.getElement(i), EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         DMatrixRMaj ejmlPoint = new DMatrixRMaj(4, 1);
         DMatrixRMaj ejmlTransformedPoint = new DMatrixRMaj(4, 1);

         Point3D point = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointTransformed = new Point3D();
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point, pointTransformed);
         CommonOps_DDRM.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), pointTransformed.getElement(i), EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         DMatrixRMaj ejmlVector = new DMatrixRMaj(4, 1);
         DMatrixRMaj ejmlTransformedVector = new DMatrixRMaj(4, 1);

         Vector3D vector = EuclidCoreRandomTools.nextVector3D(random);
         vector.get(ejmlVector);

         transform.transform(vector);
         CommonOps_DDRM.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vector.getElement(i), EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         DMatrixRMaj ejmlVector = new DMatrixRMaj(4, 1);
         DMatrixRMaj ejmlTransformedVector = new DMatrixRMaj(4, 1);

         Vector3D vector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorTransformed = new Vector3D();
         vector.get(ejmlVector);

         transform.transform(vector, vectorTransformed);
         CommonOps_DDRM.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vectorTransformed.getElement(i), EPS);
      }
   }

   @Test
   public void testTransformWithQuaternion() throws Exception
   {
      Random random = new Random(34534L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Quaternion quaternionOriginal = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      transform.getRotation(quaternionExpected);
      quaternionExpected.multiply(quaternionOriginal);

      transform.transform(quaternionOriginal, quaternionActual);
      EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPS);

      quaternionActual.set(quaternionOriginal);
      transform.transform(quaternionActual);
      EuclidCoreTestTools.assertQuaternionEquals(quaternionExpected, quaternionActual, EPS);
   }

   @Test
   public void testTransformWithVector4D() throws Exception
   {
      Random random = new Random(5634L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Vector4D vectorOriginal = EuclidCoreRandomTools.nextVector4D(random);
      Vector4D vectorExpected = new Vector4D();
      Vector4D vectorActual = new Vector4D();

      Vector3D vector3D = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), vectorOriginal.getZ());
      transform.transform(vector3D);
      vectorExpected.set(vector3D);
      vectorExpected.setS(vectorOriginal.getS());
      vectorExpected.addX(vectorExpected.getS() * transform.getM03());
      vectorExpected.addY(vectorExpected.getS() * transform.getM13());
      vectorExpected.addZ(vectorExpected.getS() * transform.getM23());

      transform.transform(vectorOriginal, vectorActual);
      EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);

      vectorActual.set(vectorOriginal);
      transform.transform(vectorActual);
      EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);

      // Try with dense-matrix
      DMatrixRMaj transformDenseMatrix = new DMatrixRMaj(4, 4);
      transform.get(transformDenseMatrix);
      DMatrixRMaj vectorOriginalDenseMatrix = new DMatrixRMaj(4, 1);
      vectorOriginal.get(vectorOriginalDenseMatrix);
      DMatrixRMaj vectorTransformedDenseMatrix = new DMatrixRMaj(4, 1);
      CommonOps_DDRM.mult(transformDenseMatrix, vectorOriginalDenseMatrix, vectorTransformedDenseMatrix);
      vectorExpected.set(vectorTransformedDenseMatrix);

      EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformWithTuple2D() throws Exception
   {
      Random random = new Random(4353L);
      RigidBodyTransform transfom2D = new RigidBodyTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));

      { // Test transform(Point2DBasics pointToTransform)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         pointActual.set(pointOriginal);
         transfom2D.transform(pointActual);
         EuclidCoreTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         pointActual.set(pointOriginal);
         transfom2D.transform(pointActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual);
         EuclidCoreTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual, true);
         EuclidCoreTestTools.assertTuple2DEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D vectorExpected = new Vector2D();
         Vector2D vectorActual = new Vector2D();

         Vector3D vector = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), 0.0);
         transfom2D.transform(vector);
         vectorExpected.set(vector.getX(), vector.getY());

         vectorActual.set(vectorOriginal);
         transfom2D.transform(vectorActual);
         EuclidCoreTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D vectorExpected = new Vector2D();
         Vector2D vectorActual = new Vector2D();

         Vector3D vector = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), 0.0);
         transfom2D.transform(vector);
         vectorExpected.set(vector.getX(), vector.getY());

         transfom2D.transform(vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertTuple2DEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testTransformWithMatrix3D() throws Exception
   {
      Random random = new Random(4534L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Matrix3D matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      {
         Matrix3D m = new Matrix3D();
         transform.getRotation(m);
         matrixExpected.set(matrixOriginal);
         matrixExpected.preMultiply(m);
         matrixExpected.multiplyTransposeOther(m);
      }

      transform.transform(matrixOriginal, matrixActual);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.set(matrixOriginal);
      transform.transform(matrixActual);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformWithRotationMatrix() throws Exception
   {
      Random random = new Random(2342L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RotationMatrix matrixOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      transform.getRotation(matrixExpected);
      matrixExpected.multiply(matrixOriginal);

      matrixActual.set(matrixOriginal);
      transform.transform(matrixActual);
      EuclidCoreTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformWithOtherRigidBodyTransform() throws Exception
   {
      Random random = new Random(23423L);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      expected.set(transform);
      expected.multiply(original);

      transform.transform(original, actual);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(23423L);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      QuaternionBasedTransform original = new QuaternionBasedTransform(originalRigidBodyTransform);
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithAffineTransform() throws Exception
   {
      Random random = new Random(23423L);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);

      AffineTransform original = new AffineTransform(originalRigidBodyTransform);
      original.setScale(scale);
      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      expected.setScale(scale);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(5464L);
      RigidBodyTransform transform = new RigidBodyTransform();
      double coeff;

      transform.setUnsafe(coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM00() == coeff);
      assertTrue(transform.getElement(0, 0) == coeff);
      transform.setUnsafe(0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM01() == coeff);
      assertTrue(transform.getElement(0, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM02() == coeff);
      assertTrue(transform.getElement(0, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM03() == coeff);
      assertTrue(transform.getElement(0, 3) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM10() == coeff);
      assertTrue(transform.getElement(1, 0) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM11() == coeff);
      assertTrue(transform.getElement(1, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM12() == coeff);
      assertTrue(transform.getElement(1, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.getM13() == coeff);
      assertTrue(transform.getElement(1, 3) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0);
      assertTrue(transform.getM20() == coeff);
      assertTrue(transform.getElement(2, 0) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0);
      assertTrue(transform.getM21() == coeff);
      assertTrue(transform.getElement(2, 1) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0);
      assertTrue(transform.getM22() == coeff);
      assertTrue(transform.getElement(2, 2) == coeff);
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble());
      assertTrue(transform.getM23() == coeff);
      assertTrue(transform.getElement(2, 3) == coeff);

      try
      {
         transform.getElement(0, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(1, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(2, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(3, 4);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         transform.getElement(4, 0);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      RigidBodyTransform t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform t2 = new RigidBodyTransform();

      assertFalse(t1.equals(t2));
      assertFalse(t1.equals(null));
      assertFalse(t1.equals(new double[4]));
      t2.set(t1);
      assertTrue(t1.equals(t2));
      Object t2AsObject = t2;
      assertTrue(t1.equals(t2AsObject));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[16];

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            t2.set(t1);
            assertTrue(t1.equals(t2));
            t1.get(coeffs);
            coeffs[3 * row + column] += smallestEpsilon;
            t2.set(coeffs);
            assertFalse(t1.equals(t2));

            t2.set(t1);
            assertTrue(t1.equals(t2));
            t1.get(coeffs);
            coeffs[3 * row + column] -= smallestEpsilon;
            t2.set(coeffs);
            assertFalse(t1.equals(t2));
         }
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      RigidBodyTransform t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform t2 = new RigidBodyTransform();
      double epsilon = 1.0e-3;
      double[] coeffs = new double[16];

      assertFalse(t1.epsilonEquals(t2, epsilon));
      t2.set(t1);
      assertTrue(t1.epsilonEquals(t2, epsilon));

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] += 0.999 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] += 1.001 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertFalse(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] -= 0.999 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.get(coeffs);
            coeffs[3 * row + column] -= 1.001 * epsilon;
            t2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]);
            assertFalse(t1.epsilonEquals(t2, epsilon));
         }
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(19825L);
      RigidBodyTransform rigbodA;
      RigidBodyTransform rigbodB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rigbodA = createRandomTransform(random);
         double angleDiff = 0.99 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         RotationMatrix rotmat = new RotationMatrix(aa);
         rotmat.preMultiply(rigbodA.getRotation());

         rigbodB = new RigidBodyTransform(rotmat, rigbodA.getTranslation());

         assertTrue(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertTrue(rigbodB.geometricallyEquals(rigbodA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rigbodA = createRandomTransform(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         RotationMatrix rotmat = new RotationMatrix(aa);
         rotmat.preMultiply(rigbodA.getRotation());

         rigbodB = new RigidBodyTransform(rotmat, rigbodA.getTranslation());

         assertFalse(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertFalse(rigbodB.geometricallyEquals(rigbodA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rigbodA = createRandomTransform(random);

         Vector3D translation = new Vector3D(rigbodA.getTranslation());
         Vector3D perturb = new Vector3D(translation);

         perturb.setX(translation.getX() + 0.9 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertTrue(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertTrue(rigbodB.geometricallyEquals(rigbodA, epsilon));

         perturb.setX(translation.getX() + 1.1 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertFalse(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertFalse(rigbodB.geometricallyEquals(rigbodA, epsilon));

         perturb = new Vector3D(translation);
         perturb.setY(translation.getY() + 0.9 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertTrue(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertTrue(rigbodB.geometricallyEquals(rigbodA, epsilon));

         perturb.setY(translation.getY() + 1.1 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertFalse(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertFalse(rigbodB.geometricallyEquals(rigbodA, epsilon));

         perturb = new Vector3D(translation);
         perturb.setZ(translation.getZ() + 0.9 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertTrue(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertTrue(rigbodB.geometricallyEquals(rigbodA, epsilon));

         perturb.setZ(translation.getZ() + 1.1 * epsilon);
         rigbodB = new RigidBodyTransform(new RotationMatrix(rigbodA.getRotation()), perturb);

         assertFalse(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertFalse(rigbodB.geometricallyEquals(rigbodA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         rigbodA = createRandomTransform(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         RotationMatrix rotmat = new RotationMatrix(aa);
         rotmat.preMultiply(rigbodA.getRotation());

         rigbodB = new RigidBodyTransform(rotmat, rigbodA.getTranslation());

         assertFalse(rigbodA.geometricallyEquals(rigbodB, epsilon));
         assertFalse(rigbodB.geometricallyEquals(rigbodA, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(12345L);

      RotationMatrix rotation;
      Vector3D translation;
      RigidBodyTransform rbt = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      int newHashCode, previousHashCode;
      newHashCode = rbt.hashCode();
      assertEquals(newHashCode, rbt.hashCode());

      previousHashCode = rbt.hashCode();

      for (int i = 0; i < ITERATIONS; ++i)
      {
         rotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         translation = EuclidCoreRandomTools.nextVector3D(random);
         rbt = new RigidBodyTransform(rotation, translation);
         newHashCode = rbt.hashCode();
         assertNotEquals(previousHashCode, newHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testToString() throws Exception
   {
      Random random = new Random(12345L);

      RigidBodyTransform rbtA;
      RigidBodyTransform rbtB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         rbtA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         rbtB = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         assertNotEquals(rbtA.toString(), rbtB.toString());
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         rbtA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         rbtB = new RigidBodyTransform(rbtA);

         assertEquals(rbtA.toString(), rbtB.toString());
      }
   }

   @Override
   public RigidBodyTransform createRandomTransform(Random random)
   {
      return EuclidCoreRandomTools.nextRigidBodyTransform(random);
   }

   @Override
   public RigidBodyTransform createRandomTransform2D(Random random)
   {
      RigidBodyTransform transfom2D = new RigidBodyTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      return transfom2D;
   }
}

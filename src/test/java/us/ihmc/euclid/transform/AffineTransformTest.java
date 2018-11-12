package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;

public class AffineTransformTest extends TransformTest<AffineTransform>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3435L);

      { // Test empty constructor
         AffineTransform transform = new AffineTransform();
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

      { // Test RigidBodyTransform(AffineTransform other)
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = new AffineTransform(expected);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test RigidBodyTransform(RigidBodyTransform rigidBodyTransform)
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform actual = new AffineTransform(expected);

         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               assertTrue(expected.getElement(row, column) == actual.getElement(row, column));
            }
         }
      }

      { // Test AffineTransform(RotationScaleMatrixReadOnly rotationScaleMatrix, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         AffineTransform transform = new AffineTransform(rotationScaleMatrix, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == transform.getElement(row, column));
            assertTrue(translation.getElement(row) == transform.getElement(row, 3));
         }
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(2342L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setToNaN();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertTrue(Double.isNaN(transform.getElement(row, column)));
         }
      }
   }

   @Test
   public void testSetRotationToNaN() throws Exception
   {
      Random random = new Random(2342L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setRotationToNaN();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column < 3)
               assertTrue(Double.isNaN(transform.getElement(row, column)));
            else
               assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }
   }

   @Test
   public void testSetTranslationToNaN() throws Exception
   {
      Random random = new Random(2342L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            assertFalse(Double.isNaN(transform.getElement(row, column)));
         }
      }

      transform.setTranslationToNaN();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column < 3)
               assertFalse(Double.isNaN(transform.getElement(row, column)));
            else
               assertTrue(Double.isNaN(transform.getElement(row, column)));
         }
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      AffineTransform transform = new AffineTransform();
      assertFalse(transform.containsNaN());
      transform.setRotationToNaN();
      assertTrue(transform.containsNaN());
      transform.setIdentity();
      assertFalse(transform.containsNaN());
      transform.setTranslationToNaN();
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testNormalizeRotationPart() throws Exception
   {
      Random random = new Random(2342L);

      RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
      Tuple3DReadOnly translation = EuclidCoreRandomTools.nextVector3D(random);
      AffineTransform affineTransform = new AffineTransform(rotationScaleMatrix, translation);

      double m00 = rotationScaleMatrix.getRotationMatrix().getM00() + 1.0e-5;
      double m01 = rotationScaleMatrix.getRotationMatrix().getM01() + 1.0e-5;
      double m02 = rotationScaleMatrix.getRotationMatrix().getM02() + 1.0e-5;
      double m10 = rotationScaleMatrix.getRotationMatrix().getM10() + 1.0e-5;
      double m11 = rotationScaleMatrix.getRotationMatrix().getM11() + 1.0e-5;
      double m12 = rotationScaleMatrix.getRotationMatrix().getM12() + 1.0e-5;
      double m20 = rotationScaleMatrix.getRotationMatrix().getM20() + 1.0e-5;
      double m21 = rotationScaleMatrix.getRotationMatrix().getM21() + 1.0e-5;
      double m22 = rotationScaleMatrix.getRotationMatrix().getM22() + 1.0e-5;

      rotationScaleMatrix.getRotationMatrix().setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      ((RotationMatrix) affineTransform.getRotationMatrix()).setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      rotationScaleMatrix.normalizeRotationMatrix();
      affineTransform.normalizeRotationPart();

      EuclidCoreTestTools.assertMatrix3DEquals(rotationScaleMatrix, affineTransform.getRotationScaleMatrix(), EPS);
      EuclidCoreTestTools.assertTuple3DEquals(translation, affineTransform.getTranslationVector(), EPS);
   }

   @Test
   public void testResetRotation() throws Exception
   {
      Random random = new Random(42353L);
      AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform transform = new AffineTransform(original);

      transform.setRotationToZero();

      for (int row = 0; row < 4; row++)
      {
         for (int column = 0; column < 4; column++)
         {
            if (column == 3)
               assertTrue(transform.getElement(row, column) == original.getElement(row, column));
            else if (row == column)
               assertTrue(transform.getElement(row, column) == transform.getRotationScaleMatrix().getScale().getElement(column));
            else
               assertTrue(transform.getElement(row, column) == 0.0);
         }
      }
   }

   @Test
   public void testResetScale() throws Exception
   {
      Random random = new Random(42353L);
      AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform transform = new AffineTransform(original);
      RotationMatrix rotation = new RotationMatrix();
      original.getRotation(rotation);

      transform.resetScale();

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
            assertTrue(transform.getElement(row, column) == rotation.getElement(row, column));
         assertTrue(transform.getElement(row, 3) == original.getElement(row, 3));
      }
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(42353L);
      AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform transform = new AffineTransform(original);

      transform.setTranslationToZero();

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
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(2342L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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
   public void testSet() throws Exception
   {
      Random random = new Random(34534L);
      AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform actual = new AffineTransform();

      { // Test set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
         actual.setIdentity();
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
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      { // Test set(AffineTransform other)
         actual.setIdentity();
         actual.set(expected);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      { // Test set(RigidBodyTransform rigidBodyTransform)
         RigidBodyTransform rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.setIdentity();
         actual.set(rigidBodyTransform);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(rigidBodyTransform.getElement(row, column) == actual.getElement(row, column));
      }

      { // Test set(DenseMatrix64F matrix)
         actual.setIdentity();
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      { // Test set(DenseMatrix64F matrix, int startRow, int startColumn)
         actual.setIdentity();
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4 + startRow, 4 + startColumn);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix, startRow, startColumn);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      { // Test set(double[] transformArray)
         actual.setIdentity();
         double[] transformArray = new double[16];
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               transformArray[4 * row + column] = expected.getElement(row, column);
            }
         }
         actual.set(transformArray);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      { // Test set(Matrix3DReadOnly rotationScaleMatrix, TupleReadOnly translation)
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(rotationMatrix, translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationScaleMatrix rotationScaleMatrix, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actual.set(rotationScaleMatrix, translation);
         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(Matrix3DReadOnly rotationMatrix, double scale, TupleReadOnly translation)
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random), scale);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set((Matrix3DReadOnly) rotationMatrix, scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(Matrix3DReadOnly rotationMatrix, double scalex, double scaley, double scalez, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double scalex = rotationScaleMatrix.getScaleX();
         double scaley = rotationScaleMatrix.getScaleY();
         double scalez = rotationScaleMatrix.getScaleZ();

         actual.set((Matrix3DReadOnly) rotationMatrix, scalex, scaley, scalez, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(Matrix3DReadOnly rotationMatrix, TupleReadOnly scales, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Tuple3DReadOnly scale = rotationScaleMatrix.getScale();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set((Matrix3DReadOnly) rotationMatrix, scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix, double scale, TupleReadOnly translation)
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random), scale);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(rotationMatrix, scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix, double scalex, double scaley, double scalez, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double scalex = rotationScaleMatrix.getScaleX();
         double scaley = rotationScaleMatrix.getScaleY();
         double scalez = rotationScaleMatrix.getScaleZ();

         actual.set(rotationMatrix, scalex, scaley, scalez, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix, TupleReadOnly scales, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Tuple3DReadOnly scale = rotationScaleMatrix.getScale();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(rotationMatrix, scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(AxisAngleReadOnly axisAngle, double scale, TupleReadOnly translation)
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random), scale);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(new AxisAngle(rotationMatrix), scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(AxisAngleReadOnly axisAngle, double scalex, double scaley, double scalez, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double scalex = rotationScaleMatrix.getScaleX();
         double scaley = rotationScaleMatrix.getScaleY();
         double scalez = rotationScaleMatrix.getScaleZ();

         actual.set(new AxisAngle(rotationMatrix), scalex, scaley, scalez, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(AxisAngleReadOnly axisAngle, TupleReadOnly scales, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Tuple3DReadOnly scale = rotationScaleMatrix.getScale();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(new AxisAngle(rotationMatrix), scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == actual.getElement(row, column));
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(QuaternionReadOnly quaternion, double scale, TupleReadOnly translation)
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random), scale);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(new Quaternion(rotationMatrix), scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(rotationScaleMatrix.getElement(row, column), actual.getElement(row, column), EPS);
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(QuaternionReadOnly quaternion, double scalex, double scaley, double scalez, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double scalex = rotationScaleMatrix.getScaleX();
         double scaley = rotationScaleMatrix.getScaleY();
         double scalez = rotationScaleMatrix.getScaleZ();

         actual.set(new Quaternion(rotationMatrix), scalex, scaley, scalez, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(rotationScaleMatrix.getElement(row, column), actual.getElement(row, column), EPS);
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }

      { // Test set(QuaternionReadOnly quaternion, TupleReadOnly scales, TupleReadOnly translation)
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
         Tuple3DReadOnly scale = rotationScaleMatrix.getScale();
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);

         actual.set(new Quaternion(rotationMatrix), scale, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertEquals(rotationScaleMatrix.getElement(row, column), actual.getElement(row, column), EPS);
            assertTrue(translation.getElement(row) == actual.getElement(row, 3));
         }
      }
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(42523L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      RotationScaleMatrixReadOnly actualRotationScale = transform.getRotationScaleMatrix();
      Vector3DReadOnly actualTranslation = transform.getTranslationVector();
      RotationScaleMatrix expectedRotationScale = new RotationScaleMatrix();
      Vector3D expectedTranslation = new Vector3D();
      transform.getTranslation(expectedTranslation);

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(axisAngle);
         transform.setRotation(axisAngle);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotation(VectorReadOnly rotationVector)
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(rotationVector);
         transform.setRotation(rotationVector);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotation(DenseMatrix64F matrix)
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         rotationMatrix.get(denseMatrix);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(rotationMatrix);
         transform.setRotation(denseMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(quaternion);
         transform.setRotation(quaternion);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotation(Matrix3DReadOnly rotationMatrix)
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(rotationMatrix);
         transform.setRotation((Matrix3DReadOnly) rotationMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotation(RotationMatrix rotationMatrix)
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotation(rotationMatrix);
         transform.setRotation(rotationMatrix);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationYaw(double yaw)
         double yaw = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationYaw(yaw);
         transform.setRotationYaw(yaw);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationPitch(double pitch)
         double pitch = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationPitch(pitch);
         transform.setRotationPitch(pitch);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationRoll(double roll)
         double roll = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationRoll(roll);
         transform.setRotationRoll(roll);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationYawPitchRoll(double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random);
         double roll = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationYawPitchRoll(yaw, pitch, roll);
         transform.setRotationYawPitchRoll(yaw, pitch, roll);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationYawPitchRoll(double[] yawPitchRoll)
         double yaw = EuclidCoreRandomTools.nextDouble(random);
         double pitch = EuclidCoreRandomTools.nextDouble(random);
         double roll = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationYawPitchRoll(yaw, pitch, roll);
         transform.setRotationYawPitchRoll(new double[] {yaw, pitch, roll});
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationEuler(Vector3DReadOnly eulerAngles)
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationEuler(eulerAngles);
         transform.setRotationEuler(eulerAngles);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setRotationEuler(double rotX, double rotY, double rotZ)
         double rotX = EuclidCoreRandomTools.nextDouble(random);
         double rotY = EuclidCoreRandomTools.nextDouble(random);
         double rotZ = EuclidCoreRandomTools.nextDouble(random);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setRotationEuler(rotX, rotY, rotZ);
         transform.setRotationEuler(rotX, rotY, rotZ);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testSetScale() throws Exception
   {
      Random random = new Random(42523L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      RotationScaleMatrixReadOnly actualRotationScale = transform.getRotationScaleMatrix();
      Vector3DReadOnly actualTranslation = transform.getTranslationVector();
      RotationScaleMatrix expectedRotationScale = new RotationScaleMatrix();
      Vector3D expectedTranslation = new Vector3D();
      transform.getTranslation(expectedTranslation);

      { // Test setScale(double scale)
         double scale = 10.0 * random.nextDouble();
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setScale(scale);
         transform.setScale(scale);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setScale(double scalex, double scaley, double scalez)
         double scaleX = 10.0 * random.nextDouble();
         double scaleY = 10.0 * random.nextDouble();
         double scaleZ = 10.0 * random.nextDouble();
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setScale(scaleX, scaleY, scaleZ);
         transform.setScale(scaleX, scaleY, scaleZ);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setScale(TupleReadOnly scales)
         Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
         transform.getRotationScale(expectedRotationScale);
         expectedRotationScale.setScale(scale);
         transform.setScale(scale);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(42523L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      RotationScaleMatrixReadOnly actualRotationScale = transform.getRotationScaleMatrix();
      Vector3DReadOnly actualTranslation = transform.getTranslationVector();
      RotationScaleMatrix expectedRotationScale = new RotationScaleMatrix();
      Vector3D expectedTranslation = new Vector3D();
      transform.getRotationScale(expectedRotationScale);

      { // Test individual setTranslation(X/Y/Z)(double)
         double x = 2.0 * random.nextDouble() - 1.0;
         double y = 2.0 * random.nextDouble() - 1.0;
         double z = 2.0 * random.nextDouble() - 1.0;
         expectedTranslation.set(x, y, z);
         transform.setTranslationX(expectedTranslation.getX());
         transform.setTranslationY(expectedTranslation.getY());
         transform.setTranslationZ(expectedTranslation.getZ());
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         for (int row = 0; row < 3; row++)
         {
            assertTrue(expectedTranslation.getElement(row) == transform.getElement(row, 3));
         }
      }

      { // Test setTranslation(double x, double y, double z)
         double x = 2.0 * random.nextDouble() - 1.0;
         double y = 2.0 * random.nextDouble() - 1.0;
         double z = 2.0 * random.nextDouble() - 1.0;
         transform.setTranslation(x, y, z);
         expectedTranslation.set(x, y, z);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         expectedTranslation = EuclidCoreRandomTools.nextRotationVector(random);
         transform.setTranslation(expectedTranslation);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         expectedTranslation = EuclidCoreRandomTools.nextRotationVector(random);
         Vector3D transformTranslationPart = new Vector3D(transform.getTranslationVector());
         transform.addTranslation(expectedTranslation);
         expectedTranslation.add(transformTranslationPart);
         EuclidCoreTestTools.assertMatrix3DEquals(expectedRotationScale, actualRotationScale, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testAppendTranslation() throws Exception
   {
      Random random = new Random(35454L);

      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(double x, double y, double z)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(x, y, z);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(Tuple3DReadOnly translation)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(translation);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationScaleMatrix expectedRotation = new RotationScaleMatrix(original.getRotationScaleMatrix());
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendYawRotation(yaw);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationScaleMatrix expectedRotation = new RotationScaleMatrix(original.getRotationScaleMatrix());
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendPitchRotation(pitch);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendRollRotation(double roll)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationScaleMatrix expectedRotation = new RotationScaleMatrix(original.getRotationScaleMatrix());
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendRollRotation(roll);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependTranslation() throws Exception
   {
      Random random = new Random(35454L);

      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(double x, double y, double z)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(x, y, z);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(Tuple3DReadOnly translation)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(translation);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform yawTransform = new AffineTransform();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         yawTransform.setRotationYaw(yaw);
         expected.set(original);
         expected.preMultiply(yawTransform);

         actual.set(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform pitchTransform = new AffineTransform();

         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         pitchTransform.setRotationPitch(pitch);
         expected.set(original);
         expected.preMultiply(pitchTransform);

         actual.set(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependRollRotation(double roll)
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform rollTransform = new AffineTransform();

         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         rollTransform.setRotationRoll(roll);
         expected.set(original);
         expected.preMultiply(rollTransform);

         actual.set(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(465416L);

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform t2 = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform t3 = new AffineTransform(t2);
         t3.preMultiply(t1);

         DenseMatrix64F m1 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m2 = new DenseMatrix64F(4, 4);
         DenseMatrix64F m3 = new DenseMatrix64F(4, 4);
         t1.get(m1);
         t2.get(m2);
         CommonOps.mult(m1, m2, m3);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
      }
   }

   @Test
   public void testPreMultiplyWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         original.getRigidBodyTransform(expectedRigidBody);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);
         expected.setScale(original.getScale());

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformWithTuple() throws Exception
   {
      Random random = new Random(432L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      transform.get(matrix);

      { // Test transform(PointBasics pointToTransform)
         DenseMatrix64F ejmlPoint = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedPoint = new DenseMatrix64F(4, 1);

         Point3D point = EuclidCoreRandomTools.nextPoint3D(random);
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point);
         CommonOps.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), point.getElement(i), EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         DenseMatrix64F ejmlPoint = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedPoint = new DenseMatrix64F(4, 1);

         Point3D point = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D pointTransformed = new Point3D();
         point.get(ejmlPoint);
         ejmlPoint.set(3, 0, 1.0);

         transform.transform(point, pointTransformed);
         CommonOps.mult(matrix, ejmlPoint, ejmlTransformedPoint);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedPoint.get(i, 0), pointTransformed.getElement(i), EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         DenseMatrix64F ejmlVector = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedVector = new DenseMatrix64F(4, 1);

         Vector3D vector = EuclidCoreRandomTools.nextVector3D(random);
         vector.get(ejmlVector);

         transform.transform(vector);
         CommonOps.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vector.getElement(i), EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         DenseMatrix64F ejmlVector = new DenseMatrix64F(4, 1);
         DenseMatrix64F ejmlTransformedVector = new DenseMatrix64F(4, 1);

         Vector3D vector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorTransformed = new Vector3D();
         vector.get(ejmlVector);

         transform.transform(vector, vectorTransformed);
         CommonOps.mult(matrix, ejmlVector, ejmlTransformedVector);

         for (int i = 0; i < 3; i++)
            assertEquals(ejmlTransformedVector.get(i, 0), vectorTransformed.getElement(i), EPS);
      }
   }

   @Test
   public void testTransformWithQuaternion() throws Exception
   {
      Random random = new Random(34534L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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
      DenseMatrix64F transformDenseMatrix = new DenseMatrix64F(4, 4);
      transform.get(transformDenseMatrix);
      DenseMatrix64F vectorOriginalDenseMatrix = new DenseMatrix64F(4, 1);
      vectorOriginal.get(vectorOriginalDenseMatrix);
      DenseMatrix64F vectorTransformedDenseMatrix = new DenseMatrix64F(4, 1);
      CommonOps.mult(transformDenseMatrix, vectorOriginalDenseMatrix, vectorTransformedDenseMatrix);
      vectorExpected.set(vectorTransformedDenseMatrix);

      EuclidCoreTestTools.assertTuple4DEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformWithTuple2D() throws Exception
   {
      Random random = new Random(4353L);
      AffineTransform transfom2D = new AffineTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      transfom2D.setScale(random.nextDouble(), random.nextDouble(), 1.0);

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
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      Matrix3D matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      {
         Matrix3D m = new Matrix3D();
         transform.getRotationScale(m);
         matrixExpected.set(matrixOriginal);
         matrixExpected.preMultiply(m);
         matrixExpected.multiplyInvertOther(m);
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
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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

      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      transform.getRigidBodyTransform(expected);
      expected.multiply(original);

      transform.transform(original, actual);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform();
      transform.getRigidBodyTransform(inverse);
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

      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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

      RigidBodyTransform inverse = new RigidBodyTransform();
      transform.getRigidBodyTransform(inverse);
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

      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
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

      RigidBodyTransform inverse = new RigidBodyTransform();
      transform.getRigidBodyTransform(inverse);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(2342L);

      { // Test getRigidBodyTransform(RigidBodyTransform rigidBodyTransformToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         transform.getRigidBodyTransform(rigidBodyTransform);
         EuclidCoreTestTools.assertMatrix3DEquals(rigidBodyTransform.getRotation(), transform.getRotationMatrix(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(rigidBodyTransform.getTranslation(), transform.getTranslationVector(), EPS);
      }

      { // Test get(DenseMatrix64F matrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(4, 4, random);
         transform.get(denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DenseMatrix64F denseMatrix = RandomMatrices.createRandom(4 + startRow, 4 + startColumn, random);
         transform.get(startRow, startColumn, denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row + startRow, column + startColumn) == transform.getElement(row, column));
      }

      { // Test get(double[] transformArrayToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         double[] transformArray = new double[16];
         transform.get(transformArray);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(transformArray[4 * row + column] == transform.getElement(row, column));
      }

      { // Test get(Matrix3DBasics rotationScaleMarixToPack, TupleBasics translationToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Matrix3D expectedMatrix = new Matrix3D();
         Matrix3D actualMatrix = new Matrix3D();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotationScale(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }

      { // Test get(RotationScaleMatrix rotationScaleMarixToPack, TupleBasics translationToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationScaleMatrix expectedMatrix = new RotationScaleMatrix();
         RotationScaleMatrix actualMatrix = new RotationScaleMatrix();
         Vector3D expectedTranslation = new Vector3D();
         Vector3D actualTranslation = new Vector3D();
         transform.getRotationScale(expectedMatrix);
         transform.getTranslation(expectedTranslation);
         transform.get(actualMatrix, actualTranslation);
         assertEquals(expectedMatrix, actualMatrix);
         assertEquals(expectedTranslation, actualTranslation);
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(2345L);
      RotationMatrix rotationMatrix = new RotationMatrix();

      { // Test getRotation(Matrix3DBasics rotationMatrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.getRotation((CommonMatrix3DBasics) rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getRotationMatrix().getElement(row, column));
      }

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.getRotation(rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getRotationMatrix().getElement(row, column));
      }

      { // Test getRotation(DenseMatrix64F matrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         transform.getRotation(denseMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getRotationMatrix().getElement(row, column));
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Quaternion quaternion = new Quaternion();
         transform.getRotation(quaternion);
         rotationMatrix.set(quaternion);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getRotationMatrix().getElement(row, column), EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         rotationMatrix.set(axisAngle);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getRotationMatrix().getElement(row, column), EPS);
      }

      { // Test getRotation(VectorBasics rotationVectorToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3D rotationVector = new Vector3D();
         transform.getRotation(rotationVector);
         rotationMatrix.setRotationVector(rotationVector);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getRotationMatrix().getElement(row, column), EPS);
      }

      { // Test getRotationEuler(Tuple3DBasics eulerAnglesToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3D eulerAngles = new Vector3D();
         transform.getRotationEuler(eulerAngles);
         rotationMatrix.setEuler(eulerAngles);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getRotationMatrix().getElement(row, column), EPS);
      }

      { // Test getRotationYawPitchRoll(double[] yawPitchRollToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         double[] yawPitchRoll = new double[3];
         transform.getRotationYawPitchRoll(yawPitchRoll);
         rotationMatrix.setYawPitchRoll(yawPitchRoll);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertEquals(rotationMatrix.getElement(row, column), transform.getRotationMatrix().getElement(row, column), EPS);
      }
   }

   @Test
   public void testGetRotationScale() throws Exception
   {
      Random random = new Random(2345L);
      RotationScaleMatrix rotationMatrix = new RotationScaleMatrix();

      { // Test getRotationScale(Matrix3DBasics rotationMatrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.getRotationScale((CommonMatrix3DBasics) rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotationScale(RotationMatrix rotationMatrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.getRotationScale(rotationMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotationScale(RotationScaleMatrix rotationMatrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.getRotationScale(rotationScaleMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(rotationScaleMatrix.getElement(row, column) == transform.getElement(row, column));
      }

      { // Test getRotationScale(DenseMatrix64F matrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
         transform.getRotationScale(denseMatrix);
         for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(2345L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);
      for (int row = 0; row < 3; row++)
         assertTrue(translation.getElement(row) == transform.getElement(row, 3));

      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);

      translation.set(transform.getTranslationX(), transform.getTranslationY(), transform.getTranslationZ());
      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testGetScale() throws Exception
   {
      Random random = new Random(324L);
      Vector3D scales = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);
      AffineTransform affineTransform = EuclidCoreRandomTools.nextAffineTransform(random);
      affineTransform.setScale(scales);
      EuclidCoreTestTools.assertTuple3DEquals(scales, affineTransform.getScale(), EPS);

      assertEquals(scales.getX(), affineTransform.getScaleX(), EPS);
      assertEquals(scales.getY(), affineTransform.getScaleY(), EPS);
      assertEquals(scales.getZ(), affineTransform.getScaleZ(), EPS);

      Vector3D actualScales = new Vector3D();
      affineTransform.getScale(actualScales);
      EuclidCoreTestTools.assertTuple3DEquals(scales, actualScales, EPS);
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(5464L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      RotationScaleMatrixReadOnly rotationScaleMatrix = transform.getRotationScaleMatrix();
      Vector3DReadOnly translation = transform.getTranslationVector();

      assertTrue(rotationScaleMatrix.getM00() == transform.getM00());
      assertTrue(rotationScaleMatrix.getM01() == transform.getM01());
      assertTrue(rotationScaleMatrix.getM02() == transform.getM02());
      assertTrue(rotationScaleMatrix.getM10() == transform.getM10());
      assertTrue(rotationScaleMatrix.getM11() == transform.getM11());
      assertTrue(rotationScaleMatrix.getM12() == transform.getM12());
      assertTrue(rotationScaleMatrix.getM20() == transform.getM20());
      assertTrue(rotationScaleMatrix.getM21() == transform.getM21());
      assertTrue(rotationScaleMatrix.getM22() == transform.getM22());

      assertTrue(rotationScaleMatrix.getM00() == transform.getElement(0, 0));
      assertTrue(rotationScaleMatrix.getM01() == transform.getElement(0, 1));
      assertTrue(rotationScaleMatrix.getM02() == transform.getElement(0, 2));
      assertTrue(rotationScaleMatrix.getM10() == transform.getElement(1, 0));
      assertTrue(rotationScaleMatrix.getM11() == transform.getElement(1, 1));
      assertTrue(rotationScaleMatrix.getM12() == transform.getElement(1, 2));
      assertTrue(rotationScaleMatrix.getM20() == transform.getElement(2, 0));
      assertTrue(rotationScaleMatrix.getM21() == transform.getElement(2, 1));
      assertTrue(rotationScaleMatrix.getM22() == transform.getElement(2, 2));

      assertTrue(translation.getElement(0) == transform.getM03());
      assertTrue(translation.getElement(1) == transform.getM13());
      assertTrue(translation.getElement(2) == transform.getM23());

      assertTrue(translation.getElement(0) == transform.getElement(0, 3));
      assertTrue(translation.getElement(1) == transform.getElement(1, 3));
      assertTrue(translation.getElement(2) == transform.getElement(2, 3));

      assertTrue(transform.getM30() == 0.0);
      assertTrue(transform.getM31() == 0.0);
      assertTrue(transform.getM32() == 0.0);
      assertTrue(transform.getM33() == 1.0);

      assertTrue(transform.getElement(3, 0) == 0.0);
      assertTrue(transform.getElement(3, 1) == 0.0);
      assertTrue(transform.getElement(3, 2) == 0.0);
      assertTrue(transform.getElement(3, 3) == 1.0);

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
      AffineTransform t1 = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform t2 = new AffineTransform();

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
      AffineTransform t1 = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform t2 = new AffineTransform();
      double epsilon = 1.0e-3;
      double[] rot = new double[9];
      Vector3D translation = new Vector3D();

      assertFalse(t1.epsilonEquals(t2, epsilon));
      t2.set(t1);
      assertTrue(t1.epsilonEquals(t2, epsilon));

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.getRotation(rot);
            rot[3 * row + column] += 0.999 * epsilon;
            ((RotationMatrix) t2.getRotationMatrix()).setUnsafe(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.getRotation(rot);
            rot[3 * row + column] += 1.001 * epsilon;
            ((RotationMatrix) t2.getRotationMatrix()).setUnsafe(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
            assertFalse(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.getRotation(rot);
            rot[3 * row + column] -= 0.999 * epsilon;
            ((RotationMatrix) t2.getRotationMatrix()).setUnsafe(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
            assertTrue(t1.epsilonEquals(t2, epsilon));

            t2.set(t1);
            assertTrue(t1.epsilonEquals(t2, epsilon));
            t1.getRotation(rot);
            rot[3 * row + column] -= 1.001 * epsilon;
            ((RotationMatrix) t2.getRotationMatrix()).setUnsafe(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
            assertFalse(t1.epsilonEquals(t2, epsilon));
         }
      }

      for (int row = 0; row < 3; row++)
      {
         t2.set(t1);
         assertTrue(t1.epsilonEquals(t2, epsilon));
         t1.getTranslation(translation);
         translation.setElement(row, translation.getElement(row) + 0.999 * epsilon);
         t2.setTranslation(translation);
         assertTrue(t1.epsilonEquals(t2, epsilon));

         t2.set(t1);
         assertTrue(t1.epsilonEquals(t2, epsilon));
         t1.getTranslation(translation);
         translation.setElement(row, translation.getElement(row) + 1.001 * epsilon);
         t2.setTranslation(translation);
         assertFalse(t1.epsilonEquals(t2, epsilon));

         t2.set(t1);
         assertTrue(t1.epsilonEquals(t2, epsilon));
         t1.getTranslation(translation);
         translation.setElement(row, translation.getElement(row) - 0.999 * epsilon);
         t2.setTranslation(translation);
         assertTrue(t1.epsilonEquals(t2, epsilon));

         t2.set(t1);
         assertTrue(t1.epsilonEquals(t2, epsilon));
         t1.getTranslation(translation);
         translation.setElement(row, translation.getElement(row) - 1.001 * epsilon);
         t2.setTranslation(translation);
         assertFalse(t1.epsilonEquals(t2, epsilon));
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(54321L);

      AffineTransform affA;
      AffineTransform affB;
      RotationMatrix rmA;
      RotationMatrix rmB;
      Vector3D scaleA;
      Vector3D scaleB;
      Vector3D translationA;
      Vector3D translationB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         double angleEps = epsilon * 0.99;

         rmA = EuclidCoreRandomTools.nextRotationMatrix(random);

         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps);

         rmB = new RotationMatrix(aa);
         rmB.preMultiply(rmA);

         scaleA = EuclidCoreRandomTools.nextVector3D(random, 0.0, 2.0);
         scaleB = new Vector3D(scaleA);

         translationA = EuclidCoreRandomTools.nextVector3D(random);
         translationB = new Vector3D(translationA);

         affA = new AffineTransform(new RotationScaleMatrix(rmA, scaleA), translationA);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         double angleEps = epsilon * 1.01;

         rmA = EuclidCoreRandomTools.nextRotationMatrix(random);

         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps);

         rmB = new RotationMatrix(aa);
         rmB.preMultiply(rmA);

         scaleA = EuclidCoreRandomTools.nextVector3D(random, 0.0, 2.0);
         scaleB = new Vector3D(scaleA);

         translationA = EuclidCoreRandomTools.nextVector3D(random);
         translationB = new Vector3D(translationA);

         affA = new AffineTransform(new RotationScaleMatrix(rmA, scaleA), translationA);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();

         rmA = EuclidCoreRandomTools.nextRotationMatrix(random);
         rmB = new RotationMatrix(rmA);

         scaleA = EuclidCoreRandomTools.nextVector3D(random, 0.0, 2.0);
         scaleB = new Vector3D(scaleA);

         translationA = EuclidCoreRandomTools.nextVector3D(random);
         translationB = new Vector3D(translationA);

         affA = new AffineTransform(new RotationScaleMatrix(rmA, scaleA), translationA);

         scaleB.setX(scaleA.getX() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));

         scaleB.setX(scaleA.getX() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));

         scaleB = new Vector3D(scaleA);
         scaleB.setY(scaleA.getY() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));

         scaleB.setY(scaleA.getY() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));

         scaleB = new Vector3D(scaleA);
         scaleB.setZ(scaleA.getZ() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));

         scaleB.setZ(scaleA.getZ() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();

         rmA = EuclidCoreRandomTools.nextRotationMatrix(random);
         rmB = new RotationMatrix(rmA);

         scaleA = EuclidCoreRandomTools.nextVector3D(random, 0.0, 2.0);
         scaleB = new Vector3D(scaleA);

         translationA = EuclidCoreRandomTools.nextVector3D(random);
         translationB = new Vector3D(translationA);

         affA = new AffineTransform(new RotationScaleMatrix(rmA, scaleA), translationA);

         translationB.setX(translationA.getX() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));

         translationB.setX(translationA.getX() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));

         translationB = new Vector3D(translationA);
         translationB.setY(translationA.getY() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));

         translationB.setY(translationA.getY() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));

         translationB = new Vector3D(translationA);
         translationB.setZ(translationA.getZ() + 0.9 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertTrue(affA.geometricallyEquals(affB, epsilon));
         assertTrue(affB.geometricallyEquals(affA, epsilon));

         translationB.setZ(translationA.getZ() + 1.1 * epsilon);
         affB = new AffineTransform(new RotationScaleMatrix(rmB, scaleB), translationB);

         assertFalse(affA.geometricallyEquals(affB, epsilon));
         assertFalse(affB.geometricallyEquals(affA, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(12345L);

      RotationScaleMatrix rsm;
      Vector3D translation;
      AffineTransform affine = EuclidCoreRandomTools.nextAffineTransform(random);

      int newHashCode, previousHashCode;
      newHashCode = affine.hashCode();
      assertEquals(newHashCode, affine.hashCode());

      previousHashCode = affine.hashCode();

      for (int i = 0; i < ITERATIONS; ++i)
      {
         rsm = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 2.0);
         translation = EuclidCoreRandomTools.nextVector3D(random);
         affine = new AffineTransform(rsm, translation);
         newHashCode = affine.hashCode();
         assertNotEquals(previousHashCode, newHashCode);

         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testToString() throws Exception
   {
      Random random = new Random(12345L);

      AffineTransform affA;
      AffineTransform affB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         affA = EuclidCoreRandomTools.nextAffineTransform(random);
         affB = EuclidCoreRandomTools.nextAffineTransform(random);

         assertNotEquals(affA.toString(), affB.toString());
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         affA = EuclidCoreRandomTools.nextAffineTransform(random);
         affB = new AffineTransform(affA);

         assertEquals(affA.toString(), affB.toString());
      }
   }

   @Override
   public AffineTransform createRandomTransform(Random random)
   {
      return EuclidCoreRandomTools.nextAffineTransform(random);
   }

   @Override
   public AffineTransform createRandomTransform2D(Random random)
   {
      AffineTransform transfom2D = new AffineTransform();
      transfom2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      transfom2D.setScale(random.nextDouble(), random.nextDouble(), 1.0);
      return transfom2D;
   }
}

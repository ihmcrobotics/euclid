package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);

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
         Matrix3D rotationPart = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         AffineTransform transform = new AffineTransform(rotationPart, translation);

         for (int row = 0; row < 3; row++)
         {
            for (int column = 0; column < 3; column++)
               assertTrue(rotationPart.getElement(row, column) == transform.getElement(row, column));
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
            if (row == 3 && column == 3)
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
   public void testContainsNaN() throws Exception
   {
      AffineTransform transform = new AffineTransform();
      assertFalse(transform.containsNaN());
      transform.getLinearTransform().setToNaN();
      assertTrue(transform.containsNaN());
      transform.setIdentity();
      assertFalse(transform.containsNaN());
      transform.getTranslation().setToNaN();
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testLinearTransformToIdentity() throws Exception
   {
      Random random = new Random(42353L);
      AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
      AffineTransform transform = new AffineTransform(original);

      transform.setLinearTransformToIdentity();

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
   public void testResetScale() throws Exception
   {
      Random random = new Random(42353L);
      RotationMatrix rotation = EuclidCoreRandomTools.nextRotationMatrix(random);
      AffineTransform transform = new AffineTransform(rotation, EuclidCoreRandomTools.nextPoint3D(random));
      double scaleX = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
      double scaleY = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
      double scaleZ = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
      EuclidCoreTestTools.assertMatrix3DEquals(rotation, new RotationMatrix(transform.getRotationView()), EPS);
      transform.getLinearTransform().scaleColumns(scaleX, scaleY, scaleZ);
      EuclidCoreTestTools.assertMatrix3DEquals(rotation, new RotationMatrix(transform.getRotationView()), EPS);

      transform.resetScale();

      EuclidCoreTestTools.assertMatrix3DEquals(rotation, transform.getLinearTransform(), EPS);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      { // Test set(AffineTransform other)
         actual.setIdentity();
         actual.set(expected);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      { // Test set(RigidBodyTransform rigidBodyTransform)
         RigidBodyTransform rigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.setIdentity();
         actual.set(rigidBodyTransform);

         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(rigidBodyTransform.getElement(row, column) == actual.getElement(row, column));
      }

      { // Test set(DMatrix matrix)
         actual.setIdentity();
         DMatrix denseMatrix = new DMatrixRMaj(4, 4);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row, column, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      { // Test set(DMatrix matrix, int startRow, int startColumn)
         actual.setIdentity();
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DMatrix denseMatrix = new DMatrixRMaj(4 + startRow, 4 + startColumn);
         for (int row = 0; row < 4; row++)
         {
            for (int column = 0; column < 4; column++)
            {
               denseMatrix.set(row + startRow, column + startColumn, expected.getElement(row, column));
            }
         }
         actual.set(denseMatrix, startRow, startColumn);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
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
   }

   @Test
   public void testSetLinearTransform() throws Exception
   {
      Random random = new Random(42523L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setLinearTransform(Matrix3DReadOnly linearTransform)
         Matrix3D expected = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.setLinearTransform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, transform.getLinearTransform(), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setLinearTransform(DMatrix linearTransform)
         DMatrix expected = EuclidCoreRandomTools.nextDMatrixRMaj(random, 3, 3, 10.0);
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.setLinearTransform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(new Matrix3D(expected), transform.getLinearTransform(), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setLinearTransform(RotationMatrixReadOnly rotationMatrix)
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.setLinearTransform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, transform.getLinearTransform(), EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setLinearTransform(Orientation3DReadOnly orientation)
         Orientation3DReadOnly expected = EuclidCoreRandomTools.nextOrientation3D(random);
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         transform.setLinearTransform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(new RotationMatrix(expected), transform.getLinearTransform(), EPS);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(42523L);

      { // Test individual setTranslation(X/Y/Z)(double)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         transform.setTranslationX(expected.getX());
         transform.setTranslationY(expected.getY());
         transform.setTranslationZ(expected.getZ());
         EuclidCoreTestTools.assertEquals(expected, transform.getTranslation(), EPS);
      }

      { // Test setTranslation(double x, double y, double z)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         transform.setTranslation(expected.getX(), expected.getY(), expected.getZ());
         EuclidCoreTestTools.assertEquals(expected, transform.getTranslation(), EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         transform.setTranslation(expected);
         EuclidCoreTestTools.assertEquals(expected, transform.getTranslation(), EPS);
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
         AffineTransform original = createRandomTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(x, y, z);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(Tuple3DReadOnly translation)
         AffineTransform original = createRandomTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(translation);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testAppendOrientation() throws Exception
   {
      Random random = new Random(3456);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         AffineTransform orientationTransform = new AffineTransform(orientation, new Vector3D());

         AffineTransform expected = new AffineTransform();
         expected.set(original);
         expected.multiply(orientationTransform);

         AffineTransform actual = new AffineTransform();
         actual.set(original);
         actual.appendOrientation(orientation);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
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
         AffineTransform original = createRandomTransform(random);
         Matrix3D expectedRotation = new Matrix3D(original.getLinearTransform());
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.multiply(new RotationMatrix(yaw, 0, 0));
         expected.set(expectedRotation, original.getTranslation());

         actual.set(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         AffineTransform original = createRandomTransform(random);
         Matrix3D expectedRotation = new Matrix3D(original.getLinearTransform());
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.multiply(new RotationMatrix(0, pitch, 0));
         expected.set(expectedRotation, original.getTranslation());

         actual.set(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendRollRotation(double roll)
         AffineTransform original = createRandomTransform(random);
         Matrix3D expectedRotation = new Matrix3D(original.getLinearTransform());
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.multiply(new RotationMatrix(0, 0, roll));
         expected.set(expectedRotation, original.getTranslation());

         actual.set(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
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
         AffineTransform original = createRandomTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(x, y, z);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(Tuple3DReadOnly translation)
         AffineTransform original = createRandomTransform(random);
         AffineTransform translationTransform = new AffineTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(translation);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependOrientation() throws Exception
   {
      Random random = new Random(3456);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = EuclidCoreRandomTools.nextAffineTransform(random);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         AffineTransform orientationTransform = new AffineTransform(orientation, new Vector3D());

         AffineTransform expected = new AffineTransform();
         expected.set(original);
         expected.preMultiply(orientationTransform);

         AffineTransform actual = new AffineTransform();
         actual.set(original);
         actual.prependOrientation(orientation);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
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
         AffineTransform original = createRandomTransform(random);
         AffineTransform yawTransform = new AffineTransform();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         yawTransform.getLinearTransform().setToYawMatrix(yaw);
         expected.set(original);
         expected.preMultiply(yawTransform);

         actual.set(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         AffineTransform original = createRandomTransform(random);
         AffineTransform pitchTransform = new AffineTransform();

         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         pitchTransform.getLinearTransform().setToPitchMatrix(pitch);
         expected.set(original);
         expected.preMultiply(pitchTransform);

         actual.set(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependRollRotation(double roll)
         AffineTransform original = createRandomTransform(random);
         AffineTransform rollTransform = new AffineTransform();

         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         rollTransform.getLinearTransform().setToRollMatrix(roll);
         expected.set(original);
         expected.preMultiply(rollTransform);

         actual.set(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextNonSingularAffineTransform(random);
         AffineTransform inverse = new AffineTransform(transform);
         inverse.invert();

         assertTrue(transform.hasLinearTransform());
         assertTrue(transform.hasTranslation());
         transform.multiply(inverse);
         assertFalse(transform.hasLinearTransform());
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
         AffineTransform t1 = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform t2 = EuclidCoreRandomTools.nextAffineTransform(random);
         checkMultiplyAgainstEJML(t1, t2);
      }

      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         AffineTransform t2 = new AffineTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), new Vector3D());
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new AffineTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), new Vector3D());
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertFalse(t1.hasTranslation());

         t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         t2.setLinearTransform(t1.getLinearTransform());
         t2.getLinearTransform().invert();
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());

         t1 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new AffineTransform(EuclidCoreRandomTools.nextLinearTransform3D(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t1.getTranslation());
         negateTranslation.negate();
         t1.inverseTransform(negateTranslation);
         t2.setTranslation(negateTranslation);
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasLinearTransform());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasLinearTransform());
         assertFalse(t1.hasTranslation());
      }
   }

   private static void checkMultiplyAgainstEJML(AffineTransform t1, AffineTransform t2)
   {
      AffineTransform t3 = new AffineTransform(t1);
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
   public void testMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         checkMultiplyAgainstEJML(original, multipliedWith);
      }
   }

   private static void checkMultiplyAgainstEJML(AffineTransform t1, RigidBodyTransform t2)
   {
      AffineTransform t3 = new AffineTransform(t1);
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
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         checkMultiplyAgainstEJML(original, multipliedWith);
      }
   }

   private static void checkMultiplyAgainstEJML(AffineTransform t1, QuaternionBasedTransform t2)
   {
      AffineTransform t3 = new AffineTransform(t1);
      t3.multiply(t2);

      DMatrixRMaj m1 = new DMatrixRMaj(4, 4);
      DMatrixRMaj m2 = new DMatrixRMaj(4, 4);
      DMatrixRMaj m3 = new DMatrixRMaj(4, 4);
      t1.get(m1);
      new RigidBodyTransform(t2).get(m2);
      CommonOps_DDRM.mult(m1, m2, m3);

      for (int row = 0; row < 4; row++)
         for (int column = 0; column < 4; column++)
            assertEquals(m3.get(row, column), t3.getElement(row, column), EPS);
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         AffineTransform multipliedWith = createRandomTransform(random);

         expected.set(original);
         expected.invert();
         expected.multiply(multipliedWith);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         AffineTransform multipliedWith = createRandomTransform(random);

         AffineTransform inverseOfMultipliedWith = new AffineTransform(multipliedWith);
         inverseOfMultipliedWith.invert();

         expected.set(original);
         expected.multiply(inverseOfMultipliedWith);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform affineMultipliedWith = new AffineTransform(multipliedWith);
         expected.set(original);
         expected.multiplyInvertThis(affineMultipliedWith);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals("Iteration " + i, expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform affineMultipliedWith = new AffineTransform(multipliedWith);

         expected.set(original);
         expected.multiplyInvertOther(affineMultipliedWith);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         AffineTransform affineMultipliedWith = new AffineTransform(multipliedWith);

         expected.set(original);
         expected.multiplyInvertThis(affineMultipliedWith);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         AffineTransform affineMultipliedWith = new AffineTransform(multipliedWith);

         expected.set(original);
         expected.multiplyInvertOther(affineMultipliedWith);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         AffineTransform multipliedWith = createRandomTransform(random);

         expected.set(multipliedWith);
         expected.multiply(original);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(465416L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.set(multipliedWith);
         expected.multiply(original);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);

      }
   }

   @Test
   public void testPreMultiplyWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expected.set(multipliedWith);
         expected.multiply(original);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         AffineTransform multipliedWith = createRandomTransform(random);

         expected.set(multipliedWith);
         expected.multiplyInvertOther(original);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         AffineTransform multipliedWith = createRandomTransform(random);

         expected.set(multipliedWith);
         expected.invert();
         expected.multiply(original);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.set(multipliedWith);
         expected.multiplyInvertOther(original);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expected.set(multipliedWith);
         expected.invert();
         expected.multiply(original);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expected.set(multipliedWith);
         expected.multiplyInvertOther(original);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform original = createRandomTransform(random);
         AffineTransform expected = createRandomTransform(random);
         AffineTransform actual = createRandomTransform(random);

         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expected.set(multipliedWith);
         expected.invert();
         expected.multiply(original);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformWithTuple() throws Exception
   {
      Random random = new Random(432L);
      AffineTransform transform = createRandomTransform(random);
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
      AffineTransform transform = createRandomTransform(random);
      Quaternion quaternionOriginal = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion quaternionExpected = new Quaternion();
      Quaternion quaternionActual = new Quaternion();

      quaternionExpected.set(transform.getLinearTransform().getAsQuaternion());
      quaternionExpected.multiply(quaternionOriginal);

      transform.transform(quaternionOriginal, quaternionActual);
      EuclidCoreTestTools.assertEquals(quaternionExpected, quaternionActual, EPS);

      quaternionActual.set(quaternionOriginal);
      transform.transform(quaternionActual);
      EuclidCoreTestTools.assertEquals(quaternionExpected, quaternionActual, EPS);
   }

   @Test
   public void testTransformWithVector4D() throws Exception
   {
      Random random = new Random(5634L);
      AffineTransform transform = createRandomTransform(random);
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
      EuclidCoreTestTools.assertEquals(vectorExpected, vectorActual, EPS);

      vectorActual.set(vectorOriginal);
      transform.transform(vectorActual);
      EuclidCoreTestTools.assertEquals(vectorExpected, vectorActual, EPS);

      // Try with EJML matrix
      DMatrixRMaj transformDenseMatrix = new DMatrixRMaj(4, 4);
      transform.get(transformDenseMatrix);
      DMatrixRMaj vectorOriginalDenseMatrix = new DMatrixRMaj(4, 1);
      vectorOriginal.get(vectorOriginalDenseMatrix);
      DMatrixRMaj vectorTransformedDenseMatrix = new DMatrixRMaj(4, 1);
      CommonOps_DDRM.mult(transformDenseMatrix, vectorOriginalDenseMatrix, vectorTransformedDenseMatrix);
      vectorExpected.set(vectorTransformedDenseMatrix);

      EuclidCoreTestTools.assertEquals(vectorExpected, vectorActual, EPS);
   }

   @Test
   public void testTransformWithTuple2D() throws Exception
   {
      Random random = new Random(4353L);
      AffineTransform transfom2D = new AffineTransform();
      transfom2D.getLinearTransform().setToYawMatrix(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      transfom2D.appendScale(random.nextDouble(), random.nextDouble(), 1.0);

      { // Test transform(Point2DBasics pointToTransform)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         pointActual.set(pointOriginal);
         transfom2D.transform(pointActual);
         EuclidCoreTestTools.assertEquals(pointExpected, pointActual, EPS);
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
         EuclidCoreTestTools.assertEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual);
         EuclidCoreTestTools.assertEquals(pointExpected, pointActual, EPS);
      }

      { // Test transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
         Point2D pointOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D pointExpected = new Point2D();
         Point2D pointActual = new Point2D();

         Point3D point = new Point3D(pointOriginal.getX(), pointOriginal.getY(), 0.0);
         transfom2D.transform(point);
         pointExpected.set(point.getX(), point.getY());

         transfom2D.transform(pointOriginal, pointActual, true);
         EuclidCoreTestTools.assertEquals(pointExpected, pointActual, EPS);
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
         EuclidCoreTestTools.assertEquals(vectorExpected, vectorActual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D vectorOriginal = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D vectorExpected = new Vector2D();
         Vector2D vectorActual = new Vector2D();

         Vector3D vector = new Vector3D(vectorOriginal.getX(), vectorOriginal.getY(), 0.0);
         transfom2D.transform(vector);
         vectorExpected.set(vector.getX(), vector.getY());

         transfom2D.transform(vectorOriginal, vectorActual);
         EuclidCoreTestTools.assertEquals(vectorExpected, vectorActual, EPS);
      }
   }

   @Test
   public void testTransformWithMatrix3D() throws Exception
   {
      Random random = new Random(4534L);
      AffineTransform transform = createRandomTransform(random);
      Matrix3D matrixOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
      Matrix3D matrixExpected = new Matrix3D();
      Matrix3D matrixActual = new Matrix3D();

      {
         Matrix3D m = new Matrix3D(transform.getLinearTransform());
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
      AffineTransform transform = createRandomTransform(random);
      RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
      RotationMatrix expected = new RotationMatrix();
      RotationMatrix actual = new RotationMatrix();

      transform.getLinearTransform().getAsQuaternion().transform(original, expected);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithOtherRigidBodyTransform() throws Exception
   {
      Random random = new Random(23423L);

      AffineTransform transform = createRandomTransform(random);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      expected.set(transform);
      expected.multiply(original);

      transform.transform(original, actual);

      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform();
      inverse.set(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(23423L);

      AffineTransform transform = createRandomTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      QuaternionBasedTransform original = new QuaternionBasedTransform(originalRigidBodyTransform);
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform();
      inverse.set(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithAffineTransform() throws Exception
   {
      Random random = new Random(23423L);

      AffineTransform transform = createRandomTransform(random);
      AffineTransform original = createRandomTransform(random);
      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      expected.set(original);
      expected.preMultiply(transform);
      transform.transform(original, actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPS);

      expected.set(original);
      expected.preMultiplyInvertOther(transform);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPS);
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(2342L);

      { // Test get(DMatrix matrixToPack)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         DMatrix denseMatrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 4, 4);
         transform.get(denseMatrix);
         for (int row = 0; row < 4; row++)
            for (int column = 0; column < 4; column++)
               assertTrue(denseMatrix.get(row, column) == transform.getElement(row, column));
      }

      { // Test get(DMatrix matrixToPack, int startRow, int startColumn)
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         int startRow = random.nextInt(10);
         int startColumn = random.nextInt(10);
         DMatrix denseMatrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, 4 + startRow, 4 + startColumn);
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
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(5464L);
      AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
      LinearTransform3D rotationScaleMatrix = transform.getLinearTransform();
      Vector3DReadOnly translation = transform.getTranslation();

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

      double smallestEpsilon = 1.0e-15;
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
      AffineTransform t = EuclidCoreRandomTools.nextAffineTransform(random);
      double m00 = t.getM00();
      double m01 = t.getM01();
      double m02 = t.getM02();
      double m10 = t.getM10();
      double m11 = t.getM11();
      double m12 = t.getM12();
      double m20 = t.getM20();
      double m21 = t.getM21();
      double m22 = t.getM22();
      double tx = t.getM03();
      double ty = t.getM13();
      double tz = t.getM23();

      double small = 0.999 * EPS;
      double big = 1.001 * EPS;

      assertTrue(t.epsilonEquals(createTransform(m00 + small, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01 + small, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02 + small, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10 + small, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11 + small, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12 + small, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 + small, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 + small, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 + small, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx + small, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty + small, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz + small), EPS));

      assertTrue(t.epsilonEquals(createTransform(m00 - small, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01 - small, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02 - small, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10 - small, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11 - small, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12 - small, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 - small, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 - small, m22, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 - small, tx, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx - small, ty, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty - small, tz), EPS));
      assertTrue(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz - small), EPS));

      assertFalse(t.epsilonEquals(createTransform(m00 + big, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01 + big, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02 + big, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10 + big, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11 + big, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12 + big, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 + big, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 + big, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 + big, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx + big, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty + big, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz + big), EPS));

      assertFalse(t.epsilonEquals(createTransform(m00 - big, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01 - big, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02 - big, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10 - big, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11 - big, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12 - big, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 - big, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 - big, m22, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 - big, tx, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx - big, ty, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty - big, tz), EPS));
      assertFalse(t.epsilonEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz - big), EPS));
   }

   private static AffineTransform createTransform(double m00,
                                                  double m01,
                                                  double m02,
                                                  double m10,
                                                  double m11,
                                                  double m12,
                                                  double m20,
                                                  double m21,
                                                  double m22,
                                                  double tx,
                                                  double ty,
                                                  double tz)
   {
      return new AffineTransform(new Matrix3D(m00, m01, m02, m10, m11, m12, m20, m21, m22), new Vector3D(tx, ty, tz));
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(2354L);
      AffineTransform t = EuclidCoreRandomTools.nextAffineTransform(random);
      double m00 = t.getM00();
      double m01 = t.getM01();
      double m02 = t.getM02();
      double m10 = t.getM10();
      double m11 = t.getM11();
      double m12 = t.getM12();
      double m20 = t.getM20();
      double m21 = t.getM21();
      double m22 = t.getM22();
      double tx = t.getM03();
      double ty = t.getM13();
      double tz = t.getM23();

      double small = 0.999 * EPS;
      double big = 1.001 * EPS;

      Vector3D d = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, small);
      double small_dx = d.getX();
      double small_dy = d.getY();
      double small_dz = d.getZ();
      d = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, big);
      double big_dx = d.getX();
      double big_dy = d.getY();
      double big_dz = d.getZ();

      assertTrue(t.geometricallyEquals(createTransform(m00 + small, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01 + small, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02 + small, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10 + small, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11 + small, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12 + small, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 + small, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 + small, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 + small, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx + small_dx, ty + small_dy, tz + small_dz), EPS));

      assertTrue(t.geometricallyEquals(createTransform(m00 - small, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01 - small, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02 - small, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10 - small, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11 - small, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12 - small, m20, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 - small, m21, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 - small, m22, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 - small, tx, ty, tz), EPS));
      assertTrue(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx - small_dx, ty - small_dy, tz - small_dz), EPS));

      assertFalse(t.geometricallyEquals(createTransform(m00 + big, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01 + big, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02 + big, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10 + big, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11 + big, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12 + big, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 + big, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 + big, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 + big, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx + big_dx, ty + big_dy, tz + big_dz), EPS));

      assertFalse(t.geometricallyEquals(createTransform(m00 - big, m01, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01 - big, m02, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02 - big, m10, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10 - big, m11, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11 - big, m12, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12 - big, m20, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20 - big, m21, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21 - big, m22, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22 - big, tx, ty, tz), EPS));
      assertFalse(t.geometricallyEquals(createTransform(m00, m01, m02, m10, m11, m12, m20, m21, m22, tx - big_dx, ty - big_dy, tz - big_dz), EPS));
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(12345L);

      AffineTransform affine = EuclidCoreRandomTools.nextAffineTransform(random);

      int newHashCode, previousHashCode;
      newHashCode = affine.hashCode();
      assertEquals(newHashCode, affine.hashCode());

      previousHashCode = affine.hashCode();

      for (int i = 0; i < ITERATIONS; ++i)
      {
         affine = EuclidCoreRandomTools.nextAffineTransform(random);
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
      AffineTransform next = EuclidCoreRandomTools.nextNonSingularAffineTransform(random);
      if (random.nextBoolean())
         next.getTranslation().setToZero();
      if (random.nextBoolean())
         next.getLinearTransform().setIdentity();
      else if (random.nextBoolean())
         next.getLinearTransform().resetScale();
      return next;
   }

   @Override
   public AffineTransform createRandomTransform2D(Random random)
   {
      AffineTransform transfom2D = new AffineTransform();
      transfom2D.getLinearTransform().setToYawMatrix(2.0 * Math.PI * random.nextDouble() - Math.PI);
      transfom2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      transfom2D.appendScale(random.nextDouble(), random.nextDouble(), 1.0);
      return transfom2D;
   }
}
